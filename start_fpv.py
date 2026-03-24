#!/usr/bin/env python3
"""FPV simulation launcher — Betaflight SITL + Gazebo camera → RTSP/TCP stream.

Starts the full stack and streams the on-board camera feed so any RTSP/TCP
video client (VLC, ffplay, Unreal Engine, Unity) can display it.

Architecture:
    Gazebo (camera sensor) ──gz-transport──► gz_image_bridge ──pipe──► ffmpeg
                                                              ──► RTSP/TCP stream

Usage (inside the Docker container):
    python3 start_fpv.py                      # TCP stream on :8554
    python3 start_fpv.py --rtsp               # RTSP via mediamtx on :8554
    python3 start_fpv.py --output file:out.mp4  # record to file

Viewer examples:
    ffplay tcp://<host>:8554                  # TCP mode (default)
    ffplay rtsp://<host>:8554/fpv             # RTSP mode (--rtsp)
    vlc rtsp://<host>:8554/fpv               # RTSP with VLC
"""

import argparse
import logging
import os
import signal
import socket
import subprocess
import sys
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("start_fpv")

# ── Defaults ──────────────────────────────────────────────────────────────────
AEROLOOP_HOME = os.environ.get("AEROLOOP_HOME", "/opt/aeroloop_gazebo")
BF_ELF = os.environ.get("BF_ELF", "/opt/betaflight/obj/main/betaflight_SITL.elf")
MSP_RADIO_HOME = os.environ.get("MSP_RADIO_HOME", "/opt/msp_virtualradio")
FPV_WORLD = "fpv_demo_harmonic.sdf"
IMAGE_BRIDGE = os.path.join(AEROLOOP_HOME, "plugins", "build", "gz_image_bridge")
STREAM_PORT = 8554


# ── Helpers ───────────────────────────────────────────────────────────────────

class ProcessManager:
    """Track child processes for clean shutdown."""

    def __init__(self):
        self.procs: list[subprocess.Popen] = []

    def spawn(self, args, **kwargs):
        log.info("Starting: %s", " ".join(args[:4]))
        p = subprocess.Popen(args, **kwargs)
        self.procs.append(p)
        return p

    def shutdown(self):
        log.info("Shutting down %d processes …", len(self.procs))
        for p in reversed(self.procs):
            if p.poll() is None:
                p.terminate()
        for p in reversed(self.procs):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()


def wait_for_port(host, port, timeout=30):
    """Block until a TCP port is accepting connections."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            s = socket.create_connection((host, port), timeout=1)
            s.close()
            return True
        except OSError:
            time.sleep(0.5)
    return False


def setup_gazebo_env():
    """Set Gazebo Harmonic environment variables (mirrors betaloop/start.py)."""

    def _prepend(var, *paths):
        existing = os.environ.get(var, "")
        os.environ[var] = os.pathsep.join(list(paths) + [existing])

    models = os.path.join(AEROLOOP_HOME, "models")
    plugins = os.path.join(AEROLOOP_HOME, "plugins", "build")
    worlds = os.path.join(AEROLOOP_HOME, "worlds")

    _prepend("SDF_PATH", models, "/usr/share/gz/gz-sim8/models")
    _prepend("GZ_SIM_RESOURCE_PATH", worlds, "/usr/share/gz/gz-sim8")
    _prepend("GZ_SIM_SYSTEM_PLUGIN_PATH", plugins, "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins")
    _prepend("LD_LIBRARY_PATH", "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins")

    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


def has_nvidia_gpu():
    """Check if an NVIDIA GPU is accessible inside this environment."""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=5,
        )
        return result.returncode == 0 and result.stdout.strip() != ""
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def _fix_dri_permissions():
    """Grant the current user access to /dev/dri render nodes.

    Inside Docker with --gpus all, the host GID on /dev/dri/renderD* often
    doesn't match any container group.  We chmod the nodes to be world-
    readable/writable (safe inside a single-user container).
    """
    import glob
    render_nodes = glob.glob("/dev/dri/renderD*") + glob.glob("/dev/dri/card*")
    for node in render_nodes:
        if os.access(node, os.R_OK | os.W_OK):
            continue
        log.info("Fixing permissions on %s", node)
        subprocess.run(["sudo", "chmod", "666", node], capture_output=True)


def discover_camera_topic(timeout=30):
    """List Gazebo topics and find the camera image topic."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True, text=True, timeout=10,
            )
            for line in result.stdout.strip().splitlines():
                line = line.strip()
                if line.endswith("/image") and "sensor" in line:
                    return line
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        time.sleep(2)
    return None


def read_image_meta(proc, timeout=30):
    """Read IMGMETA line from the bridge's stderr (width, height, pix_fmt)."""
    import select
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        ready, _, _ = select.select([proc.stderr], [], [], 1.0)
        if ready:
            try:
                chunk = proc.stderr.read(4096)
            except BlockingIOError:
                chunk = None
            if chunk is None or len(chunk) == 0:
                # Non-blocking read returned nothing — not EOF, just no data yet
                if proc.poll() is not None:
                    break  # Process exited — that's a real EOF
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode("utf-8", errors="replace").strip()
                if line.startswith("IMGMETA "):
                    parts = line.split()
                    return int(parts[1]), int(parts[2]), parts[3]
        if proc.poll() is not None:
            break
    return None, None, None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="FPV simulation launcher")
    parser.add_argument("--world", default=FPV_WORLD,
                        help=f"World SDF file (default: {FPV_WORLD})")
    parser.add_argument("--elf", default=BF_ELF,
                        help="Path to betaflight_SITL.elf")
    parser.add_argument("--port", type=int, default=STREAM_PORT,
                        help="Stream output port (default: 8554)")
    parser.add_argument("--rtsp", action="store_true",
                        help="Use RTSP streaming via mediamtx (otherwise TCP MPEG-TS)")
    parser.add_argument("--output", default=None,
                        help="Override output: 'tcp', 'rtsp', or 'file:<path>'")
    parser.add_argument("--gazebo", action="store_true",
                        help="Show the Gazebo GUI (default: headless)")
    parser.add_argument("--no-transmitter", action="store_true",
                        help="Skip starting the MSP virtual radio")
    parser.add_argument("--fps", type=int, default=30,
                        help="Output stream FPS (default: 30)")
    args = parser.parse_args()

    # Determine output mode
    if args.output:
        output_mode = args.output
    elif args.rtsp:
        output_mode = "rtsp"
    else:
        output_mode = "tcp"

    pm = ProcessManager()

    def on_signal(sig, frame):
        pm.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # ── 1. Environment ──
    log.info("Setting up Gazebo environment")
    setup_gazebo_env()

    # ── 1b. GPU detection & rendering setup ──
    gpu_available = has_nvidia_gpu()
    if gpu_available:
        log.info("NVIDIA GPU detected — using GPU-accelerated rendering")
        # Remove software rendering override — Mesa software path segfaults
        # when the NVIDIA container toolkit has replaced Gallium drivers.
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
        # Force both EGL and GLX to use the NVIDIA vendor ICD.
        # Without these, Mesa's DRI2 path is tried first and fails.
        nvidia_icd = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.isfile(nvidia_icd):
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_icd
        os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
        log.info("Forcing NVIDIA EGL + GLX vendor")
        # Ensure the current user can access /dev/dri render nodes.
        _fix_dri_permissions()
    else:
        log.info("No GPU detected — using software rendering (llvmpipe)")
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    # If headless with no DISPLAY, start Xvfb.  Ogre2's camera sensor needs
    # a display context to render frames — even with an NVIDIA GPU.
    # Xvfb provides the virtual display; the GPU still does the actual rendering.
    # When --gazebo (GUI) is requested, we use the host's real display instead.
    if args.gazebo:
        # GUI mode — need the host's real X display
        if not os.environ.get("DISPLAY"):
            log.error(
                "No DISPLAY set — the Gazebo GUI needs X11 forwarding from the host.\n"
                "  Run the container with:\n"
                "    -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"
            )
            sys.exit(1)
        # Don't force __GLX_VENDOR_LIBRARY_NAME when talking to the host X server —
        # it prevents GLX context creation through the forwarded socket.
        os.environ.pop("__GLX_VENDOR_LIBRARY_NAME", None)
        log.info("Using host display %s for Gazebo GUI", os.environ["DISPLAY"])
    else:
        # Headless mode — ALWAYS use Xvfb (even if DISPLAY is set from the host,
        # the host display causes NV-GLX BadValue errors for headless rendering).
        for lockfile in ["/tmp/.X99-lock", "/tmp/.X11-unix/X99"]:
            try:
                os.remove(lockfile)
            except OSError:
                pass
        subprocess.run(["pkill", "-9", "Xvfb"], capture_output=True)
        time.sleep(0.3)

        log.info("Starting Xvfb virtual display :99")
        xvfb = pm.spawn(
            ["Xvfb", ":99", "-screen", "0", "1280x720x24", "-ac"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(1)
        if xvfb.poll() is not None:
            log.error("Xvfb failed to start — camera rendering requires a display")
            sys.exit(1)
        os.environ["DISPLAY"] = ":99"

    # ── 2. RTSP server (optional) ──
    if output_mode == "rtsp":
        mediamtx = "/usr/local/bin/mediamtx"
        if not os.path.isfile(mediamtx):
            log.error("mediamtx not found at %s — install it or use --output tcp", mediamtx)
            sys.exit(1)
        log.info("Starting mediamtx RTSP server")
        pm.spawn([mediamtx], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)

    # ── 3. Gazebo ──
    world_path = args.world
    if not os.path.isabs(world_path):
        world_path = os.path.join(AEROLOOP_HOME, "worlds", world_path)
    if not os.path.isfile(world_path):
        log.error("World file not found: %s", world_path)
        sys.exit(1)

    gz_args = ["gz", "sim"]
    if not args.gazebo:
        gz_args.append("-s")  # headless by default
    gz_args.extend(["-r", "-v", "3", world_path])

    log.info("Starting Gazebo%s: %s", " (GUI)" if args.gazebo else " (headless)", os.path.basename(world_path))
    pm.spawn(gz_args)
    time.sleep(8)

    # ── 4. Betaflight SITL ──
    if not os.path.isfile(args.elf):
        log.error("Betaflight ELF not found: %s", args.elf)
        pm.shutdown()
        sys.exit(1)

    elf_dir = os.path.dirname(args.elf)
    log.info("Starting Betaflight SITL")
    pm.spawn([args.elf], cwd=elf_dir)

    log.info("Waiting for Betaflight CLI port (5761) …")
    if not wait_for_port("127.0.0.1", 5761, timeout=20):
        log.warning("Betaflight CLI port not ready — continuing anyway")
    time.sleep(3)

    # ── 5. MSP Virtual Radio ──
    if not args.no_transmitter:
        radio_index = os.path.join(MSP_RADIO_HOME, "index.js")
        if os.path.isfile(radio_index):
            log.info("Starting MSP Virtual Radio")
            pm.spawn(["node", radio_index])
        else:
            log.warning("MSP Virtual Radio not found at %s — skipping", radio_index)
    time.sleep(2)

    # ── 6. Discover camera topic ──
    log.info("Discovering camera image topic …")
    topic = discover_camera_topic(timeout=30)
    if not topic:
        log.error(
            "Could not find a camera image topic. "
            "Verify the world SDF includes a model with a camera sensor."
        )
        pm.shutdown()
        sys.exit(1)
    log.info("Found camera topic: %s", topic)

    # ── 7. Start image bridge ──
    if not os.path.isfile(IMAGE_BRIDGE):
        log.error(
            "gz_image_bridge not found at %s — run build_plugin.sh to build it",
            IMAGE_BRIDGE,
        )
        pm.shutdown()
        sys.exit(1)

    bridge_proc = pm.spawn(
        [IMAGE_BRIDGE, topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Make bridge stderr non-blocking for metadata reading
    import fcntl
    flags = fcntl.fcntl(bridge_proc.stderr, fcntl.F_GETFL)
    fcntl.fcntl(bridge_proc.stderr, fcntl.F_SETFL, flags | os.O_NONBLOCK)

    log.info("Waiting for first camera frame …")
    width, height, pix_fmt = read_image_meta(bridge_proc, timeout=30)
    if width is None:
        log.error("No image metadata received from bridge — camera may not be rendering")
        remaining_stderr = bridge_proc.stderr.read(2048)
        if remaining_stderr:
            log.error("Bridge stderr: %s", remaining_stderr.decode("utf-8", errors="replace"))
        pm.shutdown()
        sys.exit(1)
    log.info("Camera: %dx%d %s", width, height, pix_fmt)

    # ── 8. Start ffmpeg ──
    ffmpeg_input = [
        "ffmpeg",
        "-y",
        "-f", "rawvideo",
        "-pix_fmt", pix_fmt,
        "-s", f"{width}x{height}",
        "-r", str(args.fps),
        "-i", "pipe:0",
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-g", str(args.fps),  # keyframe interval = 1 second
    ]

    if output_mode == "tcp":
        ffmpeg_output = ["-f", "mpegts", f"tcp://0.0.0.0:{args.port}?listen=1"]
        viewer_url = f"tcp://<host>:{args.port}"
    elif output_mode == "rtsp":
        ffmpeg_output = ["-f", "rtsp", f"rtsp://127.0.0.1:{args.port}/fpv"]
        viewer_url = f"rtsp://<host>:{args.port}/fpv"
    elif output_mode.startswith("file:"):
        filepath = output_mode[5:]
        ffmpeg_output = ["-f", "mp4", filepath]
        viewer_url = filepath
    else:
        log.error("Unknown output mode: %s", output_mode)
        pm.shutdown()
        sys.exit(1)

    ffmpeg_cmd = ffmpeg_input + ffmpeg_output
    log.info("Starting ffmpeg → %s", output_mode)

    ffmpeg_proc = pm.spawn(
        ffmpeg_cmd,
        stdin=bridge_proc.stdout,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    # ── 9. Print connection info ──
    print()
    print("=" * 60)
    print("  FPV Simulation Running")
    print("=" * 60)
    print()
    print(f"  Video stream : {viewer_url}")
    print(f"  RC input     : UDP 127.0.0.1:9004  (flight_test.py)")
    print(f"  BF CLI       : TCP 127.0.0.1:5761")
    print(f"  BF Configurator: TCP 127.0.0.1:5760")
    print(f"  Camera topic : {topic}")
    print(f"  Resolution   : {width}x{height} @ {args.fps} fps")
    print()
    if output_mode == "tcp":
        print(f"  View with:  ffplay tcp://<host>:{args.port}")
    elif output_mode == "rtsp":
        print(f"  View with:  ffplay rtsp://<host>:{args.port}/fpv")
        print(f"       or:    vlc rtsp://<host>:{args.port}/fpv")
    print()
    print("  Press Ctrl-C to stop")
    print("=" * 60)
    print()

    # ── 10. Keep alive (auto-restart ffmpeg on viewer disconnect) ──
    try:
        while True:
            # Bridge is the critical process — if it dies, we're done
            if bridge_proc.poll() is not None:
                log.warning("Image bridge exited (code %d)", bridge_proc.returncode)
                break

            # ffmpeg exits when the TCP viewer disconnects — restart it so
            # a new viewer can connect without killing the whole simulation.
            if ffmpeg_proc.poll() is not None:
                rc = ffmpeg_proc.returncode
                if rc != 0:
                    log.info("ffmpeg exited (code %d) — restarting for next viewer connection …", rc)
                else:
                    log.info("ffmpeg finished — restarting …")
                # Remove dead process from the manager
                if ffmpeg_proc in pm.procs:
                    pm.procs.remove(ffmpeg_proc)
                time.sleep(1)
                ffmpeg_proc = pm.spawn(
                    ffmpeg_cmd,
                    stdin=bridge_proc.stdout,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )

            time.sleep(2)
    except KeyboardInterrupt:
        pass

    pm.shutdown()
    log.info("FPV simulation stopped")


if __name__ == "__main__":
    main()
