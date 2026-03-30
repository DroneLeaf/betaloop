#!/usr/bin/env python3
"""Rocket drone FPV launcher — Betaflight SITL + Gazebo camera → RTSP/TCP stream.

Starts the full stack and streams the on-board camera feed so any RTSP/TCP
video client (VLC, ffplay, Unreal Engine, Unity) can display it.

Architecture:
    Gazebo (camera sensor) ──gz-transport──► gz_image_bridge ──pipe──► ffmpeg
                                                              ──► RTSP/TCP stream

Usage (inside the Docker container):
    python3 start_rocket_drone_fpv_park.py                      # TCP stream on :8554
    python3 start_rocket_drone_fpv_park.py --rtsp               # RTSP via mediamtx on :8554
    python3 start_rocket_drone_fpv_park.py --output file:out.mp4  # record to file

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
log = logging.getLogger("start_rocket_drone_fpv_park")

# ── Defaults ──────────────────────────────────────────────────────────────────
# Auto-detect paths: if running from the repo (betaloop/ dir), resolve relative
# to the repo root. Otherwise fall back to Docker paths (/opt/...).
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)

def _default_path(env_var, repo_relative, docker_absolute):
    """Return env override, or repo-relative path if it exists, else Docker path."""
    if os.environ.get(env_var):
        return os.environ[env_var]
    repo_path = os.path.join(_REPO_ROOT, repo_relative)
    if os.path.exists(repo_path):
        return repo_path
    return docker_absolute

AEROLOOP_HOME = _default_path("AEROLOOP_HOME", "aeroloop_gazebo", "/opt/aeroloop_gazebo")
BF_ELF = _default_path("BF_ELF", os.path.join("betaflight", "obj", "main", "betaflight_SITL.elf"),
                        "/opt/betaflight/obj/main/betaflight_SITL.elf")
MSP_RADIO_HOME = _default_path("MSP_RADIO_HOME", os.path.join("..", "msp_virtualradio"),
                               "/opt/msp_virtualradio")
FPV_WORLD = "rocket_drone_park.world"
TOPIC_MODEL_HINT_DEFAULT = "betaflight_vehicle"
IMAGE_BRIDGE = os.path.join(AEROLOOP_HOME, "plugins", "build", "gz_image_bridge")
STREAM_PORT = 8554


def _is_container():
    """Detect if running inside a Docker container."""
    return os.path.exists("/.dockerenv") or os.environ.get("container") == "docker"


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


def cleanup_before_start():
    """Clean up any existing processes from previous runs."""
    log.info("Cleaning up from previous runs...")

    cleanup_cmds = [
        ("existing Gazebo processes", "pkill -9 -f 'gz sim' 2>/dev/null || true"),
        ("existing ffmpeg processes", "pkill -9 ffmpeg 2>/dev/null || true"),
        ("existing image bridge processes", "pkill -9 -f 'gz_image_bridge' 2>/dev/null || true"),
        ("existing Betaflight SITL processes", "pkill -9 -f 'betaflight_SITL.elf' 2>/dev/null || true"),
        ("existing MSP Virtual Radio processes", "pkill -9 -f 'msp_virtualradio/index.js' 2>/dev/null || true"),
        ("existing mediamtx processes", "pkill -9 -f mediamtx 2>/dev/null || true"),
        ("existing Xvfb processes", "pkill -9 Xvfb 2>/dev/null || true"),
    ]

    for label, cmd in cleanup_cmds:
        try:
            subprocess.run(["bash", "-c", cmd], capture_output=True, timeout=5)
            log.info("Stopped %s", label)
        except Exception:
            pass
        time.sleep(0.2)

    log.info("Cleanup complete")


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


def list_camera_topics(name_hint=None):
    """Return sorted camera image topics, optionally filtered by substring."""
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True, text=True, timeout=10,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []

    topics = []
    for line in result.stdout.strip().splitlines():
        line = line.strip()
        if not line.endswith("/image"):
            continue
        if name_hint and name_hint not in line:
            continue
        topics.append(line)
    return sorted(set(topics))


def discover_camera_topic(name_hint="fpv_cam", timeout=30, model_hint=None):
    """Find a camera image topic matching *name_hint*.

    If model_hint is set, prefer topics containing that substring.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        topics = list_camera_topics(name_hint=name_hint)
        if topics:
            if model_hint:
                preferred = [topic for topic in topics if model_hint in topic]
                if preferred:
                    return preferred[0]
            return topics[0]
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
    parser = argparse.ArgumentParser(description="Rocket drone FPV simulation launcher")
    parser.add_argument("--world", default=FPV_WORLD,
                        help=f"World SDF file (default: {FPV_WORLD})")
    parser.add_argument("--elf", default=BF_ELF,
                        help="Path to betaflight_SITL.elf")
    parser.add_argument("--port", type=int, default=STREAM_PORT,
                        help="Stream output port (default: 8554)")
    parser.add_argument("--rtsp", action="store_true",
                        help="Use RTSP streaming via mediamtx")
    parser.add_argument("--output", default=None,
                        help="Override output: 'udp' (default), 'tcp', 'rtsp', or 'file:<path>'")
    parser.add_argument("--gazebo", action="store_true",
                        help="Show the Gazebo GUI (default: headless)")
    parser.add_argument("--no-transmitter", action="store_true",
                        help="Skip starting the MSP virtual radio")
    parser.add_argument("--fps", type=int, default=30,
                        help="Output stream FPS (default: 30)")
    parser.add_argument("--chase-cam", action="store_true",
                        help="Also stream the chase camera (3rd-person view)")
    parser.add_argument("--chase-port", type=int, default=8555,
                        help="Chase-cam stream port (default: 8555)")
    parser.add_argument("--fpv-topic", default=None,
                        help="Use this exact Gazebo image topic for FPV (skip auto-discovery)")
    parser.add_argument("--chase-topic", default=None,
                        help="Use this exact Gazebo image topic for chase-cam (skip auto-discovery)")
    parser.add_argument("--topic-model-hint", default=TOPIC_MODEL_HINT_DEFAULT,
                        help="Prefer topics that contain this model path segment (e.g. betaflight_vehicle)")
    parser.add_argument("--osd", action="store_true",
                        help="Enable Betaflight OSD overlay on the FPV stream")
    parser.add_argument("--msp-port", type=int, default=5762,
                        help="Betaflight MSP TCP port for OSD telemetry (default: 5762 = UART2)")
    parser.add_argument("--osd-server-port", type=int, default=0,
                        help="TCP port for companion OSD server (LeafFC/HEAR), 0=disabled")
    parser.add_argument("--osd-grid", default="53x20",
                        help="OSD grid size for companion OSD (default: 53x20)")
    parser.add_argument("--raw", action="store_true",
                        help="Bypass ffmpeg: pipe raw frames directly to ffplay (latency test)")
    args = parser.parse_args()

    # Clean up any existing processes before starting new ones.
    cleanup_before_start()

    # Determine output mode
    if args.output:
        output_mode = args.output
    elif args.rtsp:
        output_mode = "rtsp"
    else:
        output_mode = "udp"

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
    in_container = _is_container()

    if gpu_available:
        log.info("NVIDIA GPU detected — using GPU-accelerated rendering")
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)

        # Force NVIDIA EGL vendor ICD everywhere — Ogre2's camera sensor uses
        # EGL for off-screen rendering.  Without this, Mesa's DRI2 path is
        # tried and falls back to software ("failed to create dri2 screen").
        # This is safe on the host: it only affects EGL, not GLX.
        nvidia_icd = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.isfile(nvidia_icd):
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_icd
            log.info("Forcing NVIDIA EGL vendor ICD")

        if in_container:
            # Inside Docker we must also force GLX to use NVIDIA — the
            # container toolkit injects replacement libs that Mesa can't use.
            os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
            log.info("Container detected — also forcing NVIDIA GLX vendor")
            _fix_dri_permissions()
        else:
            log.info("Host detected — native GLX, forced EGL")
    else:
        log.info("No GPU detected — using software rendering (llvmpipe)")
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    # ── 1c. Display setup ──
    # Ogre2's camera sensor needs a display context to render frames.
    if args.gazebo:
        # GUI mode — need the host's real X display
        if not os.environ.get("DISPLAY"):
            log.error(
                "No DISPLAY set — the Gazebo GUI needs a display.\n"
                "  In Docker, run with: -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"
            )
            sys.exit(1)
        # Don't force __GLX_VENDOR_LIBRARY_NAME when talking to the host X server
        os.environ.pop("__GLX_VENDOR_LIBRARY_NAME", None)
        log.info("Using display %s for Gazebo GUI", os.environ["DISPLAY"])
    elif not in_container and os.environ.get("DISPLAY"):
        # Host headless mode — use the native display (low latency, no Xvfb).
        # Native NVIDIA GLX works fine here since we didn't set __GLX_VENDOR_LIBRARY_NAME.
        log.info("Using native display %s (low-latency host mode)", os.environ["DISPLAY"])
    else:
        # Container headless or no display — start Xvfb
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

    # ── 6. Discover camera topics ──
    if args.fpv_topic:
        topic = args.fpv_topic
        log.info("Using explicit FPV camera topic: %s", topic)
    else:
        log.info("Discovering FPV camera image topic …")
        fpv_candidates = list_camera_topics(name_hint="fpv_cam")
        if fpv_candidates:
            log.info("FPV candidates: %s", ", ".join(fpv_candidates))
        topic = discover_camera_topic(
            name_hint="fpv_cam",
            timeout=30,
            model_hint=args.topic_model_hint,
        )
        if not topic:
            log.error(
                "Could not find an FPV camera image topic. "
                "Verify the world SDF includes a camera sensor named fpv_cam."
            )
            pm.shutdown()
            sys.exit(1)
        log.info("Found FPV camera topic: %s", topic)

    chase_topic = None
    if args.chase_cam:
        if args.chase_topic:
            chase_topic = args.chase_topic
            log.info("Using explicit chase camera topic: %s", chase_topic)
        else:
            log.info("Discovering chase camera image topic …")
            chase_candidates = list_camera_topics(name_hint="chase_cam")
            if chase_candidates:
                log.info("Chase candidates: %s", ", ".join(chase_candidates))
            chase_topic = discover_camera_topic(
                name_hint="chase_cam",
                timeout=30,
                model_hint=args.topic_model_hint,
            )
            if not chase_topic:
                log.warning("Chase camera topic not found — only FPV stream will run")
            else:
                log.info("Found chase camera topic: %s", chase_topic)

    # ── 7. Start image bridge ──
    if not os.path.isfile(IMAGE_BRIDGE):
        log.error(
            "gz_image_bridge not found at %s — run build_plugin.sh to build it",
            IMAGE_BRIDGE,
        )
        pm.shutdown()
        sys.exit(1)

    bridge_cmd = [IMAGE_BRIDGE, topic]
    if args.osd:
        bridge_cmd += ["--osd", "--msp-port", str(args.msp_port)]
        log.info("OSD overlay enabled (MSP port %d)", args.msp_port)
    if args.osd_server_port:
        bridge_cmd += ["--osd-server-port", str(args.osd_server_port),
                       "--osd-grid", args.osd_grid]
        log.info("Companion OSD server on port %d (grid %s)",
                 args.osd_server_port, args.osd_grid)

    bridge_proc = pm.spawn(
        bridge_cmd,
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

    # ── 8. Start ffmpeg (or raw ffplay in --raw mode) ──
    if args.raw:
        # Bypass ffmpeg entirely — pipe raw frames straight to ffplay.
        # This isolates whether latency is in ffmpeg/muxer or elsewhere.
        log.info("RAW MODE: bypassing ffmpeg, piping directly to ffplay")
        ffplay_cmd = [
            "ffplay",
            "-f", "rawvideo",
            "-pixel_format", pix_fmt,
            "-video_size", f"{width}x{height}",
            "-framerate", str(args.fps),
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-framedrop",
            "-sync", "video",
            "-window_title", f"FPV RAW {width}x{height}",
            "-",
        ]
        log.info("Starting: %s", " ".join(ffplay_cmd[:6]) + " ...")
        ffmpeg_proc = pm.spawn(
            ffplay_cmd,
            stdin=bridge_proc.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        ffmpeg_cmd = ffplay_cmd  # for auto-restart
        viewer_url = "(raw ffplay window)"
        output_mode = "raw"
    else:
        # Try NVENC hardware encoder first (RTX/GTX GPUs), fall back to libx264.
        use_nvenc = False
        if gpu_available:
            try:
                probe = subprocess.run(
                    ["ffmpeg", "-hide_banner", "-encoders"],
                    capture_output=True, text=True, timeout=5,
                )
                if "h264_nvenc" in probe.stdout:
                    use_nvenc = True
                    log.info("Using NVENC hardware encoder (h264_nvenc)")
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass
        if not use_nvenc:
            log.info("Using software encoder (libx264 ultrafast)")

        ffmpeg_input = [
            "ffmpeg",
            "-y",
            "-f", "rawvideo",
            "-pix_fmt", pix_fmt,
            "-s", f"{width}x{height}",
            "-r", str(args.fps),
            "-i", "pipe:0",
            "-pix_fmt", "yuv420p",
        ]
        if use_nvenc:
            ffmpeg_input += [
                "-c:v", "h264_nvenc",
                "-preset", "p1",
                "-tune", "ull",
                "-rc", "cbr",
                "-b:v", "4M",
                "-g", "1",
                "-bf", "0",
                "-delay", "0",
                "-zerolatency", "1",
            ]
        else:
            ffmpeg_input += [
                "-c:v", "libx264",
                "-preset", "ultrafast",
                "-tune", "zerolatency",
                "-g", "1",
            ]
        ffmpeg_input += ["-flush_packets", "1"]

        if output_mode == "udp":
            ffmpeg_output = [
                "-muxdelay", "0", "-muxpreload", "0",
                "-f", "mpegts", f"udp://127.0.0.1:{args.port}?pkt_size=1316",
            ]
            viewer_url = f"udp://@:{args.port}"
        elif output_mode == "tcp":
            ffmpeg_output = [
                "-muxdelay", "0", "-muxpreload", "0",
                "-f", "mpegts", f"tcp://0.0.0.0:{args.port}?listen=1&tcp_nodelay=1",
            ]
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

    # ── 9a. Chase camera pipeline (optional) ──
    chase_bridge_proc = None
    chase_ffmpeg_proc = None
    chase_viewer_url = None
    chase_ffmpeg_cmd = None
    if chase_topic:
        log.info("Starting chase camera bridge")
        chase_bridge_proc = pm.spawn(
            [IMAGE_BRIDGE, chase_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        chase_flags = fcntl.fcntl(chase_bridge_proc.stderr, fcntl.F_GETFL)
        fcntl.fcntl(chase_bridge_proc.stderr, fcntl.F_SETFL, chase_flags | os.O_NONBLOCK)

        log.info("Waiting for first chase camera frame …")
        cw, ch, cpf = read_image_meta(chase_bridge_proc, timeout=30)
        if cw is None:
            log.warning("No chase camera metadata — chase stream disabled")
            chase_bridge_proc = None
        else:
            log.info("Chase camera: %dx%d %s", cw, ch, cpf)

            if args.raw:
                chase_ffplay_cmd = [
                    "ffplay",
                    "-f", "rawvideo",
                    "-pixel_format", cpf,
                    "-video_size", f"{cw}x{ch}",
                    "-framerate", str(args.fps),
                    "-fflags", "nobuffer",
                    "-flags", "low_delay",
                    "-framedrop",
                    "-sync", "video",
                    "-window_title", f"Chase RAW {cw}x{ch}",
                    "-",
                ]
                log.info("Starting chase camera ffplay (raw)")
                chase_ffmpeg_proc = pm.spawn(
                    chase_ffplay_cmd,
                    stdin=chase_bridge_proc.stdout,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                chase_ffmpeg_cmd = chase_ffplay_cmd
                chase_viewer_url = "(raw ffplay window)"
            else:
                chase_ffmpeg_input = [
                    "ffmpeg", "-y",
                    "-f", "rawvideo", "-pix_fmt", cpf,
                    "-s", f"{cw}x{ch}", "-r", str(args.fps),
                    "-i", "pipe:0", "-pix_fmt", "yuv420p",
                ]
                if use_nvenc:
                    chase_ffmpeg_input += [
                        "-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ull",
                        "-rc", "cbr", "-b:v", "4M", "-g", "1", "-bf", "0",
                        "-delay", "0", "-zerolatency", "1",
                    ]
                else:
                    chase_ffmpeg_input += [
                        "-c:v", "libx264", "-preset", "ultrafast",
                        "-tune", "zerolatency", "-g", "1",
                    ]
                chase_ffmpeg_input += ["-flush_packets", "1"]

                if output_mode == "udp":
                    chase_ffmpeg_output = [
                        "-muxdelay", "0", "-muxpreload", "0",
                        "-f", "mpegts", f"udp://127.0.0.1:{args.chase_port}?pkt_size=1316",
                    ]
                    chase_viewer_url = f"udp://@:{args.chase_port}"
                elif output_mode == "tcp":
                    chase_ffmpeg_output = [
                        "-muxdelay", "0", "-muxpreload", "0",
                        "-f", "mpegts", f"tcp://0.0.0.0:{args.chase_port}?listen=1&tcp_nodelay=1",
                    ]
                    chase_viewer_url = f"tcp://<host>:{args.chase_port}"
                elif output_mode == "rtsp":
                    chase_ffmpeg_output = ["-f", "rtsp", f"rtsp://127.0.0.1:{args.port}/chase"]
                    chase_viewer_url = f"rtsp://<host>:{args.port}/chase"
                else:
                    chase_ffmpeg_output = ffmpeg_output
                    chase_viewer_url = "(same as FPV)"

                chase_ffmpeg_cmd = chase_ffmpeg_input + chase_ffmpeg_output
                log.info("Starting chase camera ffmpeg → %s", output_mode)
                chase_ffmpeg_proc = pm.spawn(
                    chase_ffmpeg_cmd,
                    stdin=chase_bridge_proc.stdout,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )

    # ── 10. Print connection info ──
    print()
    print("=" * 60)
    print("  FPV Simulation Running")
    print("=" * 60)
    print()
    print(f"  FPV stream   : {viewer_url}")
    if chase_viewer_url:
        print(f"  Chase stream : {chase_viewer_url}")
    print(f"  RC input     : UDP 127.0.0.1:9004  (flight_test.py)")
    print(f"  BF CLI       : TCP 127.0.0.1:5761")
    print(f"  BF Configurator: TCP 127.0.0.1:5760")
    print(f"  FPV topic    : {topic}")
    if chase_topic:
        print(f"  Chase topic  : {chase_topic}")
    print(f"  Resolution   : {width}x{height} @ {args.fps} fps")
    if args.raw:
        print(f"  Mode         : RAW (no ffmpeg, direct ffplay)")
    print()
    low_lat = "-probesize 32 -analyzeduration 0 -fflags nobuffer -flags low_delay -framedrop -sync ext"
    if output_mode == "raw":
        print("  FPV + Chase: displayed in ffplay windows directly (no encoding)")
    elif output_mode == "udp":
        print(f"  FPV view:   ffplay {low_lat} udp://@:{args.port}")
        if chase_viewer_url:
            print(f"  Chase view: ffplay {low_lat} udp://@:{args.chase_port}")
    elif output_mode == "tcp":
        print(f"  FPV view:   ffplay {low_lat} tcp://<host>:{args.port}")
        if chase_viewer_url:
            print(f"  Chase view: ffplay {low_lat} tcp://<host>:{args.chase_port}")
    elif output_mode == "rtsp":
        print(f"  FPV view:   ffplay {low_lat} rtsp://<host>:{args.port}/fpv")
        if chase_viewer_url:
            print(f"  Chase view: ffplay {low_lat} rtsp://<host>:{args.port}/chase")
        print(f"       or:    vlc rtsp://<host>:{args.port}/fpv")
    print()
    print("  Press Ctrl-C to stop")
    print("=" * 60)
    print()

    # ── 11. Keep alive (auto-restart ffmpeg on viewer disconnect) ──
    try:
        while True:
            # FPV bridge is critical — if it dies, we're done
            if bridge_proc.poll() is not None:
                log.warning("FPV image bridge exited (code %d)", bridge_proc.returncode)
                break

            # Auto-restart FPV ffmpeg (e.g. TCP viewer disconnect)
            if ffmpeg_proc.poll() is not None:
                rc = ffmpeg_proc.returncode
                log.info("FPV ffmpeg exited (code %d) — restarting …", rc)
                if ffmpeg_proc in pm.procs:
                    pm.procs.remove(ffmpeg_proc)
                time.sleep(1)
                ffmpeg_proc = pm.spawn(
                    ffmpeg_cmd,
                    stdin=bridge_proc.stdout,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )

            # Auto-restart chase camera ffmpeg
            if chase_ffmpeg_proc and chase_ffmpeg_cmd:
                if chase_bridge_proc and chase_bridge_proc.poll() is not None:
                    log.warning("Chase camera bridge exited (code %d)", chase_bridge_proc.returncode)
                    chase_ffmpeg_proc = None
                    chase_bridge_proc = None
                elif chase_ffmpeg_proc.poll() is not None:
                    rc = chase_ffmpeg_proc.returncode
                    log.info("Chase ffmpeg exited (code %d) — restarting …", rc)
                    if chase_ffmpeg_proc in pm.procs:
                        pm.procs.remove(chase_ffmpeg_proc)
                    time.sleep(1)
                    chase_ffmpeg_proc = pm.spawn(
                        chase_ffmpeg_cmd,
                        stdin=chase_bridge_proc.stdout,
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
