#!/usr/bin/env python3
"""Simulink dynamics backend launcher — bf_sim_bridge + Betaflight SITL + Gazebo vis-only.

Replaces Gazebo physics with the HEAR Simulink rigid-body dynamics solver.
Gazebo becomes a pose-driven visualizer and camera renderer only.

Process launch order:
    1. [Xvfb]       — virtual display (headless only)
    2. [mediamtx]   — RTSP server (if --rtsp)
    3. Gazebo        — vis-only world (rocket_drone_vis.sdf), no physics
    4. bf_sim_bridge — Simulink dynamics, UDP ↔ BF SITL + Gazebo
    5. Betaflight SITL
    6. [MSP Virtual Radio]
    7. gz_image_bridge + ffmpeg  — FPV video pipeline

Usage:
    python3 start_simulink.py                          # UDP stream on :8554
    python3 start_simulink.py --rtsp                   # RTSP via mediamtx
    python3 start_simulink.py --gazebo                 # Show Gazebo GUI
    python3 start_simulink.py --raw                    # Raw ffplay (latency test)
    python3 start_simulink.py --output file:out.mp4    # Record to file

Viewer examples:
    ffplay udp://@:8554
    ffplay rtsp://<host>:8554/fpv
"""

import argparse
import fcntl
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
log = logging.getLogger("start_simulink")

# ── Defaults ──────────────────────────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)


def _default_path(env_var, repo_relative):
    """Return env override, or repo-relative path."""
    if os.environ.get(env_var):
        return os.environ[env_var]
    return os.path.join(_REPO_ROOT, repo_relative)


AEROLOOP_HOME = _default_path("AEROLOOP_HOME", "aeroloop_gazebo")
BF_ELF = _default_path(
    "BF_ELF",
    os.path.join("betaflight", "obj", "main", "betaflight_SITL.elf"),
)
MSP_RADIO_HOME = _default_path(
    "MSP_RADIO_HOME",
    os.path.join("..", "msp_virtualradio"),
)
BF_SIM_BRIDGE = _default_path(
    "BF_SIM_BRIDGE",
    os.path.join("bf_sim_bridge", "build", "bf_sim_bridge"),
)
SIMULINK_LIB = _default_path(
    "SIMULINK_LIB",
    os.path.join("HEAR_Simulations", "Compiled_models", "rocket_drone_quad_SITL", "libinterface_simulink.so"),
)

VIS_WORLD = "rocket_drone_vis_park.world"
TOPIC_MODEL_HINT = "betaflight_vehicle"
IMAGE_BRIDGE = os.path.join(AEROLOOP_HOME, "plugins", "build", "gz_image_bridge")
STREAM_PORT = 8554


# ── Helpers ───────────────────────────────────────────────────────────────────

class ProcessManager:
    """Track child processes for clean shutdown."""

    def __init__(self):
        self.procs: list[subprocess.Popen] = []

    def spawn(self, args, **kwargs):
        log.info("Starting: %s", " ".join(str(a) for a in args[:5]))
        p = subprocess.Popen(args, **kwargs)
        self.procs.append(p)
        return p

    def shutdown(self):
        log.info("Shutting down %d processes …", len(self.procs))
        # First pass: SIGTERM (graceful)
        for p in reversed(self.procs):
            if p.poll() is None:
                p.terminate()
        # Second pass: wait for exit, then SIGKILL if stuck
        for p in reversed(self.procs):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=2)
        # Safety net: kill any orphaned simulation processes that escaped
        # the process tree (e.g. Gazebo server threads, gz-transport nodes)
        for pattern in ["gz sim", "ruby.*gz", "gz_image_bridge"]:
            subprocess.run(
                ["pkill", "-9", "-f", pattern],
                capture_output=True, timeout=3,
            )


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
    """Set Gazebo Harmonic environment variables."""

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
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=5,
        )
        return result.returncode == 0 and result.stdout.strip() != ""
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def list_camera_topics(name_hint=None):
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
    deadline = time.time() + timeout
    while time.time() < deadline:
        topics = list_camera_topics(name_hint=name_hint)
        if topics:
            if model_hint:
                preferred = [t for t in topics if model_hint in t]
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
                if proc.poll() is not None:
                    break
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
    parser = argparse.ArgumentParser(
        description="Simulink dynamics backend launcher (bf_sim_bridge + BF SITL + Gazebo vis-only)",
    )
    parser.add_argument("--world", default=VIS_WORLD,
                        help=f"World SDF file (default: {VIS_WORLD})")
    parser.add_argument("--elf", default=BF_ELF,
                        help="Path to betaflight_SITL.elf")
    parser.add_argument("--sim-lib", default=SIMULINK_LIB,
                        help="Path to libinterface_simulink.so")
    parser.add_argument("--bridge", default=BF_SIM_BRIDGE,
                        help="Path to bf_sim_bridge executable")
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
    parser.add_argument("--no-video", action="store_true",
                        help="Skip the video pipeline (Gazebo + bridge only)")
    parser.add_argument("--fps", type=int, default=30,
                        help="Output stream FPS (default: 30)")
    parser.add_argument("--chase-cam", action="store_true",
                        help="Also stream the chase camera")
    parser.add_argument("--chase-port", type=int, default=8555,
                        help="Chase-cam stream port (default: 8555)")
    parser.add_argument("--fpv-topic", default=None,
                        help="Explicit Gazebo FPV image topic (skip auto-discovery)")
    parser.add_argument("--chase-topic", default=None,
                        help="Explicit Gazebo chase image topic (skip auto-discovery)")
    parser.add_argument("--osd", action="store_true",
                        help="Enable Betaflight OSD overlay on the FPV stream")
    parser.add_argument("--msp-port", type=int, default=5762,
                        help="BF MSP TCP port for OSD telemetry (default: 5762)")
    parser.add_argument("--osd-server-port", type=int, default=0,
                        help="TCP port for companion OSD server (LeafFC/HEAR), 0=disabled")
    parser.add_argument("--osd-grid", default="53x20",
                        help="OSD grid size for companion OSD (default: 53x20)")
    parser.add_argument("--raw", action="store_true",
                        help="Bypass ffmpeg: pipe raw frames to ffplay (latency test)")
    parser.add_argument("--display", action="store_true",
                        help="Render in SDL2 window inside gz_image_bridge (lowest latency)")
    parser.add_argument("--shm", action="store_true",
                        help="Expose frames via POSIX shared memory for local tracker (zero-latency)")
    parser.add_argument("--stream", default="",
                        help="Stream raw (no OSD) H.264 over UDP to host:port (e.g. 10.0.0.87:5000)")
    args = parser.parse_args()

    # Output mode
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

    # ── 1b. GPU detection ──
    gpu_available = has_nvidia_gpu()

    if gpu_available:
        log.info("NVIDIA GPU detected — using GPU-accelerated rendering")
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
        nvidia_icd = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.isfile(nvidia_icd):
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_icd
            log.info("Forcing NVIDIA EGL vendor ICD")
    else:
        log.info("No GPU detected — using software rendering (llvmpipe)")
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    # ── 1c. Display setup ──
    if args.gazebo:
        if not os.environ.get("DISPLAY"):
            log.error("No DISPLAY set — the Gazebo GUI needs a display.")
            sys.exit(1)
        os.environ.pop("__GLX_VENDOR_LIBRARY_NAME", None)
        log.info("Using display %s for Gazebo GUI", os.environ["DISPLAY"])
    elif os.environ.get("DISPLAY"):
        log.info("Using native display %s (host headless mode)", os.environ["DISPLAY"])
    else:
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
            log.error("mediamtx not found at %s", mediamtx)
            sys.exit(1)
        log.info("Starting mediamtx RTSP server")
        pm.spawn([mediamtx], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)

    # ── 3. Gazebo (vis-only world) ──
    world_path = args.world
    if not os.path.isabs(world_path):
        world_path = os.path.join(AEROLOOP_HOME, "worlds", world_path)
    if not os.path.isfile(world_path):
        log.error("World file not found: %s", world_path)
        sys.exit(1)

    gz_args = ["gz", "sim"]
    if not args.gazebo:
        gz_args.append("-s")
    gz_args.extend(["-r", "-v", "3", world_path])

    log.info("Starting Gazebo (vis-only): %s", os.path.basename(world_path))
    pm.spawn(gz_args)
    time.sleep(8)

    # ── 4. bf_sim_bridge (Simulink dynamics) ──
    if not os.path.isfile(args.bridge):
        log.error("bf_sim_bridge not found: %s", args.bridge)
        pm.shutdown()
        sys.exit(1)
    if not os.path.isfile(args.sim_lib):
        log.error("libinterface_simulink.so not found: %s", args.sim_lib)
        pm.shutdown()
        sys.exit(1)

    bridge_args = [
        args.bridge,
        "--sim-lib", os.path.abspath(args.sim_lib),
    ]
    log.info("Starting bf_sim_bridge (Simulink dynamics)")
    bf_bridge_proc = pm.spawn(bridge_args)
    time.sleep(2)
    if bf_bridge_proc.poll() is not None:
        log.error("bf_sim_bridge exited immediately (code %d)", bf_bridge_proc.returncode)
        pm.shutdown()
        sys.exit(1)

    # ── 5. Betaflight SITL ──
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

    # ── 6. MSP Virtual Radio ──
    if not args.no_transmitter:
        radio_index = os.path.join(MSP_RADIO_HOME, "index.js")
        if os.path.isfile(radio_index):
            log.info("Starting MSP Virtual Radio")
            pm.spawn(["node", radio_index])
        else:
            log.warning("MSP Virtual Radio not found at %s — skipping", radio_index)
    time.sleep(2)

    # ── 7. Video pipeline (skip if --no-video) ──
    topic = None
    bridge_proc = None
    ffmpeg_proc = None
    ffmpeg_cmd = None
    viewer_url = "(video disabled)"
    width = height = 0
    chase_topic = None
    chase_bridge_proc = None
    chase_ffmpeg_proc = None
    chase_ffmpeg_cmd = None
    chase_viewer_url = None
    use_nvenc = False

    if not args.no_video:
        if not os.path.isfile(IMAGE_BRIDGE):
            log.error("gz_image_bridge not found at %s — run build_plugin.sh", IMAGE_BRIDGE)
            pm.shutdown()
            sys.exit(1)

        # Discover FPV camera topic
        if args.fpv_topic:
            topic = args.fpv_topic
        else:
            log.info("Discovering FPV camera image topic …")
            topic = discover_camera_topic(
                name_hint="fpv_cam", timeout=30, model_hint=TOPIC_MODEL_HINT,
            )
            if not topic:
                log.error("Could not find FPV camera image topic")
                pm.shutdown()
                sys.exit(1)
        log.info("Found FPV camera topic: %s", topic)

        # Chase camera (optional)
        if args.chase_cam:
            if args.chase_topic:
                chase_topic = args.chase_topic
            else:
                chase_topic = discover_camera_topic(
                    name_hint="chase_cam", timeout=30, model_hint=TOPIC_MODEL_HINT,
                )
                if not chase_topic:
                    log.warning("Chase camera topic not found — FPV only")
            if chase_topic:
                log.info("Found chase camera topic: %s", chase_topic)

        # Start image bridge
        img_bridge_cmd = [IMAGE_BRIDGE, topic]
        if args.osd:
            img_bridge_cmd += ["--osd", "--msp-port", str(args.msp_port)]
            log.info("OSD overlay enabled (MSP port %d)", args.msp_port)
        if args.osd_server_port:
            img_bridge_cmd += ["--osd-server-port", str(args.osd_server_port),
                               "--osd-grid", args.osd_grid]
            log.info("Companion OSD server on port %d (grid %s)",
                     args.osd_server_port, args.osd_grid)
        if args.stream:
            img_bridge_cmd += ["--stream", args.stream]
            log.info("Streaming raw frames to udp://%s", args.stream)
        if args.display:
            img_bridge_cmd += ["--display"]
            log.info("SDL2 direct display mode (zero-latency)")
        if args.shm:
            img_bridge_cmd += ["--shm"]
            log.info("Shared memory frame server enabled")

        bridge_proc = pm.spawn(
            img_bridge_cmd,
            stdout=subprocess.DEVNULL if args.display else subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        flags = fcntl.fcntl(bridge_proc.stderr, fcntl.F_GETFL)
        fcntl.fcntl(bridge_proc.stderr, fcntl.F_SETFL, flags | os.O_NONBLOCK)

        log.info("Waiting for first camera frame …")
        width, height, pix_fmt = read_image_meta(bridge_proc, timeout=30)
        if width is None:
            log.error("No image metadata from bridge — camera may not be rendering")
            pm.shutdown()
            sys.exit(1)
        log.info("Camera: %dx%d %s", width, height, pix_fmt)

        # Encoder selection
        if gpu_available:
            try:
                probe = subprocess.run(
                    ["ffmpeg", "-hide_banner", "-encoders"],
                    capture_output=True, text=True, timeout=5,
                )
                if "h264_nvenc" in probe.stdout:
                    use_nvenc = True
                    log.info("Using NVENC hardware encoder")
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass
        if not use_nvenc:
            log.info("Using software encoder (libx264 ultrafast)")

        if args.display:
            log.info("DISPLAY MODE: bridge renders directly via SDL2 (no ffmpeg)")
            ffmpeg_proc = None
            ffmpeg_cmd = None
            viewer_url = "(SDL2 window \u2014 zero-latency)"
            output_mode = "display"
        elif args.raw:
            ffplay_cmd = [
                "ffplay",
                "-f", "rawvideo",
                "-pixel_format", pix_fmt,
                "-video_size", f"{width}x{height}",
                "-framerate", str(args.fps),
                "-fflags", "nobuffer",
                "-flags", "low_delay",
                "-framedrop", "-sync", "video",
                "-window_title", f"FPV RAW {width}x{height}",
                "-",
            ]
            ffmpeg_proc = pm.spawn(
                ffplay_cmd,
                stdin=bridge_proc.stdout,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            ffmpeg_cmd = ffplay_cmd
            viewer_url = "(raw ffplay window)"
            output_mode = "raw"
        else:
            ffmpeg_input = [
                "ffmpeg", "-y",
                "-f", "rawvideo", "-pix_fmt", pix_fmt,
                "-s", f"{width}x{height}", "-r", str(args.fps),
                "-i", "pipe:0", "-pix_fmt", "yuv420p",
            ]
            if use_nvenc:
                ffmpeg_input += [
                    "-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ull",
                    "-rc", "cbr", "-b:v", "4M", "-g", "1", "-bf", "0",
                    "-delay", "0", "-zerolatency", "1",
                ]
            else:
                ffmpeg_input += [
                    "-c:v", "libx264", "-preset", "ultrafast",
                    "-tune", "zerolatency", "-g", "1",
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
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )

        # Chase camera pipeline
        if chase_topic:
            chase_bridge_proc = pm.spawn(
                [IMAGE_BRIDGE, chase_topic],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            )
            cf = fcntl.fcntl(chase_bridge_proc.stderr, fcntl.F_GETFL)
            fcntl.fcntl(chase_bridge_proc.stderr, fcntl.F_SETFL, cf | os.O_NONBLOCK)

            cw, ch, cpf = read_image_meta(chase_bridge_proc, timeout=30)
            if cw is None:
                log.warning("No chase camera metadata — chase stream disabled")
                chase_bridge_proc = None
            else:
                log.info("Chase camera: %dx%d %s", cw, ch, cpf)
                if args.raw:
                    chase_ffmpeg_cmd = [
                        "ffplay",
                        "-f", "rawvideo", "-pixel_format", cpf,
                        "-video_size", f"{cw}x{ch}", "-framerate", str(args.fps),
                        "-fflags", "nobuffer", "-flags", "low_delay",
                        "-framedrop", "-sync", "video",
                        "-window_title", f"Chase RAW {cw}x{ch}", "-",
                    ]
                    chase_viewer_url = "(raw ffplay window)"
                else:
                    chase_input = [
                        "ffmpeg", "-y",
                        "-f", "rawvideo", "-pix_fmt", cpf,
                        "-s", f"{cw}x{ch}", "-r", str(args.fps),
                        "-i", "pipe:0", "-pix_fmt", "yuv420p",
                    ]
                    if use_nvenc:
                        chase_input += [
                            "-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ull",
                            "-rc", "cbr", "-b:v", "4M", "-g", "1", "-bf", "0",
                            "-delay", "0", "-zerolatency", "1",
                        ]
                    else:
                        chase_input += [
                            "-c:v", "libx264", "-preset", "ultrafast",
                            "-tune", "zerolatency", "-g", "1",
                        ]
                    chase_input += ["-flush_packets", "1"]

                    if output_mode == "udp":
                        chase_out = [
                            "-muxdelay", "0", "-muxpreload", "0",
                            "-f", "mpegts", f"udp://127.0.0.1:{args.chase_port}?pkt_size=1316",
                        ]
                        chase_viewer_url = f"udp://@:{args.chase_port}"
                    elif output_mode == "tcp":
                        chase_out = [
                            "-muxdelay", "0", "-muxpreload", "0",
                            "-f", "mpegts", f"tcp://0.0.0.0:{args.chase_port}?listen=1&tcp_nodelay=1",
                        ]
                        chase_viewer_url = f"tcp://<host>:{args.chase_port}"
                    elif output_mode == "rtsp":
                        chase_out = ["-f", "rtsp", f"rtsp://127.0.0.1:{args.port}/chase"]
                        chase_viewer_url = f"rtsp://<host>:{args.port}/chase"
                    else:
                        chase_out = ffmpeg_output if ffmpeg_cmd else []
                        chase_viewer_url = "(same as FPV)"

                    chase_ffmpeg_cmd = chase_input + chase_out

                if chase_ffmpeg_cmd:
                    chase_ffmpeg_proc = pm.spawn(
                        chase_ffmpeg_cmd,
                        stdin=chase_bridge_proc.stdout,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    )

    # ── 8. Print connection info ──
    print()
    print("=" * 64)
    print("  Simulink Dynamics Backend — Running")
    print("=" * 64)
    print()
    print(f"  Backend      : bf_sim_bridge → Simulink → BF SITL (250 Hz)")
    print(f"  World        : {os.path.basename(args.world)} (vis-only, no GZ physics)")
    print(f"  Simulink .so : {os.path.basename(args.sim_lib)}")
    if not args.no_video:
        print(f"  FPV stream   : {viewer_url}")
        if chase_viewer_url:
            print(f"  Chase stream : {chase_viewer_url}")
        print(f"  FPV topic    : {topic}")
        if chase_topic:
            print(f"  Chase topic  : {chase_topic}")
        print(f"  Resolution   : {width}x{height} @ {args.fps} fps")
    else:
        print(f"  Video        : disabled (--no-video)")
    print(f"  RC input     : UDP 127.0.0.1:9004  (flight_test.py / virtual_rc.py)")
    print(f"  BF CLI       : TCP 127.0.0.1:5761")
    print(f"  BF Config    : TCP 127.0.0.1:5760")
    print()
    if not args.no_video:
        low_lat = "-probesize 32 -analyzeduration 0 -fflags nobuffer -flags low_delay -framedrop -sync ext"
        if output_mode == "display":
            print("  FPV: rendered in SDL2 window by gz_image_bridge (zero-latency)")
            if args.stream:
                print(f"  Companion stream: udp://{args.stream} (H.264 mpegts)")
        elif output_mode == "raw":
            print("  Video displayed in ffplay windows (no encoding)")
        elif output_mode == "udp":
            print(f"  FPV view:   ffplay {low_lat} udp://@:{args.port}")
            if chase_viewer_url:
                print(f"  Chase view: ffplay {low_lat} udp://@:{args.chase_port}")
        elif output_mode == "tcp":
            print(f"  FPV view:   ffplay {low_lat} tcp://<host>:{args.port}")
        elif output_mode == "rtsp":
            print(f"  FPV view:   ffplay {low_lat} rtsp://<host>:{args.port}/fpv")
    print()
    print("  Press Ctrl-C to stop")
    print("=" * 64)
    print()

    # ── 9. Keep alive (monitor critical processes, auto-restart ffmpeg) ──
    try:
        while True:
            # bf_sim_bridge is critical — if it dies, we're done
            if bf_bridge_proc.poll() is not None:
                log.error("bf_sim_bridge exited (code %d) — stopping", bf_bridge_proc.returncode)
                break

            # FPV image bridge is critical
            if bridge_proc and bridge_proc.poll() is not None:
                log.warning("FPV image bridge exited (code %d)", bridge_proc.returncode)
                break

            # Auto-restart FPV ffmpeg
            if ffmpeg_proc and ffmpeg_cmd and ffmpeg_proc.poll() is not None:
                rc = ffmpeg_proc.returncode
                log.info("FPV ffmpeg exited (code %d) — restarting …", rc)
                if ffmpeg_proc in pm.procs:
                    pm.procs.remove(ffmpeg_proc)
                time.sleep(1)
                ffmpeg_proc = pm.spawn(
                    ffmpeg_cmd,
                    stdin=bridge_proc.stdout,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )

            # Auto-restart chase ffmpeg
            if chase_ffmpeg_proc and chase_ffmpeg_cmd:
                if chase_bridge_proc and chase_bridge_proc.poll() is not None:
                    log.warning("Chase bridge exited (code %d)", chase_bridge_proc.returncode)
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
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    )

            time.sleep(2)
    except KeyboardInterrupt:
        pass

    pm.shutdown()
    log.info("Simulink simulation stopped")


if __name__ == "__main__":
    main()
