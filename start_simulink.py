#!/usr/bin/env python3
"""Simulink dynamics backend launcher — bf_sim_bridge + Betaflight SITL + Gazebo vis-only.

Replaces Gazebo physics with the HEAR Simulink rigid-body dynamics solver.
Gazebo becomes a pose-driven visualizer and camera renderer only.

Process launch order:
    1. [Xvfb]       — virtual display (headless only)
    2. Gazebo        — vis-only world (rocket_drone_vis.sdf), no physics
    3. bf_sim_bridge — Simulink dynamics, UDP ↔ BF SITL + Gazebo
    4. Betaflight SITL
    5. [MSP Virtual Radio]
    6. gz_image_bridge → SDL2 window  — FPV video pipeline

Usage:
    python3 start_simulink.py --display --shm          # SDL2 + shared memory
    python3 start_simulink.py --gazebo                  # Show Gazebo GUI
    python3 start_simulink.py --no-video                # Dynamics only, no video
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

VIS_WORLD = "rocket_drone_vis.sdf"
TOPIC_MODEL_HINT = "betaflight_vehicle"
IMAGE_BRIDGE = os.path.join(AEROLOOP_HOME, "plugins", "build", "gz_image_bridge")


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
    parser.add_argument("--gazebo", action="store_true",
                        help="Show the Gazebo GUI (default: headless)")
    parser.add_argument("--no-transmitter", action="store_true",
                        help="Skip starting the MSP virtual radio")
    parser.add_argument("--no-video", action="store_true",
                        help="Skip the video pipeline (Gazebo + bridge only)")
    parser.add_argument("--chase-cam", action="store_true",
                        help="Also display the chase camera (3rd-person SDL2 window)")
    parser.add_argument("--fpv-topic", default=None,
                        help="Explicit Gazebo FPV image topic (skip auto-discovery)")
    parser.add_argument("--chase-topic", default=None,
                        help="Explicit Gazebo chase image topic (skip auto-discovery)")
    parser.add_argument("--osd", action="store_true",
                        help="Enable Betaflight OSD overlay on the FPV stream")
    parser.add_argument("--msp-port", type=int, default=5762,
                        help="BF MSP TCP port for OSD telemetry (default: 5762)")
    parser.add_argument("--display", action="store_true", default=True,
                        help="SDL2 direct display in gz_image_bridge (default: enabled)")
    parser.add_argument("--shm", action="store_true",
                        help="Expose frames via POSIX shared memory for local tracker (zero-latency)")
    args = parser.parse_args()

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

    # ── 2. Gazebo (vis-only world) ──
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
    width = height = 0
    chase_topic = None
    chase_bridge_proc = None

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

        # Start image bridge (always --display)
        img_bridge_cmd = [IMAGE_BRIDGE, topic, "--display"]
        if args.osd:
            img_bridge_cmd += ["--osd", "--msp-port", str(args.msp_port)]
            log.info("OSD overlay enabled (MSP port %d)", args.msp_port)
        if args.shm:
            img_bridge_cmd += ["--shm"]
            log.info("Shared memory frame server enabled")

        bridge_proc = pm.spawn(
            img_bridge_cmd,
            stdout=subprocess.DEVNULL,
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

        # Chase camera (SDL2 window)
        if chase_topic:
            log.info("Starting chase camera SDL2 window")
            chase_bridge_proc = pm.spawn(
                [IMAGE_BRIDGE, chase_topic, "--display"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    # ── 8. Print connection info ──
    print()
    print("=" * 64)
    print("  Simulink Dynamics Backend — Running  (SDL2 display)")
    print("=" * 64)
    print()
    print(f"  Backend      : bf_sim_bridge → Simulink → BF SITL (250 Hz)")
    print(f"  World        : {os.path.basename(args.world)} (vis-only, no GZ physics)")
    print(f"  Simulink .so : {os.path.basename(args.sim_lib)}")
    if not args.no_video:
        print(f"  FPV window   : SDL2 (gz_image_bridge)")
        if chase_bridge_proc:
            print(f"  Chase window : SDL2 (gz_image_bridge)")
        print(f"  FPV topic    : {topic}")
        if chase_topic:
            print(f"  Chase topic  : {chase_topic}")
        print(f"  Resolution   : {width}x{height}")
        if args.shm:
            print(f"  Shared mem   : /dev/shm (POSIX)")
    else:
        print(f"  Video        : disabled (--no-video)")
    print(f"  RC input     : UDP 127.0.0.1:9004  (flight_test.py / virtual_rc.py)")
    print(f"  BF CLI       : TCP 127.0.0.1:5761")
    print(f"  BF Config    : TCP 127.0.0.1:5760")
    print()
    print("  Press Ctrl-C to stop")
    print("=" * 64)
    print()

    # ── 9. Keep alive ──
    try:
        while True:
            if bf_bridge_proc.poll() is not None:
                log.error("bf_sim_bridge exited (code %d) — stopping", bf_bridge_proc.returncode)
                break
            if bridge_proc and bridge_proc.poll() is not None:
                log.warning("FPV image bridge exited (code %d)", bridge_proc.returncode)
                break
            if chase_bridge_proc and chase_bridge_proc.poll() is not None:
                log.warning("Chase camera bridge exited (code %d)", chase_bridge_proc.returncode)
                chase_bridge_proc = None
            time.sleep(2)
    except KeyboardInterrupt:
        pass

    pm.shutdown()
    log.info("Simulink simulation stopped")


if __name__ == "__main__":
    main()
