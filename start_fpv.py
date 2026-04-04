#!/usr/bin/env python3
"""FPV simulation launcher — Betaflight SITL + Gazebo camera → SDL2 display.

Starts the full stack and renders the on-board camera feed in an SDL2 window
via gz_image_bridge.  Frames can also be exposed via POSIX shared memory
(--shm) for local trackers.

Architecture:
    Gazebo (camera sensor) ──gz-transport──► gz_image_bridge ──► SDL2 window
                                                               └─► /dev/shm (--shm)

Usage:
    python3 start_fpv.py --display --shm               # SDL2 + shared memory
    python3 start_fpv.py --display --shm --chase-cam   # + chase camera
    python3 start_fpv.py --display --shm --osd         # + OSD overlay
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
# Auto-detect paths: if running from the repo (betaloop/ dir), resolve relative
# to the repo root.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)

def _default_path(env_var, repo_relative):
    """Return env override, or repo-relative path."""
    if os.environ.get(env_var):
        return os.environ[env_var]
    return os.path.join(_REPO_ROOT, repo_relative)

AEROLOOP_HOME = _default_path("AEROLOOP_HOME", "aeroloop_gazebo")
BF_ELF = _default_path("BF_ELF", os.path.join("betaflight", "obj", "main", "betaflight_SITL.elf"))
MSP_RADIO_HOME = _default_path("MspVirtualRadioHome", os.path.join("..", "..", "msp_virtualradio"))
FPV_WORLD = "fpv_demo_harmonic.sdf"
IMAGE_BRIDGE = os.path.join(AEROLOOP_HOME, "plugins", "build", "gz_image_bridge")


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

    # os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


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


def discover_camera_topic(name_hint="fpv_cam", timeout=30):
    """List Gazebo topics and find a camera image topic matching *name_hint*.

    If *name_hint* is None, returns the first ``*/image`` topic found.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True, text=True, timeout=10,
            )
            for line in result.stdout.strip().splitlines():
                line = line.strip()
                if not line.endswith("/image"):
                    continue
                if name_hint and name_hint not in line:
                    continue
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
    parser.add_argument("--gazebo", action="store_true",
                        help="Show the Gazebo GUI (default: headless)")
    parser.add_argument("--no-transmitter", action="store_true",
                        help="Skip starting the MSP virtual radio")
    parser.add_argument("--chase-cam", action="store_true",
                        help="Also display the chase camera (3rd-person SDL2 window)")
    parser.add_argument("--osd", action="store_true",
                        help="Enable Betaflight OSD overlay on the FPV stream")
    parser.add_argument("--msp-port", type=int, default=5763,
                        help="Betaflight MSP TCP port for OSD telemetry (default: 5763 = UART3)")
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

    # ── 1b. GPU detection & rendering setup ──
    gpu_available = has_nvidia_gpu()

    if gpu_available:
        log.info("NVIDIA GPU detected — using GPU-accelerated rendering")
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)

        # Force NVIDIA EGL vendor ICD — Ogre2's camera sensor uses EGL for
        # off-screen rendering.  Without this, Mesa's DRI2 path is tried and
        # falls back to software ("failed to create dri2 screen").
        nvidia_icd = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.isfile(nvidia_icd):
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_icd
            log.info("Forcing NVIDIA EGL vendor ICD")

        log.info("Native GLX, forced EGL")

    else:
        log.info("No NVIDIA GPU detected — using default Mesa rendering")
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
        os.environ.pop("__GLX_VENDOR_LIBRARY_NAME", None)
        os.environ.pop("__EGL_VENDOR_LIBRARY_FILENAMES", None)

    # ── 1c. Display setup ──
    # Ogre2's camera sensor needs a display context to render frames.
    if args.gazebo:
        # GUI mode — need the host's real X display
        if not os.environ.get("DISPLAY"):
            log.error(
                "No DISPLAY set — the Gazebo GUI needs a display.\n"
                "  Set DISPLAY env var before running."
            )
            sys.exit(1)
        os.environ.pop("__GLX_VENDOR_LIBRARY_NAME", None)
        log.info("Using display %s for Gazebo GUI", os.environ["DISPLAY"])
    elif os.environ.get("DISPLAY"):
        # Headless mode — use the native display (low latency, no Xvfb).
        log.info("Using native display %s (low-latency host mode)", os.environ["DISPLAY"])
    else:
        # No display — start Xvfb
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

    # ── 2. Gazebo ──
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
    log.info("Discovering FPV camera image topic …")
    topic = discover_camera_topic(name_hint="fpv_cam", timeout=30)
    if not topic:
        log.error(
            "Could not find a camera image topic. "
            "Verify the world SDF includes a model with a camera sensor."
        )
        pm.shutdown()
        sys.exit(1)
    log.info("Found FPV camera topic: %s", topic)

    chase_topic = None
    if args.chase_cam:
        log.info("Discovering chase camera image topic …")
        chase_topic = discover_camera_topic(name_hint="chase_cam", timeout=30)
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

    bridge_cmd = [IMAGE_BRIDGE, topic, "--display"]
    if args.osd:
        bridge_cmd += ["--osd", "--msp-port", str(args.msp_port)]
        log.info("OSD overlay enabled (MSP port %d)", args.msp_port)
    if args.shm:
        bridge_cmd += ["--shm"]
        log.info("Shared memory frame server enabled")
    log.info("SDL2 direct display mode (zero-latency)")

    bridge_proc = pm.spawn(
        bridge_cmd,
        stdout=subprocess.DEVNULL,
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

    # ── 8. Chase camera (optional) ──
    chase_bridge_proc = None
    if chase_topic:
        log.info("Starting chase camera bridge (SDL2 display)")
        chase_cmd = [IMAGE_BRIDGE, chase_topic, "--display"]
        chase_bridge_proc = pm.spawn(
            chase_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )

    # ── 9. Print connection info ──
    print()
    print("=" * 60)
    print("  FPV Simulation Running")
    print("=" * 60)
    print()
    print(f"  FPV display  : SDL2 window (zero-latency)")
    if chase_topic:
        print(f"  Chase display: SDL2 window (zero-latency)")
    if args.shm:
        print(f"  Shared memory: /dev/shm (zero-copy)")
    print(f"  RC input     : UDP 127.0.0.1:9004  (flight_test.py)")
    print(f"  BF CLI       : TCP 127.0.0.1:5761")
    print(f"  BF Configurator: TCP 127.0.0.1:5760")
    print(f"  FPV topic    : {topic}")
    if chase_topic:
        print(f"  Chase topic  : {chase_topic}")
    print(f"  Resolution   : {width}x{height}")
    print()
    print("  Press Ctrl-C to stop")
    print("=" * 60)
    print()

    # ── 10. Keep alive ──
    try:
        while True:
            if bridge_proc.poll() is not None:
                log.warning("FPV image bridge exited (code %d)", bridge_proc.returncode)
                break
            if chase_bridge_proc and chase_bridge_proc.poll() is not None:
                log.warning("Chase camera bridge exited (code %d)", chase_bridge_proc.returncode)
                chase_bridge_proc = None
            time.sleep(2)
    except KeyboardInterrupt:
        pass

    pm.shutdown()
    log.info("FPV simulation stopped")


if __name__ == "__main__":
    main()
