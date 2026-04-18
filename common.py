"""Shared utilities for betaloop simulation launchers (start.py / start_px4.py).

Centralises Gazebo environment setup, process management, display configuration,
camera topic discovery, image-bridge metadata reading, cleanup, and target
trajectory generation so that both BF and PX4 launchers stay in sync.
"""

import fcntl
import glob
import logging
import math
import os
import random
import select
import socket
import struct
import subprocess
import sys
import threading
import time

log = logging.getLogger(__name__)

# ── Paths ─────────────────────────────────────────────────────────────────────

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)


def default_path(env_var: str, repo_relative: str) -> str:
    """Return *env_var* override, or *repo_relative* joined to the repo root."""
    if os.environ.get(env_var):
        return os.environ[env_var]
    return os.path.join(_REPO_ROOT, repo_relative)


AEROLOOP_HOME = default_path("AEROLOOP_HOME", "aeroloop_gazebo")
SIMULINK_LIB = default_path(
    "SIMULINK_LIB",
    os.path.join(
        "HEAR_Simulations",
        "Compiled_models",
        "rocket_drone_quad_SITL",
        "libinterface_simulink.so",
    ),
)
IMAGE_BRIDGE = os.path.join(AEROLOOP_HOME, "plugins", "build", "gz_image_bridge")
TOPIC_MODEL_HINT_DEFAULT = "rocket_drone"
DEFAULT_DRONE = "rocket_drone"

# Per-drone reference calibration.  max_thrust is the total static thrust from
# all 4 rotors at vel_cmd_max.  *_per_kg are the moment-of-inertia values
# normalised to 1 kg so we can scale linearly with mass.
DRONE_REFS = {
    "rocket_drone": {
        "model_sdf": "rocket_drone/rocket_drone.sdf",
        "model_vis_sdf": "rocket_drone_vis/model.sdf",
        "model_uri": "model://rocket_drone",
        "model_vis_uri": "model://rocket_drone_vis",
        "max_thrust": 22.0,       # N — calibrated: 0.5 kg → CTW ≈ 4.5
        "ixx_per_kg": 0.019417,   # 0.029125 / 1.5
        "iyy_per_kg": 0.019417,
        "izz_per_kg": 0.036817,   # 0.055225 / 1.5
        "default_ctw": 4.5,
        "default_standoff": 0.20,
        "leg_attach_offset": 0.025,
        "default_damping": {
            "linear_x": 0.1, "linear_y": 0.4, "linear_z": 0.03,
            "quadratic_x": 0.001, "quadratic_y": 0.03, "quadratic_z": 0.001,
            "angular": 0.03,
        },
    },
    "thaqib_1_prototype": {
        "model_sdf": "thaqib_1_prototype/thaqib_1_prototype.sdf",
        "model_vis_sdf": "thaqib_1_prototype_vis/model.sdf",
        "model_uri": "model://thaqib_1_prototype",
        "model_vis_uri": "model://thaqib_1_prototype_vis",
        "max_thrust": 22.0,
        "ixx_per_kg": 0.019417,
        "iyy_per_kg": 0.019417,
        "izz_per_kg": 0.036817,
        "default_ctw": 4.5,
        "default_standoff": 0.20,
        "leg_attach_offset": 0.025,
        "default_damping": {
            "linear_x": 0.1, "linear_y": 0.4, "linear_z": 0.03,
            "quadratic_x": 0.001, "quadratic_y": 0.03, "quadratic_z": 0.001,
            "angular": 0.03,
        },
    },
    "iris": {
        "model_sdf": "betaloop_iris_with_standoffs/model.sdf",
        "model_vis_sdf": "iris_vis/model.sdf",
        "model_uri": "model://betaloop_iris_with_standoffs",
        "model_vis_uri": "model://iris_vis",
        "max_thrust": 8.0,        # N — estimated: 0.4 kg → CTW ≈ 2
        "ixx_per_kg": 0.005,      # 0.002 / 0.4
        "iyy_per_kg": 0.010,      # 0.004 / 0.4
        "izz_per_kg": 0.01125,    # 0.0045 / 0.4
        "default_ctw": 2.0,
        "default_standoff": 0.17,
        "leg_attach_offset": 0.025,
        "default_damping": {
            "linear_x": 0.3, "linear_y": 0.3, "linear_z": 0.3,
            "quadratic_x": 0, "quadratic_y": 0, "quadratic_z": 0,
            "angular": 0.005,
        },
    },
}


# ── Jinja2 Template Rendering ─────────────────────────────────────────────────

def render_template(j2_path: str, variables: dict) -> None:
    """Render a Jinja2 template (.j2) to the corresponding output file."""
    from jinja2 import Environment, FileSystemLoader

    template_dir = os.path.dirname(j2_path)
    template_name = os.path.basename(j2_path)
    output_path = j2_path[:-3]  # strip ".j2"
    env = Environment(loader=FileSystemLoader(template_dir), keep_trailing_newline=True)
    template = env.get_template(template_name)
    rendered = template.render(**variables)
    with open(output_path, "w") as f:
        f.write(rendered)
    log.info("Rendered %s", os.path.relpath(output_path, AEROLOOP_HOME))


def compute_model_vars(
    drone: str,
    ctw: float | None = None,
    cam_pitch: float = -80.0,
    standoff: float | None = None,
    damping_overrides: dict | None = None,
) -> dict:
    """Compute model template variables from drone ref and overrides.

    Returns a dict suitable for rendering model SDF templates.
    """
    ref = DRONE_REFS[drone]
    _ctw = ctw if ctw is not None else ref["default_ctw"]
    mass = ref["max_thrust"] / (_ctw * 9.81)
    ixx = ref["ixx_per_kg"] * mass
    iyy = ref["iyy_per_kg"] * mass
    izz = ref["izz_per_kg"] * mass

    _standoff = standoff if standoff is not None else ref["default_standoff"]
    leg_z = -(_standoff / 2 + ref["leg_attach_offset"])

    fpv_cam_pitch_rad = math.radians(cam_pitch)

    dd = ref["default_damping"]
    do = damping_overrides or {}
    model_vars = {
        "mass": mass, "ixx": ixx, "iyy": iyy, "izz": izz,
        "fpv_cam_pitch_rad": fpv_cam_pitch_rad,
        "standoff_height": _standoff, "leg_z": leg_z,
        "linear_damping_x": do.get("linear_x", dd["linear_x"]),
        "linear_damping_y": do.get("linear_y", dd["linear_y"]),
        "linear_damping_z": do.get("linear_z", dd["linear_z"]),
        "quadratic_damping_x": do.get("quadratic_x", dd["quadratic_x"]),
        "quadratic_damping_y": do.get("quadratic_y", dd["quadratic_y"]),
        "quadratic_damping_z": do.get("quadratic_z", dd["quadratic_z"]),
        "angular_damping": do.get("angular", dd["angular"]),
    }

    log.info("CTW=%.1f mass=%.3fkg Ixx=%.6f Iyy=%.6f Izz=%.6f standoff=%.3fm cam_pitch=%.1f°",
             _ctw, mass, ixx, iyy, izz, _standoff, cam_pitch)
    return model_vars


def compute_world_vars(
    drone: str,
    world_name: str,
    target_altitude: float | None = None,
    target_speed: float | None = None,
    orbit_radius: float | None = None,
    patrol_length: float | None = None,
    target_x: float | None = None,
    target_y: float | None = None,
) -> dict:
    """Compute world template variables from drone and world settings.

    Returns a dict suitable for rendering world SDF templates.
    """
    ref = DRONE_REFS[drone]

    _orbit_radius = orbit_radius if orbit_radius is not None else 30.0
    speed_kmh = target_speed if target_speed is not None else 5.4
    orbit_speed = (speed_kmh / 3.6) / _orbit_radius

    _patrol_length = patrol_length if patrol_length is not None else 500.0
    patrol_speed_kmh = target_speed if target_speed is not None else 20.0
    patrol_speed_ms = patrol_speed_kmh / 3.6

    _target_x = target_x if target_x is not None else 30.0
    _target_y = target_y if target_y is not None else 0.0
    if target_altitude is not None:
        _target_z = target_altitude
    elif world_name == "patrol_park":
        _target_z = 100.0
    elif world_name == "park_chase":
        _target_z = 50.0
    else:
        _target_z = 10.0

    return {
        "drone_uri": ref["model_uri"],
        "drone_vis_uri": ref["model_vis_uri"],
        "drone_name": drone,
        "orbit_speed": orbit_speed,
        "orbit_radius": _orbit_radius,
        "target_altitude": _target_z,
        "target_x": _target_x, "target_y": _target_y, "target_z": _target_z,
        "patrol_length": _patrol_length,
        "patrol_speed_ms": patrol_speed_ms,
    }


def render_vis_templates(
    drone: str,
    world_name: str,
    world_map: dict,
    model_vars: dict,
    world_vars: dict,
) -> None:
    """Render vis-only model + world templates (shared by BF and PX4 stacks)."""
    models_dir = os.path.join(AEROLOOP_HOME, "models")
    worlds_dir = os.path.join(AEROLOOP_HOME, "worlds")
    ref = DRONE_REFS[drone]

    # Vis model SDF
    vis_j2 = os.path.join(models_dir, ref["model_vis_sdf"] + ".j2")
    if os.path.isfile(vis_j2):
        render_template(vis_j2, model_vars)

    # Vis world SDF
    wm = world_map[world_name]
    vis_wj2 = os.path.join(worlds_dir, wm["sim_world"] + ".j2")
    if os.path.isfile(vis_wj2):
        render_template(vis_wj2, world_vars)


# ── Process Manager ───────────────────────────────────────────────────────────

class ProcessManager:
    """Track child processes for clean shutdown."""

    def __init__(self):
        self.procs: list[subprocess.Popen] = []

    def spawn(self, args, **kwargs):
        log.info("Starting: %s", " ".join(str(a) for a in args[:6]))
        p = subprocess.Popen(args, **kwargs)
        self.procs.append(p)
        return p

    def shutdown(self, extra_pkill_patterns: list[str] | None = None):
        log.info("Shutting down %d processes …", len(self.procs))
        for p in reversed(self.procs):
            if p.poll() is None:
                p.terminate()
        for p in reversed(self.procs):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                try:
                    p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
        # Safety net: kill orphaned simulation processes
        patterns = ["gz sim", "ruby.*gz", "gz_image_bridge"]
        if extra_pkill_patterns:
            patterns.extend(extra_pkill_patterns)
        for pattern in patterns:
            try:
                subprocess.run(
                    ["pkill", "-9", "-f", pattern],
                    capture_output=True, timeout=3,
                )
            except (subprocess.TimeoutExpired, OSError):
                pass


# ── Gazebo Environment ────────────────────────────────────────────────────────

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
    _prepend("GZ_SIM_SYSTEM_PLUGIN_PATH", plugins,
             "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins")
    _prepend("LD_LIBRARY_PATH", "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins")

    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


# ── GPU & Display ─────────────────────────────────────────────────────────────

def has_nvidia_gpu() -> bool:
    """Check if an NVIDIA GPU is accessible."""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=5,
        )
        return result.returncode == 0 and result.stdout.strip() != ""
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def configure_display(args, pm: ProcessManager):
    """Set up GPU detection, rendering, and display (Xvfb if needed).

    *args* must have a ``.gazebo`` boolean attribute.
    """
    gpu_available = has_nvidia_gpu()

    if gpu_available:
        log.info("NVIDIA GPU detected — GPU-accelerated rendering")
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
        nvidia_icd = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.isfile(nvidia_icd):
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_icd
    else:
        log.info("No GPU detected — software rendering (llvmpipe)")
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    if args.gazebo:
        if not os.environ.get("DISPLAY"):
            log.error("No DISPLAY set — the Gazebo GUI needs a display.")
            sys.exit(1)
        os.environ.pop("__GLX_VENDOR_LIBRARY_NAME", None)
        log.info("Using display %s for Gazebo GUI", os.environ["DISPLAY"])
    elif os.environ.get("DISPLAY"):
        log.info("Using native display %s", os.environ["DISPLAY"])
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


# ── Camera Topics ─────────────────────────────────────────────────────────────

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
    """Find a camera image topic matching *name_hint*."""
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
    """Read IMGMETA line from the bridge's stderr → (width, height, pix_fmt)."""
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


# ── Cleanup ───────────────────────────────────────────────────────────────────

def cleanup_before_start(extra_pkill_cmds: list[str] | None = None):
    """Kill stale processes from previous runs and remove orphaned resources."""
    log.info("Cleaning up from previous runs...")
    cmds = [
        "pkill -9 -f 'gz sim' 2>/dev/null || true",
        "pkill -9 -f gz_image_bridge 2>/dev/null || true",
        "pkill -9 Xvfb 2>/dev/null || true",
    ]
    if extra_pkill_cmds:
        cmds.extend(extra_pkill_cmds)
    for cmd in cmds:
        try:
            subprocess.run(["bash", "-c", cmd], capture_output=True, timeout=5)
        except (subprocess.TimeoutExpired, OSError):
            pass
        time.sleep(0.2)

    for path in glob.glob("/dev/shm/gz_cam_*"):
        try:
            os.remove(path)
            log.info("Removed stale SHM: %s", path)
        except OSError:
            pass
    log.info("Cleanup complete")


# ── TCP port wait ─────────────────────────────────────────────────────────────

def wait_for_port(host: str, port: int, timeout: float = 30) -> bool:
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


# ── Video pipeline helpers ────────────────────────────────────────────────────

def start_fpv_bridge(args, pm: ProcessManager, osd_args=None):
    """Discover and start the FPV image bridge.

    Returns ``(proc, width, height)`` or ``(None, 0, 0)`` if ``--no-video``.
    *args* must have: ``no_video``, ``fpv_topic``, ``topic_model_hint``,
    ``no_display``.
    *osd_args*: optional list of OSD-related CLI flags for gz_image_bridge.
    When provided, replaces the default ``--no-osd``.
    """
    if args.no_video:
        log.info("Video pipeline disabled (--no-video)")
        return None, 0, 0

    if not os.path.isfile(IMAGE_BRIDGE):
        log.error("gz_image_bridge not found: %s — run build_plugin.sh", IMAGE_BRIDGE)
        pm.shutdown()
        sys.exit(1)

    if args.fpv_topic:
        topic = args.fpv_topic
    else:
        log.info("Discovering FPV camera image topic …")
        topic = discover_camera_topic(
            name_hint="fpv_cam", timeout=30,
            model_hint=getattr(args, "topic_model_hint", TOPIC_MODEL_HINT_DEFAULT),
        )
        if not topic:
            log.error("Could not find FPV camera topic")
            pm.shutdown()
            sys.exit(1)
    log.info("FPV topic: %s", topic)

    bridge_cmd = [IMAGE_BRIDGE, topic, "--display"]
    if osd_args:
        bridge_cmd.extend(osd_args)
    else:
        bridge_cmd.append("--no-osd")
    if args.no_display:
        bridge_cmd.append("--hidden")

    proc = pm.spawn(bridge_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    flags = fcntl.fcntl(proc.stderr, fcntl.F_GETFL)
    fcntl.fcntl(proc.stderr, fcntl.F_SETFL, flags | os.O_NONBLOCK)

    log.info("Waiting for first camera frame …")
    width, height, pix_fmt = read_image_meta(proc, timeout=30)
    if width is None:
        log.error("No image metadata from bridge — camera may not be rendering")
        pm.shutdown()
        sys.exit(1)
    log.info("Camera: %dx%d %s", width, height, pix_fmt)

    # Drain stderr in a background thread so the pipe buffer never fills up
    # and blocks the child process (which would deadlock its main loop).
    def _drain(f):
        try:
            while True:
                chunk = f.read(4096)
                if not chunk:
                    break
        except Exception:
            pass

    t = threading.Thread(target=_drain, args=(proc.stderr,), daemon=True)
    t.start()

    return proc, width, height


def start_chase_bridge(args, pm: ProcessManager):
    """Discover and start the chase camera image bridge.

    Returns the process or ``None``.
    *args* must have: ``chase_cam``, ``chase_topic``, ``topic_model_hint``.
    """
    if not getattr(args, "chase_cam", False):
        return None

    if args.chase_topic:
        chase_topic = args.chase_topic
    else:
        chase_topic = discover_camera_topic(
            name_hint="chase_cam", timeout=30,
            model_hint=getattr(args, "topic_model_hint", TOPIC_MODEL_HINT_DEFAULT),
        )
    if not chase_topic:
        log.warning("Chase camera topic not found — FPV only")
        return None

    log.info("Chase topic: %s", chase_topic)
    chase_cmd = [IMAGE_BRIDGE, chase_topic, "--display", "--no-osd"]
    if getattr(args, "no_display", False):
        chase_cmd.append("--hidden")
    return pm.spawn(chase_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ── Target Trajectory Threads ─────────────────────────────────────────────────

# UDP ports — must match ExternalPosePlugin <listen_port> in world SDF files.
TARGET_UDP_PORT = 9016
TARGET_RESET_PORT = 9017


def start_orbit_thread(
    stop_event: threading.Event,
    orbit_radius: float = 30.0,
    orbit_omega: float = 0.05,
    target_z: float = 50.0,
    udp_port: int = TARGET_UDP_PORT,
    reset_port: int = TARGET_RESET_PORT,
) -> threading.Thread:
    """Spawn a daemon thread that drives a circular orbit via UDP.

    Sends a 72-byte VisualPosePacket at ~60 Hz to ``udp_port``.
    Listens on ``reset_port`` for reset signals.
    Returns the started thread.
    """

    def _orbit_loop():
        interval = 1.0 / 60
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = ("127.0.0.1", udp_port)
        packer = struct.Struct("<Qd3d4d")
        seq = 0
        t0 = time.monotonic()
        theta = 0.0
        t_prev = t0

        rst_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rst_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rst_sock.bind(("0.0.0.0", reset_port))
        rst_sock.setblocking(False)

        log.info("Orbit thread: R=%.1fm omega=%.4f rad/s alt=%.0fm (port %d)",
                 orbit_radius, orbit_omega, target_z, udp_port)

        while not stop_event.is_set():
            try:
                while True:
                    rst_sock.recv(64)
                    theta = 0.0
                    t_prev = time.monotonic()
                    log.info("Orbit thread: reset to theta=0")
            except BlockingIOError:
                pass

            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now
            theta += orbit_omega * dt

            x = orbit_radius * math.cos(theta)
            y = orbit_radius * math.sin(theta)
            yaw = theta + math.pi / 2
            qw = math.cos(yaw / 2)
            qz = math.sin(yaw / 2)

            pkt = packer.pack(seq, t_now - t0, x, y, target_z,
                              qw, 0.0, 0.0, qz)
            try:
                sock.sendto(pkt, addr)
            except OSError:
                pass
            seq += 1
            stop_event.wait(timeout=interval)

        sock.close()
        rst_sock.close()

    t = threading.Thread(target=_orbit_loop, daemon=True, name="orbit")
    t.start()
    return t


def start_patrol_thread(
    stop_event: threading.Event,
    half_length: float = 250.0,
    speed_ms: float = 5.56,
    target_z: float = 100.0,
    sine_amp_xy: float = 0.0,
    sine_period_xy: float = 200.0,
    sine_amp_z: float = 0.0,
    sine_period_z: float = 200.0,
    udp_port: int = TARGET_UDP_PORT,
    reset_port: int = TARGET_RESET_PORT,
) -> threading.Thread:
    """Spawn a daemon thread that drives a triangle-wave patrol via UDP.

    Sends a 72-byte VisualPosePacket at ~60 Hz to ``udp_port``.
    Listens on ``reset_port`` for reset signals.
    Returns the started thread.
    """

    def _patrol_loop():
        interval = 1.0 / 60
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = ("127.0.0.1", udp_port)
        packer = struct.Struct("<Qd3d4d")
        seq = 0
        t0 = time.monotonic()
        t_prev = t0
        x = 0.0
        direction = 1.0
        qw, qz = 1.0, 0.0

        rst_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rst_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rst_sock.bind(("0.0.0.0", reset_port))
        rst_sock.setblocking(False)

        omega_xy = (2.0 * math.pi / sine_period_xy) if sine_period_xy > 0 else 0.0
        omega_z = (2.0 * math.pi / sine_period_z) if sine_period_z > 0 else 0.0

        log.info("Patrol thread: half=%.0fm speed=%.1f m/s alt=%.0fm (port %d)",
                 half_length, speed_ms, target_z, udp_port)

        while not stop_event.is_set():
            try:
                while True:
                    rst_sock.recv(64)
                    x = 0.0
                    direction = 1.0
                    qw, qz = 1.0, 0.0
                    t_prev = time.monotonic()
                    log.info("Patrol thread: reset to origin")
            except BlockingIOError:
                pass

            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now

            x += direction * speed_ms * dt
            if x >= half_length:
                x = half_length
                direction = -1.0
                qw, qz = 0.0, 1.0
            elif x <= -half_length:
                x = -half_length
                direction = 1.0
                qw, qz = 1.0, 0.0

            y_offset = sine_amp_xy * math.sin(omega_xy * x) if sine_amp_xy else 0.0
            z_offset = sine_amp_z * math.sin(omega_z * x) if sine_amp_z else 0.0

            pkt = packer.pack(seq, t_now - t0, x, y_offset, target_z + z_offset,
                              qw, 0.0, 0.0, qz)
            try:
                sock.sendto(pkt, addr)
            except OSError:
                pass
            seq += 1
            stop_event.wait(timeout=interval)

        sock.close()
        rst_sock.close()

    t = threading.Thread(target=_patrol_loop, daemon=True, name="patrol")
    t.start()
    return t


BALLOON_UDP_PORT = 9014


def start_balloon_thread(
    stop_event: threading.Event,
    mean_x: float = 30.0,
    mean_y: float = 0.0,
    mean_z: float = 10.0,
    wind_intensity: float = 2.0,
    wind_randomness: float = 1.0,
    drift_speed: float = 20.0,
    udp_port: int = BALLOON_UDP_PORT,
) -> threading.Thread:
    """Spawn a daemon thread that drives smooth Lissajous balloon motion.

    Sends a 72-byte VisualPosePacket at ~60 Hz to ``udp_port``.
    Returns the started thread.
    """

    def _wind_loop():
        interval = 1.0 / 60  # 60 Hz
        amp = wind_intensity
        amp_z = wind_randomness
        spd = drift_speed

        px = [random.uniform(0, 2 * math.pi) for _ in range(3)]
        py = [random.uniform(0, 2 * math.pi) for _ in range(3)]
        pz = [random.uniform(0, 2 * math.pi) for _ in range(3)]

        fx = [0.13 * spd, 0.31 * spd, 0.53 * spd]
        fy = [0.17 * spd, 0.41 * spd, 0.67 * spd]
        fz = [0.11 * spd, 0.29 * spd, 0.47 * spd]

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = ("127.0.0.1", udp_port)
        packer = struct.Struct("<Qd3d4d")
        seq = 0
        t0 = time.monotonic()

        log.info("Balloon drift (UDP:%d): amp=%.1f m  bob=%.1f m  speed=%.1fx",
                 udp_port, amp, amp_z, spd)

        while not stop_event.is_set():
            t = time.monotonic() - t0

            bx = mean_x + amp * (0.5 * math.sin(fx[0] * t + px[0])
                               + 0.3 * math.sin(fx[1] * t + px[1])
                               + 0.2 * math.sin(fx[2] * t + px[2]))
            by = mean_y + amp * (0.5 * math.sin(fy[0] * t + py[0])
                               + 0.3 * math.sin(fy[1] * t + py[1])
                               + 0.2 * math.sin(fy[2] * t + py[2]))
            bz = mean_z + amp_z * (0.4 * math.sin(fz[0] * t + pz[0])
                                 + 0.35 * math.sin(fz[1] * t + pz[1])
                                 + 0.25 * math.sin(fz[2] * t + pz[2]))

            # identity quaternion (w=1, x=0, y=0, z=0)
            pkt = packer.pack(seq, t, bx, by, bz, 1.0, 0.0, 0.0, 0.0)
            try:
                sock.sendto(pkt, addr)
            except OSError:
                pass
            seq += 1

            stop_event.wait(timeout=interval)

        sock.close()

    t = threading.Thread(target=_wind_loop, daemon=True, name="balloon_wind")
    t.start()
    return t
