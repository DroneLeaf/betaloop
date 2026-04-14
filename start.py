#!/usr/bin/env python3
"""Unified simulation launcher — Betaflight SITL + Gazebo + optional Simulink dynamics.

Consolidates all betaloop start scripts into a single entry point.  Choose the
world file and physics engine on the command line; OSD is always enabled.

Supported physics backends:
  - **gazebo** (default) — BetaflightPlugin drives motor forces in Gazebo's
    rigid-body solver.
  - **simulink** — bf_sim_bridge loads a pre-compiled Simulink .so and steps
    it at 250 Hz; Gazebo is a visualizer only.

Usage:
    # Gazebo physics (default)
    python3 start.py --world park_chase --gazebo
    python3 start.py --world collision_test --cam-pitch -90 --gazebo
    python3 start.py --drone iris --gazebo --chase-cam

    # Simulink dynamics backend (world file selected automatically)
    python3 start.py --world park_chase --physics simulink --gazebo --chase-cam
    python3 start.py --world collision_test --physics simulink --gazebo

    # Tune drone params
    python3 start.py --world park_chase --ctw 5 --angular-damping 0.05
"""

import argparse
import fcntl
import logging
import math
import os
import signal
import socket
import subprocess
import sys
import time

from jinja2 import Environment, FileSystemLoader

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("betaloop")


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
    os.path.join(
        "HEAR_Simulations",
        "Compiled_models",
        "rocket_drone_quad_SITL",
        "libinterface_simulink.so",
    ),
)

DEFAULT_WORLD = "park_chase"
DEFAULT_DRONE = "rocket_drone"
TOPIC_MODEL_HINT_DEFAULT = "rocket_drone"
IMAGE_BRIDGE = os.path.join(AEROLOOP_HOME, "plugins", "build", "gz_image_bridge")

# Short world name → dict of world attributes.
#   gz_world     — Gazebo physics world file
#   sim_world    — Simulink vis-only SDF file
#   gz_name      — Gazebo <world name> element
#   target_model — SDF model name of the target (for proximity OSD)
#   target_bbox  — half-extents "X,Y,Z" in metres (axis-aligned)
WORLD_MAP = {
    "park_chase": {
        "gz_world":     "rocket_drone_park_chase.world",
        "sim_world":    "rocket_drone_park_chase_vis.sdf",
        "gz_name":      "fpv_chase_park",
        "target_model": "moving_target_drone",
        "target_bbox":  "0.792,1.047,0.186",
    },
    "collision_test": {
        "gz_world":     "rocket_drone_collision_test.world",
        "sim_world":    "rocket_drone_collision_test_vis.sdf",
        "gz_name":      "collision_test",
        "target_model": "collision_test_target",
        "target_bbox":  "0.792,1.047,0.186",
    },
}

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
        for pattern in ["gz sim", "ruby.*gz", "gz_image_bridge", "betaflight_SITL.elf"]:
            try:
                subprocess.run(
                    ["pkill", "-9", "-f", pattern],
                    capture_output=True,
                    timeout=3,
                )
            except (subprocess.TimeoutExpired, OSError):
                pass


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
    _prepend(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        plugins,
        "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins",
    )
    _prepend("LD_LIBRARY_PATH", "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins")

    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


def _render_template(j2_path, variables):
    """Render a Jinja2 template (.j2) to the corresponding output file."""
    template_dir = os.path.dirname(j2_path)
    template_name = os.path.basename(j2_path)
    output_path = j2_path[:-3]  # strip ".j2"
    env = Environment(loader=FileSystemLoader(template_dir), keep_trailing_newline=True)
    template = env.get_template(template_name)
    rendered = template.render(**variables)
    with open(output_path, "w") as f:
        f.write(rendered)
    log.info("Rendered %s", os.path.relpath(output_path, AEROLOOP_HOME))


def _render_all_templates(drone, world_name, args):
    """Render all Jinja2 templates for the selected drone and world."""
    ref = DRONE_REFS[drone]
    models_dir = os.path.join(AEROLOOP_HOME, "models")
    worlds_dir = os.path.join(AEROLOOP_HOME, "worlds")

    # ── Compute model variables ──
    ctw = args.ctw if args.ctw is not None else ref["default_ctw"]
    mass = ref["max_thrust"] / (ctw * 9.81)
    ixx = ref["ixx_per_kg"] * mass
    iyy = ref["iyy_per_kg"] * mass
    izz = ref["izz_per_kg"] * mass

    standoff = args.standoff_height if args.standoff_height is not None else ref["default_standoff"]
    leg_z = -(standoff / 2 + ref["leg_attach_offset"])

    fpv_cam_pitch_rad = math.radians(args.cam_pitch)

    dd = ref["default_damping"]
    lin_x = args.linear_damping_x if args.linear_damping_x is not None else dd["linear_x"]
    lin_y = args.linear_damping_y if args.linear_damping_y is not None else dd["linear_y"]
    lin_z = args.linear_damping_z if args.linear_damping_z is not None else dd["linear_z"]
    quad_x = args.quadratic_damping_x if args.quadratic_damping_x is not None else dd["quadratic_x"]
    quad_y = args.quadratic_damping_y if args.quadratic_damping_y is not None else dd["quadratic_y"]
    quad_z = args.quadratic_damping_z if args.quadratic_damping_z is not None else dd["quadratic_z"]
    ang = args.angular_damping if args.angular_damping is not None else dd["angular"]

    model_vars = {
        "mass": mass, "ixx": ixx, "iyy": iyy, "izz": izz,
        "fpv_cam_pitch_rad": fpv_cam_pitch_rad,
        "standoff_height": standoff, "leg_z": leg_z,
        "linear_damping_x": lin_x, "linear_damping_y": lin_y, "linear_damping_z": lin_z,
        "quadratic_damping_x": quad_x, "quadratic_damping_y": quad_y, "quadratic_damping_z": quad_z,
        "angular_damping": ang,
    }

    log.info("CTW=%.1f mass=%.3fkg Ixx=%.6f Iyy=%.6f Izz=%.6f standoff=%.3fm cam_pitch=%.1f°",
             ctw, mass, ixx, iyy, izz, standoff, args.cam_pitch)

    # ── Render physics model SDF ──
    phys_j2 = os.path.join(models_dir, ref["model_sdf"] + ".j2")
    if os.path.isfile(phys_j2):
        _render_template(phys_j2, model_vars)

    # ── Render vis model SDF ──
    vis_j2 = os.path.join(models_dir, ref["model_vis_sdf"] + ".j2")
    if os.path.isfile(vis_j2):
        _render_template(vis_j2, model_vars)

    # ── Compute world variables ──
    # Park-chase orbit: user specifies tangential speed in km/h and radius.
    # omega (rad/s) = v_tangential / radius,  v_tangential = speed_kmh / 3.6
    orbit_radius = args.target_orbit_radius if args.target_orbit_radius is not None else 30.0
    speed_kmh = args.target_speed if args.target_speed is not None else 5.4  # ~1.5 m/s ≈ 0.05 rad/s @ 30 m
    orbit_speed = (speed_kmh / 3.6) / orbit_radius

    target_x = args.target_distance_x if args.target_distance_x is not None else 30.0
    target_y = args.target_distance_y if args.target_distance_y is not None else 0.0
    # target_z: collision_test default 10, park_chase default 50
    if args.target_altitude is not None:
        target_z = args.target_altitude
    elif world_name == "park_chase":
        target_z = 50.0
    else:
        target_z = 10.0

    world_vars = {
        "drone_uri": ref["model_uri"],
        "drone_vis_uri": ref["model_vis_uri"],
        "drone_name": drone,
        "orbit_speed": orbit_speed,
        "orbit_radius": orbit_radius,
        "target_altitude": target_z,
        "target_x": target_x, "target_y": target_y, "target_z": target_z,
    }

    # ── Render world files ──
    wm = WORLD_MAP[world_name]
    for wf in [wm["gz_world"], wm["sim_world"]]:
        wj2 = os.path.join(worlds_dir, wf + ".j2")
        if os.path.isfile(wj2):
            _render_template(wj2, world_vars)


def cleanup_before_start():
    """Kill stale processes from previous runs and remove orphaned resources."""
    log.info("Cleaning up from previous runs...")
    cleanup_cmds = [
        ("existing Gazebo processes", "pkill -9 -f 'gz sim' 2>/dev/null || true"),
        ("existing image bridge processes", "pkill -9 -f 'gz_image_bridge' 2>/dev/null || true"),
        ("existing Betaflight SITL processes", "pkill -9 -f 'betaflight_SITL.elf' 2>/dev/null || true"),
        ("existing MSP Virtual Radio processes", "pkill -9 -f 'msp_virtualradio/index.js' 2>/dev/null || true"),
        ("existing bf_sim_bridge processes", "pkill -9 -f 'bf_sim_bridge' 2>/dev/null || true"),
        ("existing Xvfb processes", "pkill -9 Xvfb 2>/dev/null || true"),
    ]
    for label, cmd in cleanup_cmds:
        try:
            subprocess.run(["bash", "-c", cmd], capture_output=True, timeout=5)
        except (subprocess.TimeoutExpired, OSError):
            pass
        time.sleep(0.2)

    # Remove stale POSIX shared memory segments left by killed gz_image_bridge
    # processes (SIGKILL skips the C++ cleanup handler).  Readers that still
    # hold an old fd will see a different inode after the bridge recreates the
    # segment and must reopen — clearing them here ensures no stale data.
    import glob
    stale_shm = glob.glob("/dev/shm/gz_cam_*")
    if stale_shm:
        for path in stale_shm:
            try:
                os.remove(path)
                log.info("Removed stale SHM: %s", path)
            except OSError:
                pass

    log.info("Cleanup complete")


def has_nvidia_gpu():
    """Check if an NVIDIA GPU is accessible."""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return result.returncode == 0 and result.stdout.strip() != ""
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def list_camera_topics(name_hint=None):
    """Return sorted camera image topics, optionally filtered by substring."""
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True,
            text=True,
            timeout=10,
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


def configure_display(args, pm):
    """Set up GPU detection, rendering, and display (Xvfb if needed)."""
    gpu_available = has_nvidia_gpu()

    if gpu_available:
        log.info("NVIDIA GPU detected — GPU-accelerated rendering")
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
        nvidia_icd = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.isfile(nvidia_icd):
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_icd
            log.info("Forcing NVIDIA EGL vendor ICD")
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
        log.info("Using native display %s (low-latency host mode)", os.environ["DISPLAY"])
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
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(1)
        if xvfb.poll() is not None:
            log.error("Xvfb failed to start — camera rendering requires a display")
            sys.exit(1)
        os.environ["DISPLAY"] = ":99"


# ── Main ──────────────────────────────────────────────────────────────────────


def parse_args():
    epilog_lines = ["available worlds:"]
    for name, entry in sorted(WORLD_MAP.items()):
        epilog_lines.append(f"  {name:20s}  gazebo: {entry['gz_world']}  simulink: {entry['sim_world']}")

    parser = argparse.ArgumentParser(
        description="Unified Betaflight SITL + Gazebo simulation launcher",
        epilog="\n".join(epilog_lines),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # ── Simulation Settings ──────────────────────────────────────────────
    sim = parser.add_argument_group("Simulation settings")
    sim.add_argument(
        "--world",
        default=DEFAULT_WORLD,
        choices=list(WORLD_MAP.keys()),
        help=f"World short name (default: {DEFAULT_WORLD})",
    )
    sim.add_argument(
        "--physics",
        choices=["gazebo", "simulink"],
        default="gazebo",
        help="Dynamics backend (default: gazebo)",
    )
    sim.add_argument(
        "--gazebo",
        action="store_true",
        help="Show the Gazebo GUI (default: headless)",
    )
    sim.add_argument(
        "--chase-cam",
        action="store_true",
        help="Also display the chase camera (3rd-person SDL2 window)",
    )
    sim.add_argument(
        "--no-transmitter",
        action="store_true",
        help="Skip starting the MSP virtual radio",
    )
    sim.add_argument(
        "--no-video",
        action="store_true",
        help="Skip the video pipeline (dynamics only, no camera display)",
    )
    sim.add_argument(
        "--no-display",
        action="store_true",
        help="Hide SDL2 preview windows (SHM still active)",
    )
    sim.add_argument(
        "--elf",
        default=BF_ELF,
        help="Path to betaflight_SITL.elf",
    )
    sim.add_argument(
        "--msp-port",
        type=int,
        default=5763,
        help="BF MSP TCP port for OSD telemetry (default: 5763 = UART3)",
    )
    sim.add_argument(
        "--fpv-topic",
        default=None,
        help="Explicit Gazebo FPV image topic (skip auto-discovery)",
    )
    sim.add_argument(
        "--chase-topic",
        default=None,
        help="Explicit Gazebo chase image topic (skip auto-discovery)",
    )
    sim.add_argument(
        "--topic-model-hint",
        default=TOPIC_MODEL_HINT_DEFAULT,
        help="Prefer topics containing this model path segment",
    )
    sim.add_argument(
        "--sim-lib",
        default=SIMULINK_LIB,
        help="Path to libinterface_simulink.so",
    )
    sim.add_argument(
        "--bridge",
        default=BF_SIM_BRIDGE,
        help="Path to bf_sim_bridge executable",
    )

    # ── Drone Settings ───────────────────────────────────────────────────
    drn = parser.add_argument_group("Drone settings")
    drn.add_argument(
        "--drone",
        choices=list(DRONE_REFS.keys()),
        default=DEFAULT_DRONE,
        help=f"Drone profile for parameter scaling (default: {DEFAULT_DRONE})",
    )
    drn.add_argument(
        "--cam-pitch",
        type=float,
        default=-80.0,
        help="FPV camera pitch in degrees (default: -80, i.e. 10° from +Z)",
    )
    drn.add_argument(
        "--ctw",
        type=float,
        default=None,
        help="Thrust-to-weight ratio — derives mass & inertia (default: per-drone ref)",
    )
    drn.add_argument(
        "--standoff-height",
        type=float,
        default=None,
        help="Landing leg length in metres (default: per-drone ref)",
    )
    drn.add_argument("--linear-damping-x", type=float, default=None, help="ViscousDragPlugin linear damping X")
    drn.add_argument("--linear-damping-y", type=float, default=None, help="ViscousDragPlugin linear damping Y")
    drn.add_argument("--linear-damping-z", type=float, default=None, help="ViscousDragPlugin linear damping Z")
    drn.add_argument("--quadratic-damping-x", type=float, default=None, help="ViscousDragPlugin quadratic damping X")
    drn.add_argument("--quadratic-damping-y", type=float, default=None, help="ViscousDragPlugin quadratic damping Y")
    drn.add_argument("--quadratic-damping-z", type=float, default=None, help="ViscousDragPlugin quadratic damping Z")
    drn.add_argument("--angular-damping", type=float, default=None, help="ViscousDragPlugin angular damping")

    # ── World Settings ───────────────────────────────────────────────────
    wld = parser.add_argument_group("World settings (collision_test / park_chase)")
    wld.add_argument(
        "--target-altitude",
        type=float,
        default=None,
        help="Target altitude in metres (collision_test default: 20, park_chase default: 50)",
    )
    wld.add_argument(
        "--target-distance-x",
        type=float,
        default=None,
        help="Collision-test target X distance in metres (default: 30)",
    )
    wld.add_argument(
        "--target-distance-y",
        type=float,
        default=None,
        help="Collision-test target Y distance in metres (default: 0)",
    )
    wld.add_argument(
        "--target-speed",
        type=float,
        default=None,
        help="Park-chase target tangential speed in km/h (default: 5.4)",
    )
    wld.add_argument(
        "--target-orbit-radius",
        type=float,
        default=None,
        help="Park-chase orbit radius in metres (default: 30)",
    )
    wld.add_argument(
        "--hit-box-scale",
        type=float,
        default=None,
        help="Uniform scale multiplier for the target hit box (default: 1.0)",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    cleanup_before_start()

    pm = ProcessManager()

    def on_signal(sig, frame):
        pm.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    is_simulink = args.physics == "simulink"

    # ── 1. Environment & display ──
    log.info("Setting up Gazebo environment")
    setup_gazebo_env()
    configure_display(args, pm)

    # ── 1b. Render Jinja2 templates before Gazebo launch ──
    drone = args.drone
    _render_all_templates(drone, args.world, args)

    # Use drone name as topic hint unless user explicitly overrode it.
    if args.topic_model_hint == TOPIC_MODEL_HINT_DEFAULT and drone != TOPIC_MODEL_HINT_DEFAULT:
        args.topic_model_hint = drone

    ref = DRONE_REFS[drone]

    # ── 2. Gazebo ──
    world_entry = WORLD_MAP[args.world]
    world_file = world_entry["sim_world"] if is_simulink else world_entry["gz_world"]
    gz_world_name = world_entry["gz_name"]
    world_path = os.path.join(AEROLOOP_HOME, "worlds", world_file)
    if not os.path.isfile(world_path):
        log.error("World file not found: %s", world_path)
        sys.exit(1)

    gz_args = ["gz", "sim"]
    if not args.gazebo:
        gz_args.append("-s")
    gz_args.extend(["-r", "-v", "3", world_path])

    log.info(
        "Starting Gazebo%s: %s%s",
        " (GUI)" if args.gazebo else " (headless)",
        os.path.basename(world_path),
        " (vis-only)" if is_simulink else "",
    )
    pm.spawn(gz_args)
    time.sleep(8)

    # ── 3. bf_sim_bridge (Simulink dynamics) ──
    bf_bridge_proc = None
    if is_simulink:
        if not os.path.isfile(args.bridge):
            log.error("bf_sim_bridge not found: %s", args.bridge)
            pm.shutdown()
            sys.exit(1)
        if not os.path.isfile(args.sim_lib):
            log.error("libinterface_simulink.so not found: %s", args.sim_lib)
            pm.shutdown()
            sys.exit(1)

        bridge_args = [args.bridge, "--sim-lib", os.path.abspath(args.sim_lib)]
        log.info("Starting bf_sim_bridge (Simulink dynamics)")
        bf_bridge_proc = pm.spawn(bridge_args)
        time.sleep(2)
        if bf_bridge_proc.poll() is not None:
            log.error("bf_sim_bridge exited immediately (code %d)", bf_bridge_proc.returncode)
            pm.shutdown()
            sys.exit(1)

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

    # MSP Virtual Radio
    if not args.no_transmitter:
        radio_index = os.path.join(MSP_RADIO_HOME, "index.js")
        if os.path.isfile(radio_index):
            log.info("Starting MSP Virtual Radio")
            pm.spawn(["node", radio_index])
        else:
            log.warning("MSP Virtual Radio not found at %s — skipping", radio_index)
    time.sleep(2)

    # ── 5. Video pipeline ──
    topic = None
    chase_topic = None
    bridge_proc = None
    chase_bridge_proc = None
    width = height = 0

    if args.no_video:
        log.info("Video pipeline disabled (--no-video)")
    else:
        if not os.path.isfile(IMAGE_BRIDGE):
            log.error(
                "gz_image_bridge not found at %s — run build_plugin.sh",
                IMAGE_BRIDGE,
            )
            pm.shutdown()
            sys.exit(1)

        # FPV camera topic
        if args.fpv_topic:
            topic = args.fpv_topic
            log.info("Using explicit FPV topic: %s", topic)
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
                log.error("Could not find FPV camera topic")
                pm.shutdown()
                sys.exit(1)
            log.info("Found FPV camera topic: %s", topic)

        # Chase camera topic (optional)
        if args.chase_cam:
            if args.chase_topic:
                chase_topic = args.chase_topic
                log.info("Using explicit chase topic: %s", chase_topic)
            else:
                log.info("Discovering chase camera topic …")
                chase_candidates = list_camera_topics(name_hint="chase_cam")
                if chase_candidates:
                    log.info("Chase candidates: %s", ", ".join(chase_candidates))
                chase_topic = discover_camera_topic(
                    name_hint="chase_cam",
                    timeout=30,
                    model_hint=args.topic_model_hint,
                )
                if not chase_topic:
                    log.warning("Chase camera topic not found — FPV only")
                else:
                    log.info("Found chase camera topic: %s", chase_topic)

        # Start FPV bridge (OSD always on)
        bridge_cmd = [
            IMAGE_BRIDGE, topic, "--display",
            "--osd", "--msp-port", str(args.msp_port),
            "--cam-pitch", str(args.cam_pitch),
        ]
        if args.no_display:
            bridge_cmd.append("--hidden")

        # Per-world target proximity detection
        target_model = world_entry.get("target_model")
        target_bbox  = world_entry.get("target_bbox")
        if target_model:
            bridge_cmd.extend(["--target-model", target_model])
            if target_bbox:
                bridge_cmd.extend(["--target-bbox", target_bbox])
            if args.hit_box_scale is not None:
                bridge_cmd.extend(["--hit-box-scale", str(args.hit_box_scale)])
            log.info("Target proximity: model='%s' bbox=%s scale=%s",
                     target_model, target_bbox or "default",
                     args.hit_box_scale if args.hit_box_scale is not None else "1.0")

        log.info("OSD overlay enabled (MSP port %d)", args.msp_port)

        bridge_proc = pm.spawn(
            bridge_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        flags = fcntl.fcntl(bridge_proc.stderr, fcntl.F_GETFL)
        fcntl.fcntl(bridge_proc.stderr, fcntl.F_SETFL, flags | os.O_NONBLOCK)

        log.info("Waiting for first camera frame …")
        width, height, pix_fmt = read_image_meta(bridge_proc, timeout=30)
        if width is None:
            log.error("No image metadata from bridge — camera may not be rendering")
            try:
                remaining_stderr = bridge_proc.stderr.read(2048)
                if remaining_stderr:
                    log.error(
                        "Bridge stderr: %s",
                        remaining_stderr.decode("utf-8", errors="replace"),
                    )
            except Exception:
                pass
            pm.shutdown()
            sys.exit(1)
        log.info("Camera: %dx%d %s", width, height, pix_fmt)

        # Chase camera (optional, always --display for rendering)
        if chase_topic:
            log.info("Starting chase camera bridge (no OSD)")
            chase_cmd = [IMAGE_BRIDGE, chase_topic, "--display", "--no-osd"]
            if args.no_display:
                chase_cmd.append("--hidden")
            chase_bridge_proc = pm.spawn(
                chase_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    # ── 6. Print connection info ──
    _print_status(
        args, is_simulink, topic, chase_topic,
        width, height, bridge_proc, chase_bridge_proc,
    )

    # ── 7. Keep alive ──
    try:
        while True:
            if bf_bridge_proc and bf_bridge_proc.poll() is not None:
                log.error("bf_sim_bridge exited (code %d) — stopping", bf_bridge_proc.returncode)
                break
            if bridge_proc and bridge_proc.poll() is not None:
                log.warning("Image bridge exited (code %d)", bridge_proc.returncode)
                break
            if chase_bridge_proc and chase_bridge_proc.poll() is not None:
                log.warning("Chase camera bridge exited (code %d)", chase_bridge_proc.returncode)
                chase_bridge_proc = None
            time.sleep(2)
    except KeyboardInterrupt:
        pass

    pm.shutdown()
    log.info("Simulation stopped")


def _print_status(args, is_simulink, topic, chase_topic,
                  width, height, bridge_proc, chase_bridge_proc):
    """Print active connection info."""
    world_name = args.world

    print()
    print("=" * 64)
    if is_simulink:
        print("  Simulink Dynamics Backend — Running")
    else:
        print("  FPV Simulation — Running")
    print("=" * 64)
    print()

    if is_simulink:
        print(f"  Backend      : bf_sim_bridge → Simulink → BF SITL (250 Hz)")
        print(f"  World        : {world_name} (vis-only)")
        print(f"  Simulink .so : {os.path.basename(args.sim_lib)}")
    else:
        print(f"  World        : {world_name}")

    if not args.no_video:
        print(f"  FPV window   : SDL2 (gz_image_bridge, OSD always on)")
        if chase_bridge_proc:
            print(f"  Chase window : SDL2 (gz_image_bridge)")

        if topic:
            print(f"  FPV topic    : {topic}")
        if chase_topic:
            print(f"  Chase topic  : {chase_topic}")
        if width and height:
            print(f"  Resolution   : {width}x{height}")
        print(f"  Shared mem   : /dev/shm (always active, clean + OSD)")
    else:
        print(f"  Video        : disabled (--no-video)")

    print(f"  RC input     : UDP 127.0.0.1:9004  (elrs_udp_bridge.py)")
    print(f"  BF CLI       : TCP 127.0.0.1:5761")
    print(f"  BF Config    : TCP 127.0.0.1:5760")

    print()
    print("  Press Ctrl-C to stop")
    print("=" * 64)
    print()


if __name__ == "__main__":
    main()
