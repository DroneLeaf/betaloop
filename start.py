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
import random
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

from common import (
    AEROLOOP_HOME,
    DRONE_REFS,
    IMAGE_BRIDGE,
    SIMULINK_LIB,
    TOPIC_MODEL_HINT_DEFAULT,
    ProcessManager,
    cleanup_before_start,
    compute_model_vars,
    compute_world_vars,
    configure_display,
    default_path,
    discover_camera_topic,
    list_camera_topics,
    read_image_meta,
    render_template,
    render_vis_templates,
    setup_gazebo_env,
    start_balloon_thread,
    start_orbit_thread,
    start_patrol_thread,
    wait_for_port,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("betaloop")


# ── Defaults ──────────────────────────────────────────────────────────────────

BF_ELF = default_path(
    "BF_ELF",
    os.path.join("betaflight", "obj", "main", "betaflight_SITL.elf"),
)
MSP_RADIO_HOME = default_path(
    "MSP_RADIO_HOME",
    os.path.join("..", "msp_virtualradio"),
)
BF_SIM_BRIDGE = default_path(
    "BF_SIM_BRIDGE",
    os.path.join("bf_sim_bridge", "build", "bf_sim_bridge"),
)

DEFAULT_WORLD = "park_chase"
DEFAULT_DRONE = "rocket_drone"

# Short world name → dict of world attributes.
#   gz_world     — Gazebo physics world file
#   sim_world    — Simulink vis-only SDF file
#   gz_name      — Gazebo <world name> element
#   target_model — SDF model name of the target (for proximity OSD)
#   target_link  — link inside target model whose world pose to track
#   target_bbox  — half-extents "X,Y,Z" in metres (axis-aligned)
#   patrol_joint — prismatic joint name for patrol reversal (patrol_park only)
WORLD_MAP = {
    "park_chase": {
        "gz_world":     "rocket_drone_park_chase.world",
        "sim_world":    "rocket_drone_park_chase_vis.sdf",
        "gz_name":      "fpv_chase_park",
        "target_model": "moving_target_drone",
        "target_link":  "geranium_link",
        "target_bbox":  "0.792,1.047,0.186",
        "orbit_drive":  True,
    },
    "patrol_park": {
        "gz_world":     "rocket_drone_patrol_park.world",
        "sim_world":    "rocket_drone_patrol_park_vis.sdf",
        "gz_name":      "fpv_patrol_park",
        "target_model": "patrol_target_drone",
        "target_link":  "geranium_link",
        "target_bbox":  "0.792,1.047,0.186",
        "patrol_joint": "patrol_joint",
    },
    "collision_test": {
        "gz_world":     "rocket_drone_collision_test.world",
        "sim_world":    "rocket_drone_collision_test_vis.sdf",
        "gz_name":      "collision_test",
        "target_model": "collision_test_target",
        "target_bbox":  "0.792,1.047,0.186",
    },
    "balloon_test": {
        "gz_world":     "rocket_drone_balloon_test.world",
        "sim_world":    "rocket_drone_balloon_test_vis.sdf",
        "gz_name":      "balloon_test",
        "target_model": "balloon_target",
        "balloon_wind": True,
    },
}

# ── BF-specific Helpers ───────────────────────────────────────────────────────


def _render_all_templates(drone, world_name, args):
    """Render all Jinja2 templates for the selected drone and world."""
    ref = DRONE_REFS[drone]
    models_dir = os.path.join(AEROLOOP_HOME, "models")
    worlds_dir = os.path.join(AEROLOOP_HOME, "worlds")

    # ── Damping overrides (BF-only CLI args) ──
    damping_overrides = {}
    for key, attr in [
        ("linear_x", "linear_damping_x"), ("linear_y", "linear_damping_y"),
        ("linear_z", "linear_damping_z"), ("quadratic_x", "quadratic_damping_x"),
        ("quadratic_y", "quadratic_damping_y"), ("quadratic_z", "quadratic_damping_z"),
        ("angular", "angular_damping"),
    ]:
        val = getattr(args, attr, None)
        if val is not None:
            damping_overrides[key] = val

    # ── Model variables (shared helper) ──
    ctw = args.ctw if args.ctw is not None else None
    standoff = args.standoff_height if args.standoff_height is not None else None
    model_vars = compute_model_vars(
        drone, ctw=ctw, cam_pitch=args.cam_pitch,
        standoff=standoff, damping_overrides=damping_overrides,
        tracker_cam_pitch=args.tracker_cam_pitch,
        tracker_cam_roll=args.tracker_cam_roll,
        fpv_hfov_deg=args.fpv_hfov,
        fpv_vfov_deg=args.fpv_vfov,
        tracker_hfov_deg=args.tracker_hfov,
        tracker_vfov_deg=args.tracker_vfov,
        fpv_cam_width=args.fpv_cam_width,
        tracker_cam_width=args.tracker_cam_width,
    )

    log.info("CTW=%.1f mass=%.3fkg Ixx=%.6f Iyy=%.6f Izz=%.6f standoff=%.3fm cam_pitch=%.1f°",
             ctw or ref["default_ctw"],
             model_vars["mass"], model_vars["ixx"], model_vars["iyy"], model_vars["izz"],
             model_vars["standoff_height"], args.cam_pitch)

    # ── Render physics model SDF (BF only — PX4 doesn't use it) ──
    phys_j2 = os.path.join(models_dir, ref["model_sdf"] + ".j2")
    if os.path.isfile(phys_j2):
        render_template(phys_j2, model_vars)

    # ── World variables (shared helper) ──
    world_vars = compute_world_vars(
        drone, world_name,
        target_altitude=args.target_altitude,
        target_speed=args.target_speed,
        orbit_radius=args.target_orbit_radius,
        patrol_length=args.patrol_length,
        target_x=getattr(args, "target_distance_x", None),
        target_y=getattr(args, "target_distance_y", None),
    )

    # ── Render vis model + vis world (shared helper) ──
    render_vis_templates(drone, world_name, WORLD_MAP, model_vars, world_vars)

    # ── Render physics world (BF only) ──
    wm = WORLD_MAP[world_name]
    gz_j2 = os.path.join(worlds_dir, wm["gz_world"] + ".j2")
    if os.path.isfile(gz_j2):
        render_template(gz_j2, world_vars)

    return world_vars


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
        "--tracker-cam-pitch",
        type=float,
        default=-80.0,
        help="Tracker camera pitch in degrees (default: -80)",
    )
    drn.add_argument(
        "--tracker-cam-roll",
        type=float,
        default=0.0,
        help="Tracker camera roll in degrees (default: 0; use 90 for landscape)",
    )
    drn.add_argument(
        "--fpv-hfov",
        type=float,
        default=114.6,
        help="FPV camera horizontal FOV in degrees (default: 114.6)",
    )
    drn.add_argument(
        "--fpv-vfov",
        type=float,
        default=98.9,
        help="FPV camera vertical FOV in degrees (default: 98.9)",
    )
    drn.add_argument(
        "--tracker-hfov",
        type=float,
        default=114.6,
        help="Tracker camera horizontal FOV in degrees (default: 114.6)",
    )
    drn.add_argument(
        "--tracker-vfov",
        type=float,
        default=98.9,
        help="Tracker camera vertical FOV in degrees (default: 98.9)",
    )
    drn.add_argument("--fpv-cam-width", type=float, default=640,
                     help="Pilot camera output width in pixels (default: 640)")
    drn.add_argument("--fpv-cam-height", type=float, default=480,
                     help="Pilot camera output height in pixels (default: 480)")
    drn.add_argument("--tracker-cam-width", type=float, default=640,
                     help="Tracker camera output width in pixels (default: 640)")
    drn.add_argument("--tracker-cam-height", type=float, default=480,
                     help="Tracker camera output height in pixels (default: 480)")
    # Backward compatibility: applies to both cameras if explicitly provided.
    drn.add_argument("--cam-width", type=float, default=None,
                     help="Deprecated: output width for both pilot/tracker cameras")
    drn.add_argument("--cam-height", type=float, default=None,
                     help="Deprecated: output height for both pilot/tracker cameras")
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

    # ── Simulink Settings ────────────────────────────────────────────────
    slk = parser.add_argument_group("Simulink settings")
    slk.add_argument(
        "--params",
        type=str,
        default=None,
        help="Path to a key=value .params file for bf_sim_bridge model parameters",
    )
    slk.add_argument(
        "--telem-port",
        type=int,
        default=0,
        help="UDP port for bf_sim_bridge to send raw Simulink state to sitl_redis_bridge (0=off)",
    )

    # ── World Settings ───────────────────────────────────────────────────
    wld = parser.add_argument_group("World settings (collision_test / park_chase / patrol_park)")
    wld.add_argument(
        "--target-altitude",
        type=float,
        default=None,
        help="Target altitude in metres (collision_test default: 20, park_chase default: 50, patrol_park default: 100)",
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
        help="Target speed in km/h (park_chase orbit default: 5.4, patrol_park default: 20)",
    )
    wld.add_argument(
        "--target-orbit-radius",
        type=float,
        default=None,
        help="Park-chase orbit radius in metres (default: 30)",
    )
    wld.add_argument(
        "--patrol-length",
        type=float,
        default=None,
        help="Patrol-park total patrol distance in metres (default: 2000)",
    )
    wld.add_argument(
        "--target-launch-offset",
        type=float,
        default=None,
        help="Patrol-park launch offset behind player in metres (default: 50)",
    )
    wld.add_argument(
        "--sine-amplitude-xy",
        type=float,
        default=None,
        help="Patrol lateral sine amplitude in metres (default: 0 = straight line)",
    )
    wld.add_argument(
        "--sine-period-xy",
        type=float,
        default=None,
        help="Patrol lateral sine peak-to-peak distance in metres (default: 200)",
    )
    wld.add_argument(
        "--sine-amplitude-z",
        type=float,
        default=None,
        help="Patrol vertical sine amplitude in metres (default: 0 = flat)",
    )
    wld.add_argument(
        "--sine-period-z",
        type=float,
        default=None,
        help="Patrol vertical sine peak-to-peak distance in metres (default: 200)",
    )
    wld.add_argument(
        "--hit-box-scale",
        type=float,
        default=None,
        help="Uniform scale multiplier for the target hit box (default: 1.0)",
    )
    wld.add_argument(
        "--wind-intensity",
        type=float,
        default=None,
        help="Balloon horizontal drift radius in metres (default: 2.0)",
    )
    wld.add_argument(
        "--wind-randomness",
        type=float,
        default=None,
        help="Balloon vertical bob amplitude in metres (default: 1.0)",
    )
    wld.add_argument(
        "--drift-speed",
        type=float,
        default=None,
        help="Balloon drift speed multiplier (default: 1.0)",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    if args.cam_width is not None:
        args.fpv_cam_width = args.cam_width
        args.tracker_cam_width = args.cam_width
    if args.cam_height is not None:
        args.fpv_cam_height = args.cam_height
        args.tracker_cam_height = args.cam_height

    _bf_kill = [
        "pkill -9 -f 'betaflight_SITL.elf' 2>/dev/null || true",
        "pkill -9 -f 'msp_virtualradio/index.js' 2>/dev/null || true",
        "pkill -9 -x bf_sim_bridge 2>/dev/null || true",
    ]
    cleanup_before_start(extra_pkill_cmds=_bf_kill)

    pm = ProcessManager()

    def on_signal(sig, frame):
        pm.shutdown(extra_pkill_patterns=["betaflight_SITL.elf"])
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
    world_vars = _render_all_templates(drone, args.world, args)

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
        if args.params:
            bridge_args += ["--params", os.path.abspath(args.params)]
        if args.telem_port:
            bridge_args += ["--telem-port", str(args.telem_port)]
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
    tracker_topic = None
    bridge_proc = None
    chase_bridge_proc = None
    tracker_bridge_proc = None
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
            "--out-width", str(args.fpv_cam_width),
            "--out-height", str(args.fpv_cam_height),
        ]
        if args.no_display:
            bridge_cmd.append("--hidden")

        # Per-world target proximity detection
        target_model = world_entry.get("target_model")
        target_link  = world_entry.get("target_link")
        target_bbox  = world_entry.get("target_bbox")
        if target_model:
            bridge_cmd.extend(["--target-model", target_model])
            if target_link:
                bridge_cmd.extend(["--target-link", target_link])
            if target_bbox:
                bridge_cmd.extend(["--target-bbox", target_bbox])
            if args.hit_box_scale is not None:
                bridge_cmd.extend(["--hit-box-scale", str(args.hit_box_scale)])
            log.info("Target proximity: model='%s' link=%s bbox=%s scale=%s",
                     target_model, target_link or "(model root)",
                     target_bbox or "default",
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
            chase_cmd.extend(["--out-width", str(args.fpv_cam_width), "--out-height", str(args.fpv_cam_height)])
            if args.no_display:
                chase_cmd.append("--hidden")
            chase_bridge_proc = pm.spawn(
                chase_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

        # Tracker camera (always launched if video is on, no OSD)
        log.info("Discovering tracker camera topic …")
        tracker_candidates = list_camera_topics(name_hint="fpv_tracker_cam")
        if tracker_candidates:
            log.info("Tracker candidates: %s", ", ".join(tracker_candidates))
        tracker_topic = discover_camera_topic(
            name_hint="fpv_tracker_cam",
            timeout=30,
            model_hint=args.topic_model_hint,
        )
        if not tracker_topic:
            log.warning("Tracker camera topic not found — skipping")
        else:
            log.info("Found tracker camera topic: %s", tracker_topic)
            tracker_cmd = [IMAGE_BRIDGE, tracker_topic, "--display", "--no-osd"]
            tracker_cmd.extend(["--out-width", str(args.tracker_cam_width), "--out-height", str(args.tracker_cam_height)])
            if args.no_display:
                tracker_cmd.append("--hidden")
            tracker_bridge_proc = pm.spawn(
                tracker_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    # ── 6. Print connection info ──
    _print_status(
        args, is_simulink, topic, chase_topic,
        width, height, bridge_proc, chase_bridge_proc,
    )

    # ── 6b. Patrol UDP drive thread (patrol_park only) ──
    patrol_thread = None
    patrol_stop = threading.Event()
    patrol_joint = world_entry.get("patrol_joint")
    if patrol_joint and world_entry.get("target_model"):
        patrol_length = args.patrol_length if args.patrol_length is not None else 2000.0
        launch_offset = args.target_launch_offset if args.target_launch_offset is not None else 50.0
        speed_kmh_p = args.target_speed if args.target_speed is not None else 100.0
        speed_ms = speed_kmh_p / 3.6
        target_z = world_vars["target_z"]
        patrol_thread = start_patrol_thread(
            patrol_stop,
            patrol_length=patrol_length,
            launch_offset=launch_offset,
            speed_ms=speed_ms,
            target_z=target_z,
            sine_amp_xy=args.sine_amplitude_xy or 0.0,
            sine_period_xy=args.sine_period_xy or 200.0,
            sine_amp_z=args.sine_amplitude_z or 0.0,
            sine_period_z=args.sine_period_z or 200.0,
        )

    # ── 6b2. Orbit UDP drive thread (park_chase only) ──
    orbit_thread = None
    orbit_stop = threading.Event()
    if world_entry.get("orbit_drive") and world_entry.get("target_model"):
        orbit_radius_val = world_vars["orbit_radius"]
        orbit_omega = world_vars["orbit_speed"]  # rad/s = v_tangential / R
        target_z = world_vars["target_z"]
        orbit_thread = start_orbit_thread(orbit_stop, orbit_radius_val, orbit_omega, target_z)

    # ── 6c. Balloon smooth drift thread (balloon_test only) ──
    wind_thread = None
    wind_stop = threading.Event()
    if world_entry.get("balloon_wind"):
        wind_intensity = args.wind_intensity if args.wind_intensity is not None else 2.0
        wind_sigma = args.wind_randomness if args.wind_randomness is not None else 1.0
        drift_speed = args.drift_speed if args.drift_speed is not None else 20.0
        mean_x, mean_y, mean_z = world_vars["target_x"], world_vars["target_y"], world_vars["target_z"]

        wind_thread = start_balloon_thread(
            wind_stop,
            mean_x=mean_x,
            mean_y=mean_y,
            mean_z=mean_z,
            wind_intensity=wind_intensity,
            wind_randomness=wind_sigma,
            drift_speed=drift_speed,
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

    if patrol_thread:
        patrol_stop.set()
        patrol_thread.join(timeout=2)
    if orbit_thread:
        orbit_stop.set()
        orbit_thread.join(timeout=2)
    if wind_thread:
        wind_stop.set()
        wind_thread.join(timeout=2)

    pm.shutdown(extra_pkill_patterns=["betaflight_SITL.elf"])


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
