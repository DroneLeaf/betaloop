#!/usr/bin/env python3
"""PX4 SITL simulation launcher — px4_sim_bridge + Gazebo Harmonic.

Orchestrates the host-side components of the PX4 simulation stack:
  1. Gazebo Harmonic — visualization only (ExternalPosePlugin)
  2. px4_sim_bridge — Simulink dynamics ↔ PX4 MAVLink HIL bridge
  3. Video pipeline — FPV camera + optional chase camera

PX4 and LeafFC run inside the droneleaf-sitl container and connect
to px4_sim_bridge on the host via TCP port 4560.

Usage:
    # Minimal — start bridge + Gazebo, PX4 connects from container
    python3 start_px4.py --world park_chase --gazebo

    # With video pipeline
    python3 start_px4.py --world balloon_test --gazebo --chase-cam

    # Custom Simulink model and parameters
    python3 start_px4.py --sim-lib path/to/libinterface_simulink.so \\
                         --params path/to/params.txt

    # Override GPS reference (default: Zurich 47.397742, 8.545594, 488m)
    python3 start_px4.py --lat 24.45 --lon 54.65 --alt 10
"""

import argparse
import logging
import os
import signal
import subprocess
import sys
import threading
import time

from common import (
    AEROLOOP_HOME,
    DEFAULT_DRONE,
    DRONE_REFS,
    SIMULINK_LIB,
    TOPIC_MODEL_HINT_DEFAULT,
    ProcessManager,
    cleanup_before_start,
    compute_model_vars,
    compute_world_vars,
    configure_display,
    default_path,
    render_vis_templates,
    setup_gazebo_env,
    start_balloon_thread,
    start_chase_bridge,
    start_fpv_bridge,
    start_orbit_thread,
    start_patrol_thread,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("px4-sim")


# ── PX4-specific defaults ─────────────────────────────────────────────────────

PX4_SIM_BRIDGE = default_path(
    "PX4_SIM_BRIDGE",
    os.path.join("px4_sim_bridge", "build", "px4_sim_bridge"),
)

DEFAULT_WORLD = "balloon_test"

WORLD_MAP = {
    "park_chase": {
        "sim_world": "rocket_drone_park_chase_vis.sdf",
        "gz_name":   "fpv_chase_park",
        "target_model": "moving_target_drone",
        "orbit_drive":  True,
    },
    "patrol_park": {
        "sim_world": "rocket_drone_patrol_park_vis.sdf",
        "gz_name":   "fpv_patrol_park",
        "target_model": "patrol_target_drone",
        "patrol_joint": "patrol_joint",
    },
    "collision_test": {
        "sim_world": "rocket_drone_collision_test_vis.sdf",
        "gz_name":   "collision_test",
    },
    "balloon_test": {
        "sim_world": "rocket_drone_balloon_test_vis.sdf",
        "gz_name":   "balloon_test",
        "target_model": "balloon_target",
        "balloon_wind": True,
    },
}


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="PX4 SITL + Simulink dynamics + Gazebo visualization launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    sim = parser.add_argument_group("Simulation settings")
    sim.add_argument("--world", default=DEFAULT_WORLD,
                     choices=list(WORLD_MAP.keys()),
                     help=f"World short name (default: {DEFAULT_WORLD})")
    sim.add_argument("--drone", default=DEFAULT_DRONE,
                     choices=list(DRONE_REFS.keys()),
                     help=f"Drone profile (default: {DEFAULT_DRONE})")
    sim.add_argument("--cam-pitch", type=float, default=-80.0,
                     help="FPV camera pitch in degrees (default: -80)")
    sim.add_argument("--tracker-cam-pitch", type=float, default=-80.0,
                     help="Tracker camera pitch in degrees (default: -80)")
    sim.add_argument("--tracker-cam-roll", type=float, default=0.0,
                     help="Tracker camera roll in degrees (default: 0)")
    sim.add_argument("--fpv-hfov", type=float, default=114.6,
                     help="FPV camera horizontal FOV in degrees (default: 114.6)")
    sim.add_argument("--fpv-vfov", type=float, default=98.9,
                     help="FPV camera vertical FOV in degrees (default: 98.9)")
    sim.add_argument("--tracker-hfov", type=float, default=114.6,
                     help="Tracker camera horizontal FOV in degrees (default: 114.6)")
    sim.add_argument("--tracker-vfov", type=float, default=98.9,
                     help="Tracker camera vertical FOV in degrees (default: 98.9)")
    sim.add_argument("--fpv-cam-width", type=float, default=640,
                     help="Pilot camera output width in pixels (default: 640)")
    sim.add_argument("--fpv-cam-height", type=float, default=480,
                     help="Pilot camera output height in pixels (default: 480)")
    sim.add_argument("--tracker-cam-width", type=float, default=640,
                     help="Tracker camera source width in pixels (default: 640)")
    sim.add_argument("--tracker-cam-height", type=float, default=480,
                     help="Tracker camera output height in pixels (default: 480)")
    # Backward compatibility: applies to both cameras if explicitly provided.
    sim.add_argument("--cam-width", type=float, default=None,
                     help="Deprecated: output width for both pilot/tracker cameras")
    sim.add_argument("--cam-height", type=float, default=None,
                     help="Deprecated: output height for both pilot/tracker cameras")
    sim.add_argument("--gazebo", action="store_true",
                     help="Show the Gazebo GUI (default: headless)")
    sim.add_argument("--chase-cam", action="store_true",
                     help="Display chase camera (3rd-person SDL2 window)")
    sim.add_argument("--no-video", action="store_true",
                     help="Skip the video pipeline")
    sim.add_argument("--no-display", action="store_true",
                     help="Hide SDL2 preview windows (SHM still active)")
    sim.add_argument("--topic-model-hint", default=TOPIC_MODEL_HINT_DEFAULT,
                     help="Prefer topics containing this model path segment")
    sim.add_argument("--fpv-topic", default=None,
                     help="Explicit Gazebo FPV image topic")
    sim.add_argument("--chase-topic", default=None,
                     help="Explicit Gazebo chase image topic")

    brg = parser.add_argument_group("Bridge settings")
    brg.add_argument("--bridge", default=PX4_SIM_BRIDGE,
                     help="Path to px4_sim_bridge executable")
    brg.add_argument("--sim-lib", default=SIMULINK_LIB,
                     help="Path to libinterface_simulink.so")
    brg.add_argument("--params", default=None,
                     help="Path to key=value parameter overrides file")
    brg.add_argument("--tcp-port", type=int, default=4560,
                     help="TCP port for PX4 simulator connection (default: 4560)")
    brg.add_argument("--telem-port", type=int, default=0,
                     help="UDP port for raw telemetry output (0=off)")
    brg.add_argument("--mavlink-port", type=int, default=14560,
                     help="MAVLink UDP port for OSD telemetry (default: 14560)")

    gps = parser.add_argument_group("GPS reference")
    gps.add_argument("--lat", type=float, default=47.397742,
                     help="GPS reference latitude (default: 47.397742 Zurich)")
    gps.add_argument("--lon", type=float, default=8.545594,
                     help="GPS reference longitude (default: 8.545594)")
    gps.add_argument("--alt", type=float, default=488.0,
                     help="GPS reference altitude AMSL in metres (default: 488.0)")

    tgt = parser.add_argument_group("Target trajectory")
    tgt.add_argument("--target-altitude", type=float, default=None,
                     help="Target altitude in metres (park_chase: 50, patrol_park: 100, balloon: 10)")
    tgt.add_argument("--target-distance-x", type=float, default=None,
                     help="Target initial X position in metres (default: 30)")
    tgt.add_argument("--target-distance-y", type=float, default=None,
                     help="Target initial Y position in metres (default: 0)")
    tgt.add_argument("--target-speed", type=float, default=None,
                     help="Target speed km/h (park_chase orbit: 5.4, patrol_park: 20)")
    tgt.add_argument("--target-orbit-radius", type=float, default=None,
                     help="Park-chase orbit radius in metres (default: 30)")
    tgt.add_argument("--patrol-length", type=float, default=None,
                     help="Patrol total distance in metres (default: 500)")
    tgt.add_argument("--target-launch-offset", type=float, default=None,
                     help="Patrol launch offset behind player in metres (default: 50)")
    tgt.add_argument("--sine-amplitude-xy", type=float, default=None,
                     help="Patrol lateral sine amplitude in metres (default: 0)")
    tgt.add_argument("--sine-period-xy", type=float, default=None,
                     help="Patrol lateral sine period in metres (default: 200)")
    tgt.add_argument("--sine-amplitude-z", type=float, default=None,
                     help="Patrol vertical sine amplitude in metres (default: 0)")
    tgt.add_argument("--sine-period-z", type=float, default=None,
                     help="Patrol vertical sine period in metres (default: 200)")
    tgt.add_argument("--wind-intensity", type=float, default=2.0,
                     help="Balloon lateral drift amplitude in metres (default: 2.0)")
    tgt.add_argument("--wind-randomness", type=float, default=1.0,
                     help="Balloon vertical bobbing amplitude in metres (default: 1.0)")
    tgt.add_argument("--drift-speed", type=float, default=20.0,
                     help="Balloon Lissajous frequency multiplier (default: 20.0)")

    return parser.parse_args()


def main():
    args = parse_args()

    if args.cam_width is not None:
        args.fpv_cam_width = args.cam_width
        args.tracker_cam_width = args.cam_width
    if args.cam_height is not None:
        args.fpv_cam_height = args.cam_height
        args.tracker_cam_height = args.cam_height

    _px4_kill = ["pkill -9 -x px4_sim_bridge 2>/dev/null || true"]
    cleanup_before_start(extra_pkill_cmds=_px4_kill)
    pm = ProcessManager()

    def on_signal(sig, frame):
        pm.shutdown(extra_pkill_patterns=["px4_sim_bridge"])
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # ── 1. Environment & display ──
    log.info("Setting up Gazebo environment")
    setup_gazebo_env()
    configure_display(args, pm)

    # ── 1b. Render Jinja2 vis templates before Gazebo launch ──
    model_vars = compute_model_vars(
        args.drone, cam_pitch=args.cam_pitch,
        tracker_cam_pitch=args.tracker_cam_pitch,
        tracker_cam_roll=args.tracker_cam_roll,
        fpv_hfov_deg=args.fpv_hfov,
        fpv_vfov_deg=args.fpv_vfov,
        tracker_hfov_deg=args.tracker_hfov,
        tracker_vfov_deg=args.tracker_vfov,
        fpv_cam_width=args.fpv_cam_width,
        tracker_cam_width=args.tracker_cam_width,
    )
    world_vars = compute_world_vars(
        args.drone, args.world,
        target_altitude=args.target_altitude,
        target_speed=args.target_speed,
        orbit_radius=args.target_orbit_radius,
        patrol_length=args.patrol_length,
        target_x=args.target_distance_x,
        target_y=args.target_distance_y,
    )
    render_vis_templates(args.drone, args.world, WORLD_MAP, model_vars, world_vars)

    # Use drone name as topic hint unless user explicitly overrode it.
    if args.topic_model_hint == TOPIC_MODEL_HINT_DEFAULT and args.drone != TOPIC_MODEL_HINT_DEFAULT:
        args.topic_model_hint = args.drone

    # ── 2. Gazebo (vis-only) ──
    world_entry = WORLD_MAP[args.world]
    world_file = world_entry["sim_world"]
    world_path = os.path.join(AEROLOOP_HOME, "worlds", world_file)
    if not os.path.isfile(world_path):
        log.error("World file not found: %s", world_path)
        sys.exit(1)

    gz_args = ["gz", "sim"]
    if not args.gazebo:
        gz_args.append("-s")
    gz_args.extend(["-r", "-v", "3", world_path])

    log.info("Starting Gazebo%s: %s (vis-only)",
             " (GUI)" if args.gazebo else " (headless)",
             os.path.basename(world_path))
    pm.spawn(gz_args)
    time.sleep(8)

    # ── 3. px4_sim_bridge ──
    if not os.path.isfile(args.bridge):
        log.error("px4_sim_bridge not found: %s (run: cd px4_sim_bridge/build && cmake .. && make)",
                  args.bridge)
        pm.shutdown()
        sys.exit(1)
    if not os.path.isfile(args.sim_lib):
        log.error("Simulink library not found: %s", args.sim_lib)
        pm.shutdown()
        sys.exit(1)

    bridge_args = [
        args.bridge,
        "--sim-lib", os.path.abspath(args.sim_lib),
        "--port", str(args.tcp_port),
        "--lat", str(args.lat),
        "--lon", str(args.lon),
        "--alt", str(args.alt),
    ]
    if args.params:
        bridge_args += ["--params", os.path.abspath(args.params)]
    if args.telem_port:
        bridge_args += ["--telem-port", str(args.telem_port)]

    log.info("Starting px4_sim_bridge (Simulink → PX4 HIL)")
    bridge_proc = pm.spawn(bridge_args)
    time.sleep(2)
    if bridge_proc.poll() is not None:
        log.error("px4_sim_bridge exited immediately (code %d)", bridge_proc.returncode)
        pm.shutdown()
        sys.exit(1)

    # ── 4. Video pipeline (optional) ──
    osd_args = [
        "--mavlink-osd",
        "--mavlink-port", str(args.mavlink_port),
        "--cam-pitch", str(args.cam_pitch),
        "--out-width", str(args.fpv_cam_width),
        "--out-height", str(args.fpv_cam_height),
    ]
    # Per-world target proximity detection
    target_model = world_entry.get("target_model")
    target_link  = world_entry.get("target_link")
    target_bbox  = world_entry.get("target_bbox")
    if target_model:
        osd_args.extend(["--target-model", target_model])
        if target_link:
            osd_args.extend(["--target-link", target_link])
        if target_bbox:
            osd_args.extend(["--target-bbox", target_bbox])
        log.info("Target proximity: model='%s' link=%s bbox=%s",
                 target_model, target_link or "(model root)",
                 target_bbox or "default")
    log.info("MAVLink OSD enabled (UDP port %d)", args.mavlink_port)
    fpv_bridge_proc, width, height = start_fpv_bridge(args, pm, osd_args=osd_args)
    chase_bridge_proc = start_chase_bridge(args, pm)

    # ── 5. Target trajectory threads ──
    traj_stop = threading.Event()

    if world_entry.get("orbit_drive") and world_entry.get("target_model"):
        orbit_radius = args.target_orbit_radius if args.target_orbit_radius is not None else 30.0
        speed_kmh = args.target_speed if args.target_speed is not None else 5.4
        orbit_omega = (speed_kmh / 3.6) / orbit_radius
        target_z = args.target_altitude if args.target_altitude is not None else 50.0
        start_orbit_thread(traj_stop, orbit_radius, orbit_omega, target_z)

    elif world_entry.get("patrol_joint") and world_entry.get("target_model"):
        patrol_length = args.patrol_length if args.patrol_length is not None else 500.0
        launch_offset = args.target_launch_offset if args.target_launch_offset is not None else 50.0
        speed_kmh = args.target_speed if args.target_speed is not None else 20.0
        speed_ms = speed_kmh / 3.6
        target_z = args.target_altitude if args.target_altitude is not None else 100.0
        start_patrol_thread(
            traj_stop,
            patrol_length=patrol_length,
            launch_offset=launch_offset,
            speed_ms=speed_ms,
            target_z=target_z,
            sine_amp_xy=args.sine_amplitude_xy or 0.0,
            sine_period_xy=args.sine_period_xy or 200.0,
            sine_amp_z=args.sine_amplitude_z or 0.0,
            sine_period_z=args.sine_period_z or 200.0,
        )

    elif world_entry.get("balloon_wind") and world_entry.get("target_model"):
        target_x = args.target_distance_x if args.target_distance_x is not None else 30.0
        target_y = args.target_distance_y if args.target_distance_y is not None else 0.0
        target_z = args.target_altitude if args.target_altitude is not None else 10.0
        start_balloon_thread(
            traj_stop,
            mean_x=target_x,
            mean_y=target_y,
            mean_z=target_z,
            wind_intensity=args.wind_intensity,
            wind_randomness=args.wind_randomness,
            drift_speed=args.drift_speed,
        )

    # ── 6. Print status ──
    print()
    print("=" * 64)
    print("  PX4 Simulink SITL — Running")
    print("=" * 64)
    print()
    print(f"  Backend      : px4_sim_bridge → Simulink → PX4 HIL (250 Hz)")
    print(f"  World        : {args.world} (vis-only)")
    print(f"  Simulink .so : {os.path.basename(args.sim_lib)}")
    print(f"  TCP port     : {args.tcp_port} (PX4 connects here)")
    print(f"  GPS ref      : lat={args.lat:.6f} lon={args.lon:.6f} alt={args.alt:.1f}")
    print()
    print(f"  PX4 offboard : UDP 127.0.0.1:14540  (MAVLink)")
    print(f"  PX4 GCS      : UDP 127.0.0.1:18570  (MAVLink / QGC)")
    print(f"  OSD telemetry: UDP 127.0.0.1:{args.mavlink_port}  (MAVLink → gz_image_bridge)")
    print(f"  Simulator    : TCP 0.0.0.0:{args.tcp_port}   (HIL lockstep)")
    print(f"  Reset        : UDP 127.0.0.1:9011   (reset_world.py)")
    if not args.no_video and width:
        print(f"  FPV camera   : {width}x{height}")
    print()
    print("  PX4 SITL must connect to this host on TCP port %d." % args.tcp_port)
    print("  From droneleaf-sitl container, set PX4_SIM_HOSTNAME=<host-ip>")
    print()
    print("  Press Ctrl-C to stop")
    print("=" * 64)
    print()

    # ── 6. Keep alive ──
    try:
        while True:
            if bridge_proc.poll() is not None:
                log.error("px4_sim_bridge exited (code %d) — stopping",
                          bridge_proc.returncode)
                break
            if fpv_bridge_proc and fpv_bridge_proc.poll() is not None:
                log.warning("Image bridge exited (code %d)",
                            fpv_bridge_proc.returncode)
                break
            if chase_bridge_proc and chase_bridge_proc.poll() is not None:
                log.warning("Chase camera bridge exited")
                chase_bridge_proc = None
            time.sleep(2)
    except KeyboardInterrupt:
        pass

    pm.shutdown(extra_pkill_patterns=["px4_sim_bridge"])


if __name__ == "__main__":
    main()
