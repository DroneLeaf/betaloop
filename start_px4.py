#!/usr/bin/env python3
"""PX4 SITL simulation launcher — px4_sim_bridge + Gazebo Harmonic.

Orchestrates the host-side components of the PX4 simulation stack:
  1. Gazebo Harmonic — visualization only (ExternalPosePlugin)
  2. px4_sim_bridge — Simulink dynamics ↔ PX4 MAVLink HIL bridge
  3. Video pipeline — FPV camera + optional chase camera

PX4 and LeafFC run inside the droneleaf-sitl-noetic container and connect
to px4_sim_bridge on the host via TCP port 4560.

Usage:
    # Minimal — start bridge + Gazebo, PX4 connects from container
    python3 start_px4.py --world moving_target --gazebo

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
    DEFAULT_TARGET_DRONE,
    DEFAULT_TARGET_MESH_COLOR,
    DRONE_REFS,
    SIMULINK_LIB,
    TARGET_MESH_COLORS,
    TARGET_REFS,
    TOPIC_MODEL_HINT_DEFAULT,
    ProcessManager,
    cleanup_before_start,
    add_lens_args,
    compute_model_vars,
    compute_world_vars,
    configure_display,
    lens_kwargs_from_args,
    default_path,
    render_vis_templates,
    setup_gazebo_env,
    start_balloon_thread,
    start_static_target_thread,
    start_chase_bridge,
    start_fpv_bridge,
    start_trajectory_thread,
    TERRAIN_THEMES,
    TRAJ_TYPES,
    start_tracker_bridges,
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
    "moving_target": {
        "sim_world": "rocket_drone_moving_target_vis.sdf",
        "gz_name":   "fpv_moving_target",
        "target_model": "moving_target",
        "target_link":  "geranium_link",
        "target_drone": True,
        "trajectory_drive": True,
    },
    "collision_test": {
        "sim_world": "rocket_drone_collision_test_vis.sdf",
        "gz_name":   "collision_test",
        "target_model": "collision_test_target",
        "target_drone": True,
        "static_target": True,
    },
    "windy_target": {
        "sim_world": "rocket_drone_windy_target_vis.sdf",
        "gz_name":   "windy_target",
        "target_model": "windy_target",
        # Selectable target (balloon/shahed/stingjet), defaults to balloon. Its
        # pose is driven by the wind (Lissajous drift / harmonic) thread on 9014.
        "target_drone":   True,
        "default_target": "balloon",
        "balloon_wind":   True,
    },
    "shake_test": {
        "sim_world": "rocket_drone_shake_test_vis.sdf",
        "gz_name":   "shake_test",
        # The kinematic IMU shake is BF-only (it lives in bf_sim_bridge); on the
        # PX4 stack this world renders the drone static with a static balloon.
        "target_model":  "shake_target",
        "target_drone":  True,
        "force_target":  "balloon",
        "static_target": True,
    },
    "pilot_controlled_target": {
        "sim_world": "rocket_drone_pilot_target_vis.sdf",
        "gz_name":   "pilot_target",
        "target_model": "pilot_target",
        "target_link":  "geranium_link",
        "target_drone": True,
        # The target is manually steered by the standalone pilot_target.py tool
        # (WASD / USB joystick -> UDP 9016, reset 9017, GT mirror 9018). The
        # launcher starts NO drive thread for it.
        "pilot_drive": True,
    },
}

# Backward-compat world short-name aliases (old name → canonical).
WORLD_ALIASES = {
    "balloon_test": "windy_target",
}


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="PX4 SITL + Simulink dynamics + Gazebo visualization launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    sim = parser.add_argument_group("Simulation settings")
    sim.add_argument("--world", default=DEFAULT_WORLD,
                     choices=list(WORLD_MAP.keys()) + list(WORLD_ALIASES.keys()),
                     help=f"World short name (default: {DEFAULT_WORLD})")
    sim.add_argument("--drone", default=DEFAULT_DRONE,
                     choices=list(DRONE_REFS.keys()),
                     help=f"Drone profile (default: {DEFAULT_DRONE})")
    sim.add_argument("--cam-pitch", type=float, default=-80.0,
                     help="Pilot camera pitch in degrees (default: -80)")
    # ── Per-camera on/off switches (all sensors switchable before launch) ──
    sim.add_argument("--pilot-cam", action=argparse.BooleanOptionalAction, default=True,
                     help="Enable the pilot/FPV camera (rectilinear, OSD) (default: on)")
    sim.add_argument("--tracker-wide-cam", action=argparse.BooleanOptionalAction, default=True,
                     help="Enable the wide-FOV tracker camera (fisheye by default) (default: on)")
    sim.add_argument("--tracker-narrow-cam", action=argparse.BooleanOptionalAction, default=False,
                     help="Enable the narrow-FOV tracker camera (rectilinear by default) (default: off)")
    sim.add_argument("--thermal-cam", action=argparse.BooleanOptionalAction, default=False,
                     help="Enable the thermal camera (rectilinear by default, white-hot) (default: off)")
    # ── Pilot (FPV) camera geometry — rectilinear ──
    sim.add_argument("--fpv-hfov", type=float, default=114.6,
                     help="Pilot camera horizontal FOV in degrees (default: 114.6)")
    sim.add_argument("--fpv-vfov", type=float, default=98.9,
                     help="Pilot camera vertical FOV in degrees (default: 98.9)")
    sim.add_argument("--fpv-cam-width", type=float, default=640,
                     help="Pilot camera output width in pixels (default: 640)")
    sim.add_argument("--fpv-cam-height", type=float, default=480,
                     help="Pilot camera output height in pixels (default: 480)")
    # ── Tracker WIDE camera geometry (fisheye/rectilinear via --tracker-wide-fisheye) ──
    sim.add_argument("--tracker-wide-cam-pitch", type=float, default=-80.0,
                     help="Wide tracker camera pitch in degrees (default: -80)")
    sim.add_argument("--tracker-wide-cam-roll", type=float, default=0.0,
                     help="Wide tracker camera roll in degrees (default: 0)")
    sim.add_argument("--tracker-wide-cam-offset-x", type=float, default=0.0,
                     help="Wide tracker camera mount offset X in mm, drone body frame "
                          "(x forward); unrotated by the camera tilt/twist (default: 0)")
    sim.add_argument("--tracker-wide-cam-offset-y", type=float, default=0.0,
                     help="Wide tracker camera mount offset Y in mm, drone body frame "
                          "(y positive to the drone's RIGHT); unrotated by the camera "
                          "tilt/twist (default: 0)")
    sim.add_argument("--tracker-wide-hfov", type=float, default=114.6,
                     help="Wide tracker camera horizontal FOV in degrees (default: 114.6)")
    sim.add_argument("--tracker-wide-vfov", type=float, default=98.9,
                     help="Wide tracker camera vertical FOV in degrees (default: 98.9)")
    sim.add_argument("--tracker-wide-cam-width", type=float, default=640,
                     help="Wide tracker camera source width in pixels (default: 640)")
    sim.add_argument("--tracker-wide-cam-height", type=float, default=480,
                     help="Wide tracker camera output height in pixels (default: 480)")
    sim.add_argument("--tracker-wide-cam-fps", type=int, default=30,
                     help="Wide tracker camera Gazebo update rate in Hz (default: 30)")
    sim.add_argument("--tracker-wide-fisheye", action=argparse.BooleanOptionalAction, default=True,
                     help="Render the wide tracker as fisheye (wideanglecamera); "
                          "--no-tracker-wide-fisheye makes it rectilinear (default: fisheye)")
    # ── Tracker NARROW camera geometry (rectilinear/fisheye via --tracker-narrow-fisheye) ──
    sim.add_argument("--tracker-narrow-cam-pitch", type=float, default=-80.0,
                     help="Narrow tracker camera pitch in degrees (default: -80)")
    sim.add_argument("--tracker-narrow-cam-offset-x", type=float, default=0.0,
                     help="Narrow tracker camera mount offset X in mm, drone body frame "
                          "(x forward); unrotated by the camera tilt/twist (default: 0)")
    sim.add_argument("--tracker-narrow-cam-offset-y", type=float, default=0.0,
                     help="Narrow tracker camera mount offset Y in mm, drone body frame "
                          "(y positive to the drone's RIGHT); unrotated by the camera "
                          "tilt/twist (default: 0)")
    sim.add_argument("--tracker-narrow-cam-roll", type=float, default=0.0,
                     help="Narrow tracker camera roll in degrees (default: 0)")
    sim.add_argument("--tracker-narrow-hfov", type=float, default=45.0,
                     help="Narrow tracker camera horizontal FOV in degrees (default: 45)")
    sim.add_argument("--tracker-narrow-vfov", type=float, default=34.0,
                     help="Narrow tracker camera vertical FOV in degrees (default: 34)")
    sim.add_argument("--tracker-narrow-cam-width", type=float, default=640,
                     help="Narrow tracker camera source width in pixels (default: 640)")
    sim.add_argument("--tracker-narrow-cam-height", type=float, default=480,
                     help="Narrow tracker camera output height in pixels (default: 480)")
    sim.add_argument("--tracker-narrow-cam-fps", type=int, default=30,
                     help="Narrow tracker camera Gazebo update rate in Hz (default: 30)")
    sim.add_argument("--tracker-narrow-fisheye", action=argparse.BooleanOptionalAction, default=False,
                     help="Render the narrow tracker as fisheye (wideanglecamera); default "
                          "rectilinear, pass --tracker-narrow-fisheye to enable (default: rectilinear)")
    # ── Thermal camera geometry (rectilinear/fisheye via --thermal-fisheye), white-hot ──
    sim.add_argument("--thermal-cam-pitch", type=float, default=-80.0,
                     help="Thermal camera pitch in degrees (default: -80)")
    sim.add_argument("--thermal-cam-offset-x", type=float, default=0.0,
                     help="Thermal camera mount offset X in mm, drone body frame "
                          "(x forward); unrotated by the camera tilt/twist (default: 0)")
    sim.add_argument("--thermal-cam-offset-y", type=float, default=0.0,
                     help="Thermal camera mount offset Y in mm, drone body frame "
                          "(y positive to the drone's RIGHT); unrotated by the camera "
                          "tilt/twist (default: 0)")
    sim.add_argument("--thermal-cam-roll", type=float, default=0.0,
                     help="Thermal camera roll in degrees (default: 0)")
    sim.add_argument("--thermal-hfov", type=float, default=114.6,
                     help="Thermal camera horizontal FOV in degrees (default: 114.6)")
    sim.add_argument("--thermal-vfov", type=float, default=98.9,
                     help="Thermal camera vertical FOV in degrees (default: 98.9)")
    sim.add_argument("--thermal-cam-width", type=float, default=640,
                     help="Thermal camera source width in pixels (default: 640)")
    sim.add_argument("--thermal-cam-height", type=float, default=480,
                     help="Thermal camera output height in pixels (default: 480)")
    sim.add_argument("--thermal-cam-fps", type=int, default=30,
                     help="Thermal camera Gazebo update rate in Hz (default: 30)")
    sim.add_argument("--thermal-fisheye", action=argparse.BooleanOptionalAction, default=False,
                     help="Render the thermal cam as fisheye (wideanglecamera); default "
                          "rectilinear, pass --thermal-fisheye to enable (default: rectilinear)")
    # ── Utility camera (clone of the wide tracker; clean, no OSD; own SHM) ──
    sim.add_argument("--utility-cam", action=argparse.BooleanOptionalAction, default=False,
                     help="Enable the utility camera — clone of the wide tracker feed "
                          "(clean/no-OSD, own SHM fpv_utility_cam) (default: off)")
    sim.add_argument("--utility-cam-pitch", type=float, default=-80.0,
                     help="Utility camera pitch in degrees (default: -80)")
    sim.add_argument("--utility-cam-roll", type=float, default=0.0,
                     help="Utility camera roll in degrees (default: 0)")
    sim.add_argument("--utility-hfov", type=float, default=114.6,
                     help="Utility camera horizontal FOV in degrees (default: 114.6)")
    sim.add_argument("--utility-vfov", type=float, default=98.9,
                     help="Utility camera vertical FOV in degrees (default: 98.9)")
    sim.add_argument("--utility-cam-width", type=float, default=640,
                     help="Utility camera source width in pixels (default: 640)")
    sim.add_argument("--utility-cam-height", type=float, default=480,
                     help="Utility camera output height in pixels (default: 480)")
    sim.add_argument("--utility-cam-fps", type=int, default=30,
                     help="Utility camera Gazebo update rate in Hz (default: 30)")
    sim.add_argument("--utility-fisheye", action=argparse.BooleanOptionalAction, default=True,
                     help="Render the utility cam as fisheye (wideanglecamera); "
                          "--no-utility-fisheye makes it rectilinear (default: fisheye)")
    # Per-camera fisheye lens intrinsics (--<cam>-lens-c1/c2/c3/fun); see common.add_lens_args.
    add_lens_args(sim)
    # Backward compatibility: applies to both cameras if explicitly provided.
    sim.add_argument("--cam-width", type=float, default=None,
                     help="Deprecated: output width for both pilot/tracker cameras")
    sim.add_argument("--cam-height", type=float, default=None,
                     help="Deprecated: output height for both pilot/tracker cameras")
    sim.add_argument("--gazebo", action="store_true",
                     help="Show the Gazebo GUI (default: headless)")
    sim.add_argument("--no-clouds", dest="clouds", action="store_false", default=True,
                     help="Disable clouds in the world skybox (default: clouds on)")
    sim.add_argument("--cloud-density", dest="cloud_density", type=float, default=0.7,
                     help="Cloud coverage 0.0-1.0: 0.8 = stock, 1.0 = denser than "
                          "stock, lower = fewer/wispier (cirrus) (default: 0.7)")
    sim.add_argument("--cloud-darkness", dest="cloud_darkness", type=float, default=0.0,
                     help="Cloud darkness 0.0-1.0: 0 = white, 1 ≈ dark grey nimbus "
                          "(default: 0)")
    sim.add_argument("--terrain-theme", choices=list(TERRAIN_THEMES), default=None,
                     help="Terrain theme: swaps the baylands ground textures + "
                          "ground-plane tint (desert | lush) (default: desert)")
    sim.add_argument("--sky-brightness", dest="sky_brightness", type=float, default=None,
                     help="Sky/sun brightness 0.15-1.0: 1.0 = bright noon, lower = "
                          "warmer, lower sun like late afternoon (default: 1.0)")
    sim.add_argument("--chase-cam", action=argparse.BooleanOptionalAction, default=False,
                     help="Enable the chase camera (3rd-person SDL2 window) (default: off)")
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
                     help="Path to a model-ready JSON params file for px4_sim_bridge")
    brg.add_argument("--tcp-port", type=int, default=4560,
                     help="TCP port for PX4 simulator connection (default: 4560)")
    brg.add_argument("--telem-port", type=int, default=0,
                     help="UDP port for raw telemetry output (0=off)")
    brg.add_argument("--mavlink-port", type=int, default=14560,
                     help="MAVLink UDP port for OSD telemetry (default: 14560)")
    brg.add_argument("--pedestal-radius", type=float, default=None,
                     help="Pedestal vis-cylinder radius (m); matches Simulink "
                          "pedestal_radius (default 0.5)")
    brg.add_argument("--pedestal-height", type=float, default=None,
                     help="Pedestal vis-cylinder height (m); matches Simulink "
                          "pedestal_height (default 0.30)")

    gps = parser.add_argument_group("GPS reference")
    gps.add_argument("--lat", type=float, default=47.397742,
                     help="GPS reference latitude (default: 47.397742 Zurich)")
    gps.add_argument("--lon", type=float, default=8.545594,
                     help="GPS reference longitude (default: 8.545594)")
    gps.add_argument("--alt", type=float, default=488.0,
                     help="GPS reference altitude AMSL in metres (default: 488.0)")

    tgt = parser.add_argument_group("Target trajectory")
    tgt.add_argument("--target-altitude", type=float, default=None,
                     help="Target altitude in metres (moving_target: 50, balloon: 10)")
    tgt.add_argument("--target-distance-x", type=float, default=None,
                     help="Target initial X position in metres (default: 30)")
    tgt.add_argument("--target-distance-y", type=float, default=None,
                     help="Target initial Y position in metres (default: 0)")
    tgt.add_argument("--target-speed", type=float, default=None,
                     help="Target speed km/h along the moving_target trajectory (default: 18)")
    tgt.add_argument("--target-orbit-radius", type=float, default=None,
                     help="Park-chase orbit radius in metres (default: 30)")
    tgt.add_argument("--target-orbit-center-x", type=float, default=None,
                     help="Park-chase orbit centre X in metres relative to player spawn (default: 0)")
    tgt.add_argument("--target-orbit-center-y", type=float, default=None,
                     help="Park-chase orbit centre Y in metres relative to player spawn (default: 0)")
    tgt.add_argument("--target-orbit-theta-deg", type=float, default=None,
                     help="Park-chase target initial tangential angle theta in degrees "
                          "(0 = +X from centre, 90 = +Y, default: 0)")
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
    tgt.add_argument("--patrol-lateral-offset", type=float, default=None,
                     help="Patrol lateral offset in metres so the target does not fly directly overhead (default: 0)")
    tgt.add_argument("--patrol-rotation", type=float, default=None,
                     help="Patrol path rotation in degrees clockwise when viewed from above (default: 0)")
    # ── Moving-target parametric trajectory (oval / circle / line) ──
    tgt.add_argument("--traj-type", choices=list(TRAJ_TYPES), default=None,
                     help="Trajectory preset (oval | circle | line) — seeds the loop "
                          "geometry; any loop param overrides it (default: oval)")
    tgt.add_argument("--traj-rotation-deg", type=float, default=None,
                     help="Trajectory rotation from the east axis in degrees, applied first (default: 0)")
    tgt.add_argument("--traj-offset-ew", type=float, default=None,
                     help="Target start offset EAST of the player in metres, after rotation (default: 0)")
    tgt.add_argument("--traj-offset-ns", type=float, default=None,
                     help="Target start offset NORTH of the player in metres, after rotation (default: 0)")
    tgt.add_argument("--oval-ew-length", type=float, default=None,
                     help="Loop straight E-W section length in metres (0 ⇒ no E-W straight)")
    tgt.add_argument("--oval-ns-length", type=float, default=None,
                     help="Loop straight N-S section length in metres (0 ⇒ no N-S straight)")
    tgt.add_argument("--corner-radius", type=float, default=None,
                     help="Loop rounded-corner radius in metres (0 ⇒ sharp; circle ⇒ the radius)")
    tgt.add_argument("--player-heading-deg", type=float, default=None,
                     help="Player drone initial heading in degrees from east, CCW (default: 0)")
    tgt.add_argument("--pilot-heading-deg", type=float, default=None,
                     help="pilot_controlled_target: the target's initial heading in "
                          "degrees from east, CCW — the world spawn yaw; must match "
                          "pilot_target.py --initial-heading-deg (default: 0)")
    tgt.add_argument("--traj-start-pos", type=float, default=None,
                     help="Target start position along the loop, 0..1 of the perimeter (default: 0)")
    tgt.add_argument("--traj-reverse", action="store_true",
                     help="Reverse the target's travel direction along the loop")
    tgt.add_argument("--traj-perturb", action="store_true",
                     help="Add sinusoidal perturbations about the nominal trajectory: a "
                          "lateral weave + an altitude oscillation, with a flown attitude "
                          "(pitch follows the climb, roll banks into the weave and the "
                          "loop corners)")
    tgt.add_argument("--traj-perturb-lat-amp", type=float, default=10.0,
                     help="Perturbation: lateral weave amplitude in metres, left/right of "
                          "the nominal track (default: 10)")
    tgt.add_argument("--traj-perturb-lat-rate", type=float, default=0.1,
                     help="Perturbation: lateral weave rate in Hz (default: 0.1)")
    tgt.add_argument("--traj-perturb-vert-amp", type=float, default=5.0,
                     help="Perturbation: altitude oscillation amplitude in metres about "
                          "the nominal altitude (default: 5)")
    tgt.add_argument("--traj-perturb-vert-rate", type=float, default=0.1,
                     help="Perturbation: altitude oscillation rate in Hz (default: 0.1)")
    tgt.add_argument("--traj-perturb-phase-deg", type=float, default=0.0,
                     help="Perturbation: altitude phase offset vs the weave in degrees "
                          "(default: 0)")
    tgt.add_argument("--wind-intensity", type=float, default=2.0,
                     help="Balloon lateral drift amplitude in metres (default: 2.0)")
    tgt.add_argument("--wind-randomness", type=float, default=1.0,
                     help="Balloon vertical bobbing amplitude in metres (default: 1.0)")
    tgt.add_argument("--drift-speed", type=float, default=20.0,
                     help="Balloon Lissajous frequency multiplier (default: 20.0)")
    # Balloon simple-harmonic mode (about the target centre); per-axis amp + rate.
    tgt.add_argument("--balloon-harmonic", action="store_true",
                     help="Balloon: per-axis simple-harmonic oscillation about the "
                          "target centre instead of the default Lissajous drift")
    tgt.add_argument("--balloon-amp-x", type=float, default=10.0,
                     help="Harmonic balloon X amplitude in metres (default: 10)")
    tgt.add_argument("--balloon-amp-y", type=float, default=10.0,
                     help="Harmonic balloon Y amplitude in metres (default: 10)")
    tgt.add_argument("--balloon-rate-x", type=float, default=0.05,
                     help="Harmonic balloon X oscillation rate in Hz (default: 0.05)")
    tgt.add_argument("--balloon-rate-y", type=float, default=0.05,
                     help="Harmonic balloon Y oscillation rate in Hz (default: 0.05)")
    tgt.add_argument("--balloon-phase-y-deg", type=float, default=90.0,
                     help="Harmonic balloon Y phase offset, degrees "
                          "(90=circle/ellipse, 0=diagonal line; default: 90)")
    tgt.add_argument("--target-drone", choices=list(TARGET_REFS.keys()),
                     default=None,
                     help="Target drone model to track (default per world: balloon for "
                          f"windy_target/shake_test, else {DEFAULT_TARGET_DRONE})")
    tgt.add_argument("--target-scale", type=float, default=None,
                     help="Uniform scale multiplier for the target mesh (any target; "
                          "default per target: shahed 1.0, stingjet 0.1, "
                          "falcon_trainer 1.0). Scales the hit-box too.")
    tgt.add_argument("--target-mesh-color", default=DEFAULT_TARGET_MESH_COLOR,
                     help="Flat colour for MESH targets (shahed/stingjet/falcon_trainer), overriding the "
                          "model's own greyish material. 'default' keeps it; presets: "
                          f"{', '.join(TARGET_MESH_COLORS)}; or a literal 'R G B' (0-1).")

    args = parser.parse_args()

    # Resolve backward-compat world aliases (e.g. balloon_test → windy_target).
    args.world = WORLD_ALIASES.get(args.world, args.world)

    # Resolve the effective target drone per world (force > explicit > default).
    _entry = WORLD_MAP.get(args.world, {})
    args.target_drone = (
        _entry.get("force_target")
        or args.target_drone
        or _entry.get("default_target")
        or DEFAULT_TARGET_DRONE
    )

    # shake_test: place the look-at balloon close and near drone height.
    if args.world == "shake_test":
        if args.target_distance_x is None:
            args.target_distance_x = 8.0
        if args.target_altitude is None:
            args.target_altitude = 2.0

    return args


def main():
    args = parse_args()

    if args.cam_width is not None:
        args.fpv_cam_width = args.cam_width
        args.tracker_wide_cam_width = args.cam_width
        args.tracker_narrow_cam_width = args.cam_width
    if args.cam_height is not None:
        args.fpv_cam_height = args.cam_height
        args.tracker_wide_cam_height = args.cam_height
        args.tracker_narrow_cam_height = args.cam_height

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
        pilot_cam_enabled=getattr(args, "pilot_cam", True),
        fpv_hfov_deg=args.fpv_hfov,
        fpv_vfov_deg=args.fpv_vfov,
        fpv_cam_width=args.fpv_cam_width,
        tracker_wide_cam_enabled=getattr(args, "tracker_wide_cam", True),
        tracker_wide_cam_pitch=args.tracker_wide_cam_pitch,
        tracker_wide_cam_roll=args.tracker_wide_cam_roll,
        tracker_wide_cam_offset_x_mm=getattr(args, "tracker_wide_cam_offset_x", 0.0),
        tracker_wide_cam_offset_y_mm=getattr(args, "tracker_wide_cam_offset_y", 0.0),
        tracker_wide_hfov_deg=args.tracker_wide_hfov,
        tracker_wide_vfov_deg=args.tracker_wide_vfov,
        tracker_wide_cam_width=args.tracker_wide_cam_width,
        tracker_wide_cam_fps=getattr(args, "tracker_wide_cam_fps", 30),
        tracker_wide_fisheye=getattr(args, "tracker_wide_fisheye", True),
        tracker_narrow_enabled=getattr(args, "tracker_narrow_cam", False),
        tracker_narrow_cam_pitch=args.tracker_narrow_cam_pitch,
        tracker_narrow_cam_roll=args.tracker_narrow_cam_roll,
        tracker_narrow_cam_offset_x_mm=getattr(args, "tracker_narrow_cam_offset_x", 0.0),
        tracker_narrow_cam_offset_y_mm=getattr(args, "tracker_narrow_cam_offset_y", 0.0),
        tracker_narrow_hfov_deg=args.tracker_narrow_hfov,
        tracker_narrow_vfov_deg=args.tracker_narrow_vfov,
        tracker_narrow_cam_width=args.tracker_narrow_cam_width,
        tracker_narrow_cam_fps=getattr(args, "tracker_narrow_cam_fps", 30),
        tracker_narrow_fisheye=getattr(args, "tracker_narrow_fisheye", False),
        thermal_cam_enabled=getattr(args, "thermal_cam", False),
        thermal_cam_pitch=getattr(args, "thermal_cam_pitch", -80.0),
        thermal_cam_roll=getattr(args, "thermal_cam_roll", 0.0),
        thermal_cam_offset_x_mm=getattr(args, "thermal_cam_offset_x", 0.0),
        thermal_cam_offset_y_mm=getattr(args, "thermal_cam_offset_y", 0.0),
        thermal_hfov_deg=getattr(args, "thermal_hfov", 114.6),
        thermal_vfov_deg=getattr(args, "thermal_vfov", 98.9),
        thermal_cam_width=getattr(args, "thermal_cam_width", 640),
        thermal_cam_fps=getattr(args, "thermal_cam_fps", 30),
        thermal_fisheye=getattr(args, "thermal_fisheye", False),
        utility_cam_enabled=getattr(args, "utility_cam", False),
        utility_cam_pitch=getattr(args, "utility_cam_pitch", -80.0),
        utility_cam_roll=getattr(args, "utility_cam_roll", 0.0),
        utility_hfov_deg=getattr(args, "utility_hfov", 114.6),
        utility_vfov_deg=getattr(args, "utility_vfov", 98.9),
        utility_cam_width=getattr(args, "utility_cam_width", 640),
        utility_cam_fps=getattr(args, "utility_cam_fps", 30),
        utility_fisheye=getattr(args, "utility_fisheye", True),
        **lens_kwargs_from_args(args),
        chase_cam_enabled=getattr(args, "chase_cam", False),
    )
    world_vars = compute_world_vars(
        args.drone, args.world,
        target_altitude=args.target_altitude,
        target_speed=args.target_speed,
        orbit_radius=args.target_orbit_radius,
        orbit_center_x=getattr(args, "target_orbit_center_x", None),
        orbit_center_y=getattr(args, "target_orbit_center_y", None),
        orbit_theta_deg=getattr(args, "target_orbit_theta_deg", None),
        patrol_length=args.patrol_length,
        target_x=args.target_distance_x,
        target_y=args.target_distance_y,
        clouds=getattr(args, "clouds", True),
        cloud_density=getattr(args, "cloud_density", 0.7),
        cloud_darkness=getattr(args, "cloud_darkness", 0.0),
        pedestal_radius=getattr(args, "pedestal_radius", None),
        pedestal_height=getattr(args, "pedestal_height", None),
        target_drone=getattr(args, "target_drone", DEFAULT_TARGET_DRONE),
        target_scale=getattr(args, "target_scale", None),
        target_mesh_color=getattr(args, "target_mesh_color", None),
        traj_type=getattr(args, "traj_type", None),
        traj_rotation_deg=getattr(args, "traj_rotation_deg", None),
        traj_offset_ew=getattr(args, "traj_offset_ew", None),
        traj_offset_ns=getattr(args, "traj_offset_ns", None),
        oval_ew_len=getattr(args, "oval_ew_length", None),
        oval_ns_len=getattr(args, "oval_ns_length", None),
        corner_radius=getattr(args, "corner_radius", None),
        traj_start_pos=getattr(args, "traj_start_pos", None),
        traj_reverse=getattr(args, "traj_reverse", False),
        player_heading_deg=getattr(args, "player_heading_deg", None),
        pilot_heading_deg=getattr(args, "pilot_heading_deg", None),
        terrain_theme=getattr(args, "terrain_theme", None),
        sky_brightness=getattr(args, "sky_brightness", None),
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
    target_bbox  = (world_vars.get("target_bbox")
                    if world_entry.get("target_drone")
                    else world_entry.get("target_bbox"))
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
    # Optional clean (no-OSD) wide + narrow tracker + thermal feeds.
    start_tracker_bridges(args, pm)

    # ── 5. Target trajectory threads ──
    traj_stop = threading.Event()

    if world_entry.get("trajectory_drive") and world_entry.get("target_model"):
        speed_ms = (args.target_speed if args.target_speed is not None else 18.0) / 3.6
        target_z = args.target_altitude if args.target_altitude is not None else 50.0
        start_trajectory_thread(
            traj_stop,
            traj_type=getattr(args, "traj_type", None) or "oval",
            speed_ms=speed_ms,
            target_z=target_z,
            rotation_deg=args.traj_rotation_deg if args.traj_rotation_deg is not None else 0.0,
            offset_ew=args.traj_offset_ew if args.traj_offset_ew is not None else 0.0,
            offset_ns=args.traj_offset_ns if args.traj_offset_ns is not None else 0.0,
            oval_ew_len=args.oval_ew_length,
            oval_ns_len=args.oval_ns_length,
            corner_radius=args.corner_radius,
            start_pos=args.traj_start_pos if args.traj_start_pos is not None else 0.0,
            reverse=getattr(args, "traj_reverse", False),
            perturb=getattr(args, "traj_perturb", False),
            perturb_lat_amp=getattr(args, "traj_perturb_lat_amp", 10.0),
            perturb_lat_rate=getattr(args, "traj_perturb_lat_rate", 0.1),
            perturb_vert_amp=getattr(args, "traj_perturb_vert_amp", 5.0),
            perturb_vert_rate=getattr(args, "traj_perturb_vert_rate", 0.1),
            perturb_phase_deg=getattr(args, "traj_perturb_phase_deg", 0.0),
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
            harmonic=getattr(args, "balloon_harmonic", False),
            amp_x=getattr(args, "balloon_amp_x", 10.0),
            amp_y=getattr(args, "balloon_amp_y", 10.0),
            rate_x=getattr(args, "balloon_rate_x", 0.05),
            rate_y=getattr(args, "balloon_rate_y", 0.05),
            phase_y_deg=getattr(args, "balloon_phase_y_deg", 90.0),
        )

    elif world_entry.get("static_target"):
        # Fixed <include>d target (collision_test): publish its known ENU pose to
        # the GT mirror port so sitl_redis_bridge's target:gps populates. Uses the
        # same world_vars the world SDF was rendered from → drawn target == GT.
        start_static_target_thread(
            traj_stop,
            x=world_vars["target_x"],
            y=world_vars["target_y"],
            z=world_vars["target_z"],
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
    print("  From droneleaf-sitl-noetic container, set PX4_SIM_HOSTNAME=<host-ip>")
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
