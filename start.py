#!/usr/bin/env python3
"""Unified simulation launcher — Betaflight SITL + Gazebo + Simulink dynamics.

Gazebo runs visualization-only: the drone pose is driven over UDP by
bf_sim_bridge, which loads a pre-compiled Simulink .so and steps the rigid-body
dynamics at 250 Hz.  OSD is always enabled.

Cameras (each independently switchable on/off before launch):
  - pilot          fpv_cam                rectilinear, OSD   --pilot-cam/--no-pilot-cam
  - tracker-wide   fpv_tracker_wide_cam   fisheye (default)  --tracker-wide-cam/--no-...
  - tracker-narrow fpv_tracker_narrow_cam rectilinear (def.) --tracker-narrow-cam/--no-...
  - thermal        fpv_thermal_cam        rectilinear, w-hot --thermal-cam/--no-thermal-cam
  - chase          chase_cam              rectilinear 3rd-pp --chase-cam/--no-chase-cam

Each tracker/thermal feed's projection is independently selectable between fisheye
(wideanglecamera) and rectilinear via --<feed>-fisheye / --no-<feed>-fisheye.
Fisheye matches the spherical-calibrated camera the MSC/PN guidance estimator
assumes; rectilinear renders faster (fisheye cubemaps serialise the gz render
thread). Defaults: wide=fisheye, narrow=rectilinear, thermal=rectilinear.

Usage:
    python3 start.py --world moving_target --gazebo
    python3 start.py --world collision_test --cam-pitch -90 --gazebo
    python3 start.py --drone iris --gazebo --chase-cam
    python3 start.py --world moving_target --tracker-narrow-cam --thermal-cam --gazebo
    python3 start.py --world moving_target --no-pilot-cam --gazebo

    # Tune drone params
    python3 start.py --world moving_target --ctw 5 --angular-damping 0.05
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
    DEFAULT_TARGET_DRONE,
    DRONE_REFS,
    IMAGE_BRIDGE,
    SIMULINK_LIB,
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
    discover_camera_topic,
    list_camera_topics,
    read_image_meta,
    render_template,
    render_vis_templates,
    setup_gazebo_env,
    start_balloon_thread,
    start_static_target_thread,
    start_trajectory_thread,
    TERRAIN_THEMES,
    TRAJ_TYPES,
    start_tracker_bridges,
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

DEFAULT_WORLD = "moving_target"
DEFAULT_DRONE = "rocket_drone"

# Short world name → dict of world attributes.
#   sim_world    — Simulink vis-only SDF file (Gazebo = visualization only)
#   gz_name      — Gazebo <world name> element
#   target_model — SDF model name of the target (for proximity OSD)
#   target_link  — link inside target model whose world pose to track
#   target_drone — True if the target is a selectable drone (bbox from TARGET_REFS)
#   trajectory_drive — drive the target along a parametric trajectory (moving_target)
WORLD_MAP = {
    "moving_target": {
        "sim_world":    "rocket_drone_moving_target_vis.sdf",
        "gz_name":      "fpv_moving_target",
        "target_model": "moving_target",
        "target_link":  "geranium_link",
        "target_drone": True,
        "trajectory_drive": True,
    },
    "collision_test": {
        "sim_world":    "rocket_drone_collision_test_vis.sdf",
        "gz_name":      "collision_test",
        "target_model": "collision_test_target",
        "target_drone": True,
        "static_target": True,
    },
    "windy_target": {
        "sim_world":    "rocket_drone_windy_target_vis.sdf",
        "gz_name":      "windy_target",
        "target_model": "windy_target",
        # Selectable target (balloon/shahed/stingjet), defaults to balloon. Its
        # pose is driven by the wind (Lissajous drift / harmonic) thread on 9014.
        "target_drone":   True,
        "default_target": "balloon",
        "balloon_wind":   True,
    },
    "shake_test": {
        "sim_world":    "rocket_drone_shake_test_vis.sdf",
        "gz_name":      "shake_test",
        # The bridge runs with --shake so the DRONE itself oscillates (X/Y) +
        # rocks (roll/pitch); BF's IMU reflects it. A static balloon sits in
        # front for the drone to look at (forced balloon, not shahed/stingjet).
        "shake":         True,
        "target_model":  "shake_target",
        "target_drone":  True,
        "force_target":  "balloon",
        "static_target": True,
    },
}

# Backward-compat world short-name aliases (old name → canonical).
WORLD_ALIASES = {
    "balloon_test": "windy_target",
}

# ── BF-specific Helpers ───────────────────────────────────────────────────────


def _render_all_templates(drone, world_name, args):
    """Render the vis-only Jinja2 templates for the selected drone and world."""
    ref = DRONE_REFS[drone]

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
        pilot_cam_enabled=getattr(args, "pilot_cam", True),
        fpv_hfov_deg=args.fpv_hfov,
        fpv_vfov_deg=args.fpv_vfov,
        fpv_cam_width=args.fpv_cam_width,
        tracker_wide_cam_enabled=getattr(args, "tracker_wide_cam", True),
        tracker_wide_cam_pitch=args.tracker_wide_cam_pitch,
        tracker_wide_cam_roll=args.tracker_wide_cam_roll,
        tracker_wide_hfov_deg=args.tracker_wide_hfov,
        tracker_wide_vfov_deg=args.tracker_wide_vfov,
        tracker_wide_cam_width=args.tracker_wide_cam_width,
        tracker_wide_cam_fps=getattr(args, "tracker_wide_cam_fps", 30),
        tracker_wide_fisheye=getattr(args, "tracker_wide_fisheye", True),
        tracker_narrow_enabled=getattr(args, "tracker_narrow_cam", False),
        tracker_narrow_cam_pitch=args.tracker_narrow_cam_pitch,
        tracker_narrow_cam_roll=args.tracker_narrow_cam_roll,
        tracker_narrow_hfov_deg=args.tracker_narrow_hfov,
        tracker_narrow_vfov_deg=args.tracker_narrow_vfov,
        tracker_narrow_cam_width=args.tracker_narrow_cam_width,
        tracker_narrow_cam_fps=getattr(args, "tracker_narrow_cam_fps", 30),
        tracker_narrow_fisheye=getattr(args, "tracker_narrow_fisheye", False),
        thermal_cam_enabled=getattr(args, "thermal_cam", False),
        thermal_cam_pitch=getattr(args, "thermal_cam_pitch", -80.0),
        thermal_cam_roll=getattr(args, "thermal_cam_roll", 0.0),
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

    log.info("CTW=%.1f mass=%.3fkg Ixx=%.6f Iyy=%.6f Izz=%.6f standoff=%.3fm cam_pitch=%.1f°",
             ctw or ref["default_ctw"],
             model_vars["mass"], model_vars["ixx"], model_vars["iyy"], model_vars["izz"],
             model_vars["standoff_height"], args.cam_pitch)

    # ── World variables (shared helper) ──
    world_vars = compute_world_vars(
        drone, world_name,
        target_altitude=args.target_altitude,
        target_speed=args.target_speed,
        orbit_radius=args.target_orbit_radius,
        orbit_center_x=getattr(args, "target_orbit_center_x", None),
        orbit_center_y=getattr(args, "target_orbit_center_y", None),
        orbit_theta_deg=getattr(args, "target_orbit_theta_deg", None),
        patrol_length=args.patrol_length,
        target_x=getattr(args, "target_distance_x", None),
        target_y=getattr(args, "target_distance_y", None),
        clouds=getattr(args, "clouds", True),
        cloud_density=getattr(args, "cloud_density", 0.7),
        pedestal_radius=getattr(args, "pedestal_radius", None),
        pedestal_height=getattr(args, "pedestal_height", None),
        target_drone=getattr(args, "target_drone", DEFAULT_TARGET_DRONE),
        target_scale=getattr(args, "target_scale", None),
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
        terrain_theme=getattr(args, "terrain_theme", None),
        sky_brightness=getattr(args, "sky_brightness", None),
    )

    # ── Render vis model + vis world (shared helper) ──
    render_vis_templates(drone, world_name, WORLD_MAP, model_vars, world_vars)

    return world_vars


def send_bf_cli_commands(commands, host="127.0.0.1", port=5761, timeout=2.0):
    """Send a small list of CLI commands to BF SITL over TCP."""
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            # Drain banner/prompt if present.
            try:
                sock.recv(4096)
            except OSError:
                pass
            for cmd in commands:
                sock.sendall((cmd + "\n").encode("utf-8"))
                time.sleep(0.05)
                try:
                    sock.recv(4096)
                except OSError:
                    pass
        return True
    except OSError as e:
        log.warning("Failed to send BF CLI commands to %s:%d: %s", host, port, e)
        return False


# ── Main ──────────────────────────────────────────────────────────────────────


def parse_args():
    epilog_lines = ["available worlds:"]
    for name, entry in sorted(WORLD_MAP.items()):
        epilog_lines.append(f"  {name:20s}  {entry['sim_world']}")

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
        choices=list(WORLD_MAP.keys()) + list(WORLD_ALIASES.keys()),
        help=f"World short name (default: {DEFAULT_WORLD})",
    )
    sim.add_argument(
        "--physics",
        choices=["simulink"],
        default="simulink",
        help="Dynamics backend; engines follow the simulink interface "
             "(Gazebo is visualization-only). Only 'simulink' for now.",
    )
    sim.add_argument(
        "--gazebo",
        action="store_true",
        help="Show the Gazebo GUI (default: headless)",
    )
    sim.add_argument(
        "--no-clouds",
        dest="clouds",
        action="store_false",
        default=True,
        help="Disable clouds in the world skybox (default: clouds on)",
    )
    sim.add_argument(
        "--cloud-density",
        dest="cloud_density",
        type=float,
        default=0.7,
        help="Cloud density / humidity 0.0-1.0 (default: 0.7)",
    )
    sim.add_argument(
        "--terrain-theme",
        choices=list(TERRAIN_THEMES),
        default=None,
        help="Terrain theme: swaps the baylands ground textures + ground-plane "
             "tint (desert | lush) (default: desert)",
    )
    sim.add_argument(
        "--sky-brightness",
        dest="sky_brightness",
        type=float,
        default=None,
        help="Sky/sun brightness 0.15-1.0: 1.0 = bright noon, lower = warmer, "
             "lower sun like late afternoon (default: 1.0)",
    )
    # ── Per-camera on/off switches (all sensors switchable before launch) ──
    sim.add_argument(
        "--pilot-cam",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable the pilot/FPV camera (rectilinear, OSD) (default: on)",
    )
    sim.add_argument(
        "--tracker-wide-cam",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable the wide-FOV tracker camera (fisheye by default) (default: on)",
    )
    sim.add_argument(
        "--tracker-narrow-cam",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable the narrow-FOV tracker camera (rectilinear by default) (default: off)",
    )
    sim.add_argument(
        "--chase-cam",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable the chase camera (3rd-person SDL2 window) (default: off)",
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
    # ── Pilot (FPV) camera geometry — rectilinear ──
    drn.add_argument(
        "--fpv-hfov",
        type=float,
        default=114.6,
        help="Pilot camera horizontal FOV in degrees (default: 114.6)",
    )
    drn.add_argument(
        "--fpv-vfov",
        type=float,
        default=98.9,
        help="Pilot camera vertical FOV in degrees (default: 98.9)",
    )
    drn.add_argument("--fpv-cam-width", type=float, default=640,
                     help="Pilot camera output width in pixels (default: 640)")
    drn.add_argument("--fpv-cam-height", type=float, default=480,
                     help="Pilot camera output height in pixels (default: 480)")

    # ── Tracker WIDE camera geometry (fisheye/rectilinear via --tracker-wide-fisheye) ──
    drn.add_argument(
        "--tracker-wide-cam-pitch",
        type=float,
        default=-80.0,
        help="Wide tracker camera pitch in degrees (default: -80)",
    )
    drn.add_argument(
        "--tracker-wide-cam-roll",
        type=float,
        default=0.0,
        help="Wide tracker camera roll in degrees (default: 0; use 90 for landscape)",
    )
    drn.add_argument(
        "--tracker-wide-hfov",
        type=float,
        default=114.6,
        help="Wide tracker camera horizontal FOV in degrees (default: 114.6)",
    )
    drn.add_argument(
        "--tracker-wide-vfov",
        type=float,
        default=98.9,
        help="Wide tracker camera vertical FOV in degrees (default: 98.9)",
    )
    drn.add_argument("--tracker-wide-cam-width", type=float, default=640,
                     help="Wide tracker camera output width in pixels (default: 640)")
    drn.add_argument("--tracker-wide-cam-height", type=float, default=480,
                     help="Wide tracker camera output height in pixels (default: 480)")
    drn.add_argument("--tracker-wide-cam-fps", type=int, default=30,
                     help="Wide tracker camera Gazebo update rate in Hz; also the RTSP "
                          "stream framerate (default: 30)")
    drn.add_argument("--tracker-wide-fisheye", action=argparse.BooleanOptionalAction, default=True,
                     help="Render the wide tracker as fisheye (wideanglecamera). "
                          "--no-tracker-wide-fisheye makes it rectilinear (default: fisheye)")
    drn.add_argument("--tracker-wide-rtsp", type=str, default=None, metavar="URL",
                     help="Push the (clean, no-OSD) wide tracker feed as H.264 to this "
                          "RTSP URL, e.g. rtsp://127.0.0.1:8554/tracker_wide (off if unset)")
    drn.add_argument("--tracker-wide-rtsp-bitrate", type=str, default="4M",
                     help="Wide tracker RTSP libx264 target/cap bitrate (default: 4M)")
    drn.add_argument("--tracker-wide-rtsp-crf", type=int, default=23,
                     help="Wide tracker RTSP libx264 CRF 0-51 (lower=better; 0=off/use bitrate), capped by bitrate (default: 23)")
    drn.add_argument("--tracker-wide-rtsp-gop", type=int, default=0,
                     help="Wide tracker RTSP GOP/keyframe interval in frames; shorter=faster reader startup (0=auto: fps*2) (default: 0)")
    drn.add_argument("--tracker-wide-rtsp-preset", type=str, default="ultrafast",
                     help="Wide tracker RTSP libx264 preset (default: ultrafast)")
    drn.add_argument("--tracker-wide-rtsp-tune", type=str, default="zerolatency",
                     help="Wide tracker RTSP libx264 tune; 'none' omits -tune (default: zerolatency)")
    drn.add_argument("--tracker-wide-rtsp-width", type=int, default=0,
                     help="Explicit wide tracker RTSP output width in px (0=camera width)")
    drn.add_argument("--tracker-wide-rtsp-height", type=int, default=0,
                     help="Explicit wide tracker RTSP output height in px (0=camera height)")

    # ── Tracker NARROW camera geometry (rectilinear/fisheye via --tracker-narrow-fisheye), narrower FOV ──
    drn.add_argument(
        "--tracker-narrow-cam-pitch",
        type=float,
        default=-80.0,
        help="Narrow tracker camera pitch in degrees (default: -80)",
    )
    drn.add_argument(
        "--tracker-narrow-cam-roll",
        type=float,
        default=0.0,
        help="Narrow tracker camera roll in degrees (default: 0)",
    )
    drn.add_argument(
        "--tracker-narrow-hfov",
        type=float,
        default=45.0,
        help="Narrow tracker camera horizontal FOV in degrees (default: 45)",
    )
    drn.add_argument(
        "--tracker-narrow-vfov",
        type=float,
        default=34.0,
        help="Narrow tracker camera vertical FOV in degrees (default: 34)",
    )
    drn.add_argument("--tracker-narrow-cam-width", type=float, default=640,
                     help="Narrow tracker camera output width in pixels (default: 640)")
    drn.add_argument("--tracker-narrow-cam-height", type=float, default=480,
                     help="Narrow tracker camera output height in pixels (default: 480)")
    drn.add_argument("--tracker-narrow-cam-fps", type=int, default=30,
                     help="Narrow tracker camera Gazebo update rate in Hz; also the RTSP "
                          "stream framerate (default: 30)")
    drn.add_argument("--tracker-narrow-fisheye", action=argparse.BooleanOptionalAction, default=False,
                     help="Render the narrow tracker as fisheye (wideanglecamera). Default is "
                          "rectilinear; pass --tracker-narrow-fisheye to enable (default: rectilinear)")
    drn.add_argument("--tracker-narrow-rtsp", type=str, default=None, metavar="URL",
                     help="Push the (clean, no-OSD) narrow tracker feed as H.264 to this "
                          "RTSP URL, e.g. rtsp://127.0.0.1:8554/tracker_narrow (off if unset)")
    drn.add_argument("--tracker-narrow-rtsp-bitrate", type=str, default="4M",
                     help="Narrow tracker RTSP libx264 target/cap bitrate (default: 4M)")
    drn.add_argument("--tracker-narrow-rtsp-crf", type=int, default=23,
                     help="Narrow tracker RTSP libx264 CRF 0-51 (lower=better; 0=off/use bitrate), capped by bitrate (default: 23)")
    drn.add_argument("--tracker-narrow-rtsp-gop", type=int, default=0,
                     help="Narrow tracker RTSP GOP/keyframe interval in frames; shorter=faster reader startup (0=auto: fps*2) (default: 0)")
    drn.add_argument("--tracker-narrow-rtsp-preset", type=str, default="ultrafast",
                     help="Narrow tracker RTSP libx264 preset (default: ultrafast)")
    drn.add_argument("--tracker-narrow-rtsp-tune", type=str, default="zerolatency",
                     help="Narrow tracker RTSP libx264 tune; 'none' omits -tune (default: zerolatency)")
    drn.add_argument("--tracker-narrow-rtsp-width", type=int, default=0,
                     help="Explicit narrow tracker RTSP output width in px (0=camera width)")
    drn.add_argument("--tracker-narrow-rtsp-height", type=int, default=0,
                     help="Explicit narrow tracker RTSP output height in px (0=camera height)")

    # ── Thermal camera (optional dedicated sensor, rectilinear/fisheye via --thermal-fisheye; white-hot) ──
    drn.add_argument("--thermal-cam", action=argparse.BooleanOptionalAction, default=False,
                     help="Enable the simulated thermal camera (a 2nd tracker feed, "
                          "white-hot grayscale, dedicated sensor, rectilinear by default) (default: off)")
    drn.add_argument("--thermal-cam-pitch", type=float, default=-80.0,
                     help="Thermal camera tilt in degrees (default: -80)")
    drn.add_argument("--thermal-cam-roll", type=float, default=0.0,
                     help="Thermal camera twist in degrees (default: 0)")
    drn.add_argument("--thermal-hfov", type=float, default=114.6,
                     help="Thermal camera horizontal FOV in degrees (default: 114.6)")
    drn.add_argument("--thermal-vfov", type=float, default=98.9,
                     help="Thermal camera vertical FOV in degrees (default: 98.9)")
    drn.add_argument("--thermal-cam-width", type=float, default=640,
                     help="Thermal camera output width in px (default: 640)")
    drn.add_argument("--thermal-cam-height", type=float, default=480,
                     help="Thermal camera output height in px (default: 480)")
    drn.add_argument("--thermal-cam-fps", type=int, default=30,
                     help="Thermal camera Gazebo update rate / RTSP framerate (default: 30)")
    drn.add_argument("--thermal-fisheye", action=argparse.BooleanOptionalAction, default=False,
                     help="Render the thermal cam as fisheye (wideanglecamera). Default is "
                          "rectilinear; pass --thermal-fisheye to enable (default: rectilinear)")
    drn.add_argument("--thermal-rtsp", type=str, default=None, metavar="URL",
                     help="Push the thermal feed as H.264 to this RTSP URL "
                          "(off if unset), e.g. rtsp://127.0.0.1:8554/thermal")
    drn.add_argument("--thermal-rtsp-bitrate", type=str, default="4M",
                     help="Thermal RTSP libx264 target/cap bitrate (default: 4M)")
    drn.add_argument("--thermal-rtsp-crf", type=int, default=23,
                     help="Thermal RTSP libx264 CRF 0-51 (lower=better; 0=off/use bitrate), capped by bitrate (default: 23)")
    drn.add_argument("--thermal-rtsp-gop", type=int, default=0,
                     help="Thermal RTSP GOP/keyframe interval in frames; shorter=faster reader startup (0=auto: fps*2) (default: 0)")
    drn.add_argument("--thermal-rtsp-preset", type=str, default="ultrafast",
                     help="Thermal RTSP libx264 preset (default: ultrafast)")
    drn.add_argument("--thermal-rtsp-tune", type=str, default="zerolatency",
                     help="Thermal RTSP libx264 tune; 'none' omits -tune (default: zerolatency)")
    drn.add_argument("--thermal-rtsp-width", type=int, default=0,
                     help="Explicit thermal RTSP output width in px (0=camera width)")
    drn.add_argument("--thermal-rtsp-height", type=int, default=0,
                     help="Explicit thermal RTSP output height in px (0=camera height)")

    # ── Utility camera (clone of the wide tracker; clean, no OSD; own SHM + RTSP) ──
    drn.add_argument("--utility-cam", action=argparse.BooleanOptionalAction, default=False,
                     help="Enable the utility camera — a clone of the wide tracker feed "
                          "(clean/no-OSD, own SHM `fpv_utility_cam` + optional RTSP) (default: off)")
    drn.add_argument("--utility-cam-pitch", type=float, default=-80.0,
                     help="Utility camera tilt in degrees (default: -80)")
    drn.add_argument("--utility-cam-roll", type=float, default=0.0,
                     help="Utility camera twist in degrees (default: 0)")
    drn.add_argument("--utility-hfov", type=float, default=114.6,
                     help="Utility camera horizontal FOV in degrees (default: 114.6)")
    drn.add_argument("--utility-vfov", type=float, default=98.9,
                     help="Utility camera vertical FOV in degrees (default: 98.9)")
    drn.add_argument("--utility-cam-width", type=float, default=640,
                     help="Utility camera output width in px (default: 640)")
    drn.add_argument("--utility-cam-height", type=float, default=480,
                     help="Utility camera output height in px (default: 480)")
    drn.add_argument("--utility-cam-fps", type=int, default=30,
                     help="Utility camera Gazebo update rate / RTSP framerate (default: 30)")
    drn.add_argument("--utility-fisheye", action=argparse.BooleanOptionalAction, default=True,
                     help="Render the utility cam as fisheye (wideanglecamera); "
                          "--no-utility-fisheye makes it rectilinear (default: fisheye)")
    # Per-camera fisheye lens intrinsics (--<cam>-lens-c1/c2/c3/fun); see common.add_lens_args.
    add_lens_args(drn)
    drn.add_argument("--utility-rtsp", type=str, default=None, metavar="URL",
                     help="Push the utility feed as H.264 to this RTSP URL "
                          "(off if unset), e.g. rtsp://127.0.0.1:8554/utility")
    drn.add_argument("--utility-rtsp-bitrate", type=str, default="4M",
                     help="Utility RTSP libx264 target/cap bitrate (default: 4M)")
    drn.add_argument("--utility-rtsp-crf", type=int, default=23,
                     help="Utility RTSP libx264 CRF 0-51 (lower=better; 0=off/use bitrate), capped by bitrate (default: 23)")
    drn.add_argument("--utility-rtsp-gop", type=int, default=0,
                     help="Utility RTSP GOP/keyframe interval in frames; shorter=faster reader startup (0=auto: fps*2) (default: 0)")
    drn.add_argument("--utility-rtsp-preset", type=str, default="ultrafast",
                     help="Utility RTSP libx264 preset (default: ultrafast)")
    drn.add_argument("--utility-rtsp-tune", type=str, default="zerolatency",
                     help="Utility RTSP libx264 tune; 'none' omits -tune (default: zerolatency)")
    drn.add_argument("--utility-rtsp-width", type=int, default=0,
                     help="Explicit utility RTSP output width in px (0=camera width)")
    drn.add_argument("--utility-rtsp-height", type=int, default=0,
                     help="Explicit utility RTSP output height in px (0=camera height)")

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
        help="Path to a model-ready JSON params file for bf_sim_bridge model parameters",
    )
    slk.add_argument(
        "--telem-port",
        type=int,
        default=0,
        help="UDP port for bf_sim_bridge to send raw Simulink state to sitl_redis_bridge (0=off)",
    )
    slk.add_argument(
        "--launcher-port",
        type=int,
        default=0,
        help="UDP port for bf_sim_bridge to receive launcher telemetry; pan->yaw, "
             "tilt->pitch (roll=0) drive the pedestal attitude (0=off)",
    )
    slk.add_argument(
        "--home-lat",
        type=float,
        default=None,
        help="SITL virtual-GPS home latitude in degrees (bridge default: 24.128861, UAE)",
    )
    slk.add_argument(
        "--home-lon",
        type=float,
        default=None,
        help="SITL virtual-GPS home longitude in degrees (bridge default: 55.785944, UAE)",
    )
    slk.add_argument(
        "--home-alt",
        type=float,
        default=None,
        help="SITL virtual-GPS home altitude (MSL, metres) (bridge default: 0)",
    )
    slk.add_argument(
        "--pedestal-radius",
        type=float,
        default=None,
        help="Pedestal launch-stand radius (m) for the vis cylinder under the "
             "drone; matches the Simulink pedestal_radius (default 0.5)",
    )
    slk.add_argument(
        "--pedestal-height",
        type=float,
        default=None,
        help="Pedestal launch-stand height (m) for the vis cylinder under the "
             "drone; matches the Simulink pedestal_height (default 0.30)",
    )

    # ── World Settings ───────────────────────────────────────────────────
    wld = parser.add_argument_group("World settings (moving_target / collision_test / balloon_test)")
    wld.add_argument(
        "--target-altitude",
        type=float,
        default=None,
        help="Target altitude in metres (moving_target default: 50, collision_test: 20, balloon: 10)",
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
        help="Target speed in km/h along the moving_target trajectory (default: 18)",
    )
    wld.add_argument(
        "--target-orbit-radius",
        type=float,
        default=None,
        help="Park-chase orbit radius in metres (default: 30)",
    )
    wld.add_argument(
        "--target-orbit-center-x",
        type=float,
        default=None,
        help="Park-chase orbit centre X in metres relative to player spawn (default: 0)",
    )
    wld.add_argument(
        "--target-orbit-center-y",
        type=float,
        default=None,
        help="Park-chase orbit centre Y in metres relative to player spawn (default: 0)",
    )
    wld.add_argument(
        "--target-orbit-theta-deg",
        type=float,
        default=None,
        help="Park-chase target initial tangential angle theta in degrees "
             "(0 = +X from centre, 90 = +Y, default: 0)",
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
        "--patrol-lateral-offset",
        type=float,
        default=None,
        help="Patrol lateral offset in metres so the target does not fly directly overhead (default: 0)",
    )
    wld.add_argument(
        "--patrol-rotation",
        type=float,
        default=None,
        help="Patrol path rotation in degrees clockwise when viewed from above (default: 0)",
    )
    # ── Moving-target parametric trajectory (oval / circle / line) ──
    wld.add_argument("--traj-type", choices=list(TRAJ_TYPES), default=None,
                     help="Trajectory preset (oval | circle | line) — seeds the loop "
                          "geometry below; any of the 3 loop params overrides it (default: oval)")
    wld.add_argument("--traj-rotation-deg", type=float, default=None,
                     help="Trajectory rotation from the east axis in degrees, applied first (default: 0)")
    wld.add_argument("--traj-offset-ew", type=float, default=None,
                     help="Target start offset EAST of the player in metres, after rotation (default: 0)")
    wld.add_argument("--traj-offset-ns", type=float, default=None,
                     help="Target start offset NORTH of the player in metres, after rotation (default: 0)")
    wld.add_argument("--oval-ew-length", type=float, default=None,
                     help="Loop straight E-W section length in metres (0 ⇒ no E-W straight)")
    wld.add_argument("--oval-ns-length", type=float, default=None,
                     help="Loop straight N-S section length in metres (0 ⇒ no N-S straight)")
    wld.add_argument("--corner-radius", type=float, default=None,
                     help="Loop rounded-corner radius in metres (0 ⇒ sharp; circle ⇒ the radius)")
    wld.add_argument("--player-heading-deg", type=float, default=None,
                     help="Player drone initial heading in degrees from east, CCW (default: 0)")
    wld.add_argument("--traj-start-pos", type=float, default=None,
                     help="Target start position along the loop, 0..1 of the perimeter (default: 0)")
    wld.add_argument("--traj-reverse", action="store_true",
                     help="Reverse the target's travel direction along the loop")
    wld.add_argument(
        "--hit-box-scale",
        type=float,
        default=None,
        help="Uniform scale multiplier for the target hit box (default: 1.0)",
    )
    wld.add_argument(
        "--target-drone",
        choices=list(TARGET_REFS.keys()),
        default=None,
        help=f"Target drone model to track (default per world: balloon for "
             f"windy_target/shake_test, else {DEFAULT_TARGET_DRONE}; "
             f"choices: {', '.join(TARGET_REFS)})",
    )
    wld.add_argument(
        "--target-scale",
        type=float,
        default=None,
        help="Uniform scale multiplier for the target mesh (applies to any target; "
             "default per target: shahed 1.0, stingjet 0.1). Also scales the hit-box.",
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
    # Balloon simple-harmonic mode (about the target centre = --target-distance-x/-y
    # + --target-altitude). Per-axis amplitude + rate; phase_y gives circle/ellipse.
    wld.add_argument("--balloon-harmonic", action="store_true",
                     help="Balloon: per-axis simple-harmonic oscillation about the "
                          "target centre instead of the default Lissajous drift")
    wld.add_argument("--balloon-amp-x", type=float, default=10.0,
                     help="Harmonic balloon X amplitude in metres (default: 10)")
    wld.add_argument("--balloon-amp-y", type=float, default=10.0,
                     help="Harmonic balloon Y amplitude in metres (default: 10)")
    wld.add_argument("--balloon-rate-x", type=float, default=0.05,
                     help="Harmonic balloon X oscillation rate in Hz (default: 0.05)")
    wld.add_argument("--balloon-rate-y", type=float, default=0.05,
                     help="Harmonic balloon Y oscillation rate in Hz (default: 0.05)")
    wld.add_argument("--balloon-phase-y-deg", type=float, default=90.0,
                     help="Harmonic balloon Y phase offset, degrees "
                          "(90=circle/ellipse, 0=diagonal line; default: 90)")

    # ── Shake-table (shake_test world, or --shake on any world) ──
    # Kinematic harmonic shake of the DRONE itself, injected in bf_sim_bridge so
    # BF's gyro+accel (MSP_RAW_IMU) reflect the motion. Translation drives accel;
    # the coupled roll/pitch rocking drives the gyro.
    shk = parser.add_argument_group("Shake-table settings (shake_test world)")
    shk.add_argument("--shake", action="store_true",
                     help="Force the kinematic shake-table on any world "
                          "(the shake_test world enables it automatically)")
    shk.add_argument("--shake-gated", action="store_true",
                     help="Shake only while the reset switch is held high "
                          "(reset_world.py --shake-gate); holds still at spawn "
                          "otherwise. Great for observing onset/cessation transients.")
    shk.add_argument("--shake-amp-x", type=float, default=0.10,
                     help="Shake East translation amplitude in metres (default: 0.10)")
    shk.add_argument("--shake-amp-y", type=float, default=0.10,
                     help="Shake North translation amplitude in metres (default: 0.10)")
    shk.add_argument("--shake-amp-z", type=float, default=0.10,
                     help="Shake Up (vertical) translation amplitude in metres (default: 0.10)")
    shk.add_argument("--shake-rate-x", type=float, default=1.5,
                     help="Shake East translation rate in Hz (default: 1.5)")
    shk.add_argument("--shake-rate-y", type=float, default=1.5,
                     help="Shake North translation rate in Hz (default: 1.5)")
    shk.add_argument("--shake-rate-z", type=float, default=1.5,
                     help="Shake Up (vertical) translation rate in Hz (default: 1.5)")
    shk.add_argument("--shake-phase-x-deg", type=float, default=0.0,
                     help="Shake East phase offset in degrees (default: 0)")
    shk.add_argument("--shake-phase-y-deg", type=float, default=90.0,
                     help="Shake North phase offset in degrees (default: 90)")
    shk.add_argument("--shake-phase-z-deg", type=float, default=0.0,
                     help="Shake Up phase offset in degrees (default: 0)")
    shk.add_argument("--shake-roll-amp-deg", type=float, default=8.0,
                     help="Shake roll amplitude in degrees (default: 8)")
    shk.add_argument("--shake-pitch-amp-deg", type=float, default=8.0,
                     help="Shake pitch amplitude in degrees (default: 8)")
    shk.add_argument("--shake-yaw-amp-deg", type=float, default=8.0,
                     help="Shake yaw amplitude in degrees (drives gyro Z; default: 8)")
    shk.add_argument("--shake-rate-roll", type=float, default=1.5,
                     help="Shake roll rate in Hz (default: 1.5)")
    shk.add_argument("--shake-rate-pitch", type=float, default=1.5,
                     help="Shake pitch rate in Hz (default: 1.5)")
    shk.add_argument("--shake-rate-yaw", type=float, default=1.5,
                     help="Shake yaw rate in Hz (default: 1.5)")
    shk.add_argument("--shake-phase-roll-deg", type=float, default=90.0,
                     help="Shake roll phase offset in degrees (default: 90)")
    shk.add_argument("--shake-phase-pitch-deg", type=float, default=0.0,
                     help="Shake pitch phase offset in degrees (default: 0)")
    shk.add_argument("--shake-phase-yaw-deg", type=float, default=0.0,
                     help="Shake yaw phase offset in degrees (default: 0)")

    args = parser.parse_args()

    # Resolve backward-compat world aliases (e.g. balloon_test → windy_target).
    args.world = WORLD_ALIASES.get(args.world, args.world)

    # Resolve the effective target drone per world: a world may FORCE a target
    # (shake_test = balloon) or set a DEFAULT_target (windy_target = balloon);
    # an explicit --target-drone overrides the default but not a forced target.
    _entry = WORLD_MAP.get(args.world, {})
    args.target_drone = (
        _entry.get("force_target")
        or args.target_drone
        or _entry.get("default_target")
        or DEFAULT_TARGET_DRONE
    )

    # shake_test: place the look-at balloon close and near drone height by
    # default (it's just something to look at while the drone shakes).
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
    world_file = world_entry["sim_world"]
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
        "Starting Gazebo%s: %s (vis-only, Simulink dynamics)",
        " (GUI)" if args.gazebo else " (headless)",
        os.path.basename(world_path),
    )
    pm.spawn(gz_args)
    time.sleep(8)

    # ── 3. bf_sim_bridge (Simulink dynamics — the only backend) ──
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
    if args.launcher_port:
        bridge_args += ["--launcher-port", str(args.launcher_port)]
    if args.home_lat is not None:
        bridge_args += ["--home-lat", str(args.home_lat)]
    if args.home_lon is not None:
        bridge_args += ["--home-lon", str(args.home_lon)]
    if args.home_alt is not None:
        bridge_args += ["--home-alt", str(args.home_alt)]
    # Kinematic shake-table: on for the shake_test world, or forced via --shake.
    if world_entry.get("shake") or getattr(args, "shake", False):
        bridge_args += ["--shake"]
        if getattr(args, "shake_gated", False):
            bridge_args += ["--shake-gated"]
        bridge_args += [
            "--shake-amp-x", str(args.shake_amp_x),
            "--shake-amp-y", str(args.shake_amp_y),
            "--shake-amp-z", str(args.shake_amp_z),
            "--shake-rate-x", str(args.shake_rate_x),
            "--shake-rate-y", str(args.shake_rate_y),
            "--shake-rate-z", str(args.shake_rate_z),
            "--shake-phase-x-deg", str(args.shake_phase_x_deg),
            "--shake-phase-y-deg", str(args.shake_phase_y_deg),
            "--shake-phase-z-deg", str(args.shake_phase_z_deg),
            "--shake-roll-amp-deg", str(args.shake_roll_amp_deg),
            "--shake-pitch-amp-deg", str(args.shake_pitch_amp_deg),
            "--shake-yaw-amp-deg", str(args.shake_yaw_amp_deg),
            "--shake-rate-roll", str(args.shake_rate_roll),
            "--shake-rate-pitch", str(args.shake_rate_pitch),
            "--shake-rate-yaw", str(args.shake_rate_yaw),
            "--shake-phase-roll-deg", str(args.shake_phase_roll_deg),
            "--shake-phase-pitch-deg", str(args.shake_phase_pitch_deg),
            "--shake-phase-yaw-deg", str(args.shake_phase_yaw_deg),
        ]
        log.info("Shake-table enabled (transl amp=[%.3g,%.3g,%.3g]m; "
                 "rot roll/pitch/yaw=[%.3g,%.3g,%.3g]deg)",
                 args.shake_amp_x, args.shake_amp_y, args.shake_amp_z,
                 args.shake_roll_amp_deg, args.shake_pitch_amp_deg,
                 args.shake_yaw_amp_deg)
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
    else:
        # Runtime guard: ensure BOOT_GRACE never blocks arming in SITL.
        send_bf_cli_commands(["set pwr_on_arm_grace = 0"])
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

        # ── FPV (pilot) camera — rectilinear, OSD always on. Only when enabled. ──
        if not args.pilot_cam:
            log.info("Pilot camera disabled (--no-pilot-cam) — skipping FPV/OSD feed")
        else:
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

            # SHM + RTSP are always active; the SDL2 window (--display) is added
            # ONLY when not headless. --no-display runs the bridge truly headless.
            bridge_cmd = [
                IMAGE_BRIDGE, topic,
                "--osd", "--msp-port", str(args.msp_port),
                "--cam-pitch", str(args.cam_pitch),
                "--out-width", str(args.fpv_cam_width),
                "--out-height", str(args.fpv_cam_height),
            ]
            bridge_cmd.append("--no-display" if args.no_display else "--display")

            # Per-world target proximity detection
            target_model = world_entry.get("target_model")
            target_link  = world_entry.get("target_link")
            target_bbox  = (world_vars.get("target_bbox")
                            if world_entry.get("target_drone")
                            else world_entry.get("target_bbox"))
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

        # ── Chase camera — rectilinear 3rd-person, optional (--chase-cam). ──
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
                    log.warning("Chase camera topic not found — skipping")
                else:
                    log.info("Found chase camera topic: %s", chase_topic)
            # Hardcoded 4:3 resolution, independent of FPV/tracker cam settings.
            if chase_topic:
                log.info("Starting chase camera bridge (no OSD)")
                chase_cmd = [IMAGE_BRIDGE, chase_topic, "--no-osd"]
                chase_cmd.extend(["--out-width", "640", "--out-height", "480"])
                chase_cmd.append("--no-display" if args.no_display else "--display")
                chase_bridge_proc = pm.spawn(
                    chase_cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )

        # ── Tracker WIDE / NARROW + thermal feeds (clean, no OSD). ──
        # Each is gated by its --…-cam toggle inside the shared helper.
        start_tracker_bridges(args, pm)

    # ── 6. Print connection info ──
    _print_status(
        args, is_simulink, topic, chase_topic,
        width, height, bridge_proc, chase_bridge_proc,
    )

    # ── 6b. Parametric trajectory drive thread (moving_target world) ──
    traj_thread = None
    traj_stop = threading.Event()
    if world_entry.get("trajectory_drive") and world_entry.get("target_model"):
        speed_ms = (args.target_speed if args.target_speed is not None else 18.0) / 3.6
        traj_thread = start_trajectory_thread(
            traj_stop,
            traj_type=getattr(args, "traj_type", None) or "oval",
            speed_ms=speed_ms,
            target_z=world_vars["target_z"],
            rotation_deg=args.traj_rotation_deg if args.traj_rotation_deg is not None else 0.0,
            offset_ew=args.traj_offset_ew if args.traj_offset_ew is not None else 0.0,
            offset_ns=args.traj_offset_ns if args.traj_offset_ns is not None else 0.0,
            oval_ew_len=args.oval_ew_length,
            oval_ns_len=args.oval_ns_length,
            corner_radius=args.corner_radius,
            start_pos=args.traj_start_pos if args.traj_start_pos is not None else 0.0,
            reverse=getattr(args, "traj_reverse", False),
        )

    # ── 6c. Wind (drift/harmonic) target thread (windy_target only) ──
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
            harmonic=getattr(args, "balloon_harmonic", False),
            amp_x=getattr(args, "balloon_amp_x", 10.0),
            amp_y=getattr(args, "balloon_amp_y", 10.0),
            rate_x=getattr(args, "balloon_rate_x", 0.05),
            rate_y=getattr(args, "balloon_rate_y", 0.05),
            phase_y_deg=getattr(args, "balloon_phase_y_deg", 90.0),
        )

    # ── 6d. Static target ground-truth publisher (collision_test only) ──
    # The target is a fixed <include>d model with no ExternalPosePlugin driving it,
    # so nothing else feeds sitl_redis_bridge's target:gps. Emit its known static
    # ENU position to the GT mirror port so the ground truth is populated.
    static_thread = None
    static_stop = threading.Event()
    if world_entry.get("static_target"):
        static_thread = start_static_target_thread(
            static_stop,
            x=world_vars["target_x"],
            y=world_vars["target_y"],
            z=world_vars["target_z"],
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

    if traj_thread:
        traj_stop.set()
        traj_thread.join(timeout=2)
    if wind_thread:
        wind_stop.set()
        wind_thread.join(timeout=2)
    if static_thread:
        static_stop.set()
        static_thread.join(timeout=2)

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
