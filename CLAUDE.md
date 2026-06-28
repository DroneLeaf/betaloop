# CLAUDE.md — Agent Handover (`betaloop/`)

Python launchers for the drone SITL stacks. Two entry points share one utility
module so BF and PX4 stay in feature parity.

## Files

| File | Purpose |
|------|---------|
| `start.py` | Betaflight (BF) stack launcher — BF SITL + `bf_sim_bridge` + Gazebo + OSD |
| `start_px4.py` | PX4 stack launcher (host side) — `px4_sim_bridge` + Gazebo + MAVLink OSD |
| `common.py` | **Shared** utilities: Gazebo env, process mgmt, camera topics, image-bridge metadata, cleanup, and **target trajectory threads** |

**Rule of thumb:** any logic needed by both stacks lives in `common.py`. Do not
duplicate it across `start.py` / `start_px4.py`.

## WORLD_MAP

Each launcher has a `WORLD_MAP` dict mapping a short world name → config:
`sim_world` (SDF file), `gz_name`, and optional target fields:
- `target_model` — passes `--target-model` to `gz_image_bridge` so `onPoseV()`
  tracks the target (required for proximity OSD + tracking).
- `orbit_drive` → `start_orbit_thread` (park_chase)
- `patrol_joint` → `start_patrol_thread` (patrol_park)
- `balloon_wind` → `start_balloon_thread` (balloon_test)

## Target trajectory threads (in `common.py`)

All send a 72-byte `VisualPosePacket` (`struct "<Qd3d4d"`: seq, t, pos[3], quat[4])
at ~60 Hz to an `ExternalPosePlugin` UDP port:

| Function | UDP port | Motion |
|----------|----------|--------|
| `start_orbit_thread` | 9017 reset / `TARGET_UDP_PORT` | circular orbit |
| `start_patrol_thread` | — | back-and-forth + optional sine |
| `start_balloon_thread` | 9014 (`BALLOON_UDP_PORT`) | smooth Lissajous drift |

## Recent Work

### MAVLink OSD wiring (PX4)
`start_px4.py` injects `--mavlink-port` (default **14560**) and OSD args into
`start_fpv_bridge`; `gz_image_bridge` parses MAVLink over UDP. `common.start_fpv_bridge`
supports `osd_args` and drains the bridge's stderr via a daemon thread (see fix below).

### Balloon target parity (BF ↔ PX4)
Previously the balloon target was static in the PX4 stack. Root cause:
`start_px4.py`'s `WORLD_MAP["balloon_test"]` was missing `target_model` and
`balloon_wind`. Fixed by:
1. Adding `"target_model": "balloon_target"` + `"balloon_wind": True` to PX4 WORLD_MAP.
2. Adding `--wind-intensity`, `--wind-randomness`, `--drift-speed` CLI args to `start_px4.py`.
3. **Centralizing** the Lissajous drift loop into `common.start_balloon_thread()`
   (previously an inline `_wind_loop` closure in `start.py`). Both launchers now call it.

The balloon model is `balloon_target` in `rocket_drone_balloon_test_vis.sdf`, driven
by `ExternalPosePlugin` on UDP **9014**.

### stderr pipe deadlock fix
The parent launcher opened the FPV bridge with `stderr=PIPE` but never drained it
after reading IMGMETA. `gz_image_bridge`'s MAVLink thread blocked on `fprintf`
(stderr 64KB pipe full) while holding `g_telem_mutex`, deadlocking the render loop
→ cameras froze on disarm/land. Fix: drain stderr in a daemon thread in `common.py`
(and removed per-packet debug prints in the C++ side).

### Intermittent `ARM denied BOOT GRACE` (BF stack)
Symptom: sometimes arming is blocked by `BOOT GRACE` and never clears; other
times it's never seen. Two compounding causes:

1. **SITL sim-time clock.** BF's `millis()` is driven by `fdm_packet`
   timestamps from `bf_sim_bridge`. The grace check is
   `millis() >= pwr_on_arm_grace * 1000`. With the BF default of 5 s, the
   condition can never be met in the window before the bridge starts feeding
   packets (sim time stays ~0) → grace appears stuck forever.
2. **Stale eeprom.** The setting `set pwr_on_arm_grace = 0` lives in
   `configure_betaflight.py` COMMON_COMMANDS, but it only takes effect if
   eeprom.bin was actually (re)written with it. A flaky `--config` write left
   the old 5 s default in place → intermittent behavior.

**Runtime guard (in `start.py`):** `send_bf_cli_commands()` opens the BF CLI
(TCP 5761) right after the port comes up and sends `set pwr_on_arm_grace = 0`
every launch. This enforces grace=0 even if eeprom is stale. See
`send_bf_cli_commands()` and the call right after `wait_for_port(... 5761 ...)`.

See also the root `CLAUDE.md` and `configure_betaflight.py` notes on the
non-destructive eeprom write.

## Quick test

```bash
python3 start_px4.py --world balloon_test --gazebo --chase-cam
python3 start.py     --world balloon_test --gazebo --chase-cam
```

Import sanity check:
```bash
python3 -c "import start, start_px4; from common import start_balloon_thread"
```

## Adding a new world (checklist)

A new world short-name must be registered in **several** maps or it half-works:
1. Both launchers' `WORLD_MAP` (`start.py` **and** `start_px4.py`) — keep parity.
2. `reset_world.py`'s separate `WORLD_MAP` (short name → Gazebo `<world>` name),
   or SPACE/RC resets silently no-op for that world.
3. `leaf-sim-ui/ui/screens/settings.py` `WORLDS`, plus per-world widget
   visibility + `get_settings`/`restore` branches, `backend/supervisor.py`
   flag mapping, and `ui/i18n.py` (en + ar). See `leaf-sim-ui/CLAUDE.md`.

Target-trajectory threads (orbit/patrol/balloon) are daemon threads stopped via
a `threading.Event` and `join(timeout=…)` on shutdown — follow that pattern for
any new motion so Ctrl-C exits cleanly.

## Session Addendum (2026-06-13)

- No launcher code changes were required in `betaloop/`, but BF defaults consumed
   by the BF startup flow were updated upstream in `configure_betaflight.py`.
- ANGLE mode AUX mapping is now CH11 (AUX7), not CH10:
   `aux 1 1 6 1200 2100 0 0`.
- Thaqib profile defaults were synced to `F7_104_tuning.txt` profile 0/rateprofile 0.
   If a transmitter still toggles CH10 for ANGLE, mode engagement will no longer
   match expectations; use CH11 for ANGLE with current defaults.

## Session Addendum (2026-06-22)

- `--physics` retained but simulink-only (`choices=["simulink"]`); physics-world
  rendering removed from `_render_all_templates`; `bf_sim_bridge` always runs.
- Cameras: per-feed BooleanOptional toggles (`--pilot-cam` / `--tracker-wide-cam`
  / `--tracker-narrow-cam` / `--thermal-cam` / `--chase-cam`). Wide tracker
  renamed `fpv_tracker_cam`→`fpv_tracker_wide_cam`; narrow geometry
  `--tracker-narrow-*`. Clean tracker/thermal feeds spawn via shared
  `common.start_tracker_bridges()`. `compute_model_vars` gained per-camera enable
  flags + `tracker_wide_*` / `tracker_narrow_*` vars.
- Targets: `common.TARGET_REFS` (shahed/stingjet);
  `compute_world_vars(target_drone=)` emits `target_mesh_uri` /
  `target_model_uri` / `target_visual_pose` / `target_scale`. `--target-drone` on
  both launchers; WORLD_MAP marks drone-target worlds `"target_drone": True`
  (proximity bbox then comes from `TARGET_REFS`, not WORLD_MAP).

## Session Addendum (2026-06-23) — per-camera fisheye toggle

- `--tracker-wide-fisheye` / `--tracker-narrow-fisheye` / `--thermal-fisheye`
  (BooleanOptional on both `start.py` and `start_px4.py`) pick fisheye
  (wideanglecamera) vs rectilinear per feed; defaults wide=fisheye,
  narrow=rectilinear, thermal=rectilinear. `compute_model_vars` gained
  `tracker_wide_fisheye` / `tracker_narrow_fisheye` / `thermal_fisheye`, emitted to
  the model templates. `launch.sh`/`launch_px4.sh` forward the flags via their
  catch-all `*)` case (no explicit case needed).

## Session Addendum (2026-06-23) — utility camera

- `--utility-cam` (+ `--utility-cam-pitch/-roll`, `--utility-hfov/-vfov`,
  `--utility-cam-width/-height/-fps`, `--utility-fisheye`, and BF-only
  `--utility-rtsp[...]`) add a 4th clean tracker-style feed `fpv_utility_cam`, cloned
  from the wide tracker. `compute_model_vars` gained `utility_*` params/vars;
  `start_tracker_bridges` feeds tuple got
  `("utility_cam","fpv_utility_cam",[],"utility","Utility")` → SHM + (BF) RTSP.
  start_px4.py mirrors enable+geometry+fisheye (no per-feed RTSP, like the other px4
  feeds). Defaults seeded from wide (fisheye on, 114.6/98.9), enable default off.

## Session Addendum (2026-06-23) — parameterised target scale (all targets)

- `TARGET_REFS` entries gained `base_scale` (scale at which `bbox` was measured) +
  `default_scale` (applied default; shahed 1.0, stingjet 0.1).
  `compute_world_vars(target_scale=)` applies to **any** target: effective =
  override else `default_scale`; scales `target_bbox` by `scale/base_scale`; emits
  `target_model_name`. `render_vis_templates` also renders
  `models/<target>/model.sdf.j2` (both shahed + stingjet) with `target_scale`.
  `--target-scale` on start.py + start_px4.py; both use the scaled
  `world_vars["target_bbox"]` for `--target-bbox`.

## Session Addendum (2026-06-23) — parametric moving_target trajectory

- `common.py`: `start_orbit_thread`/`start_patrol_thread` superseded by
  `start_trajectory_thread` (60 Hz, `<Qd3d4d` to 9016 + mirror 9018, reset on 9017
  → s=0). Pure helpers `_traj_local_path` (oval/circle/line, path(0)=(0,0), +X start),
  `trajectory_sample` (world E/N/yaw after rotation+offset), `trajectory_start_pose`
  (s=0, for the world spawn). `TRAJ_TYPES = (oval,circle,line)`.
- `compute_world_vars` gained `traj_type/traj_rotation_deg/traj_offset_ew/ns/
  oval_ew_len/oval_ns_len/corner_radius/circle_radius/line_length/player_heading_deg`
  (all None-coalesced) → emits `target_spawn_x/y/yaw` + `player_heading_rad`.
- WORLD_MAP collapsed to `moving_target` (`trajectory_drive: True`,
  target_model `moving_target`); `park_chase`/`patrol_park` removed from both
  start.py + start_px4.py + reset_world.py. Dispatch is a single
  `if trajectory_drive: start_trajectory_thread(...)`. `DEFAULT_WORLD=moving_target`
  (start.py). The legacy `--target-orbit-*`/`--patrol-*` args remain defined but unused.

## Session Addendum (2026-06-24) — RTSP CRF flag

- `_append_rtsp` gained a `crf` param (after `bitrate`) → emits `--stream-crf` to the
  bridge. New per-feed CLI args `--tracker-wide-rtsp-crf` / `--tracker-narrow-rtsp-crf`
  / `--thermal-rtsp-crf` / `--utility-rtsp-crf` (int, default **23**; 0 = off/ABR).
  RTSP stays BF-only (px4 passes no `*_rtsp` url → `_append_rtsp` no-ops). Fixes 480p+
  RTSP macroblocking (bitrate starvation, not CPU). See `aeroloop_gazebo/CLAUDE.md`.

## Session Addendum (2026-06-24) — per-camera fisheye lens intrinsics

- `common.py`: `compute_model_vars` gained 16 lens params (`<cam>_lens_c1/c2/c3/fun`
  for wide/narrow/thermal/utility; defaults 1.05/4.0/0.0/"tan") → emitted to the model
  templates (fun sanitized via `sanitize_lens_fun`). New module consts `LENS_FUNS`
  (tan/sin/id), `LENS_DEFAULT`, `LENS_PRESETS` (stereographic/equidistant/equisolid/
  orthographic). Shared helpers `add_lens_args(group)` (adds `--<cam>-lens-c1/c2/c3/fun`
  to a parser) and `lens_kwargs_from_args(args)` (collects them) keep both launchers DRY.
- `start.py` + `start_px4.py` both call `add_lens_args(drn/sim)` and pass
  `**lens_kwargs_from_args(args)` into `compute_model_vars`. Lens only affects fisheye
  feeds (it's inside the `{%- if <cam>_fisheye %}` block). See `aeroloop_gazebo/CLAUDE.md`.

## Session Addendum (2026-06-28) — balloon harmonic-oscillation mode

- `common.start_balloon_thread` gained a **harmonic** mode (default off → existing
  Lissajous drift preserved). Params `harmonic`, `amp_x`, `amp_y`, `rate_x`, `rate_y`
  (Hz), `phase_y_deg`. Motion: `x = cx + amp_x·sin(2π·rate_x·t)`,
  `y = cy + amp_y·sin(2π·rate_y·t + phase_y)`, `z = cz` (constant). The **centre is
  the target position** (`mean_x/y/z` ← `--target-distance-x/-y` + `--target-altitude`),
  so no new centre args. Equal amp/rate + phase 90° = circle (verified: traces a circle
  of radius=amp about the centre to machine precision).
- CLI on **both** launchers: `--balloon-harmonic` (store_true) + `--balloon-amp-x/-y`,
  `--balloon-rate-x/-y`, `--balloon-phase-y-deg`. start.py reads centre from
  `world_vars["target_*"]`; start_px4.py from `args.target_distance_*`/`target_altitude`.

## Session Addendum (2026-06-28) — balloon target type + windy_target rename

- `common.TARGET_REFS` gained **`balloon`** (a primitive: `primitive:"sphere"`,
  `radius`, `color`; no mesh). `compute_world_vars` now also emits
  `target_primitive` / `target_radius` / `target_color`. `--target-drone` choices
  include `balloon`; its default scale is 1.0.
- World **`balloon_test` → `windy_target`** in both `WORLD_MAP`s. New keys on the
  entry: `target_drone:True`, `default_target:"balloon"` (keeps `balloon_wind:True`).
  `shake_test` gained `target_model:"shake_target"`, `target_drone:True`,
  `force_target:"balloon"`, `static_target:True`.
- New `WORLD_ALIASES = {"balloon_test":"windy_target"}`; `parse_args` (both
  launchers) resolves the alias, then resolves the **effective target**:
  `force_target` > explicit `--target-drone` > `default_target` > DEFAULT. So
  shake_test is always balloon, windy_target defaults to balloon, others shahed.
  `--target-drone` default is now `None`. shake_test also defaults the look-at
  balloon to x=8, alt=2 when unset.
- The balloon (drift/harmonic) thread is unchanged — it now drives whatever
  target the windy_target world spawns (balloon sphere or a drifting mesh).
