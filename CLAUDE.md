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
