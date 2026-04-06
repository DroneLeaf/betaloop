# Betaloop

Simulation launcher and orchestrator for
[Betaflight](https://github.com/betaflight/betaflight) SITL +
[Gazebo Harmonic](https://gazebosim.org/). Manages the full stack: Gazebo,
Betaflight SITL, image bridges, SDL2 video display, and the virtual radio.

## Features

1. Uses real Betaflight firmware (SITL target)
2. FPV camera with OSD overlay (battery, attitude, flight mode, timer) — always enabled
3. Chase camera (3rd-person SDL2 window)
4. Zero-latency SDL2 display via `gz_image_bridge --display`
5. POSIX shared memory output for external trackers (always active, clean + OSD)

## Launcher

All worlds are launched via a single unified `start.py` with CLI arguments:

| Argument | Purpose |
|---|---|
| `--world <file>` | World SDF/world file (from `aeroloop_gazebo/worlds/`) |
| `--physics {gazebo,simulink}` | Physics backend (default: gazebo) |
| `--gazebo` | Show the Gazebo GUI (default: headless) |
| `--chase-cam` | Also display the chase camera (3rd-person SDL2 window) |
| `--cam-pitch <deg>` | FPV camera pitch in degrees (default: -80). Patches the model SDF and OSD crosshair. |
| `--no-video` | Skip video pipeline |

### Quick start

```bash
# Iris FPV
python3 start.py --world betaloop_iris_betaflight_demo_harmonic.sdf --gazebo --chase-cam

# Rocket drone
python3 start.py --world rocket_drone.world --gazebo --chase-cam

# Collision test (upward-facing camera)
python3 start.py --world rocket_drone_collision_test.world --cam-pitch -90 --gazebo

# Simulink dynamics backend
python3 start.py --world rocket_drone_vis.sdf --physics simulink --gazebo --chase-cam
```

### Key flags

| Flag | Effect |
|---|---|
| `--gazebo` | Show the Gazebo GUI (default: headless) |
| `--chase-cam` | Also display the chase camera (3rd-person SDL2 window) |
| `--display` | SDL2 direct display in gz_image_bridge (default: enabled) |
| `--physics simulink` | Use Simulink dynamics backend (bf_sim_bridge) |

## Motor Mapping

Gazebo:
- Rotor 0 = Front Right
- Rotor 1 = Back Left
- Rotor 2 = Front Left
- Rotor 3 = Back Right

Standard Betaflight Mixer:
- Rotor 0 = Back Right
- Rotor 1 = Front Right
- Rotor 2 = Back Left
- Rotor 3 = Front Left
