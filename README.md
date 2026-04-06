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

All worlds are launched via a single unified `start.py` with CLI arguments
organised into three groups: **simulation**, **drone**, and **world** settings.

### Simulation settings

| Argument | Purpose |
|---|---|
| `--world <file>` | World SDF/world file (default: `rocket_drone_park_chase.world`) |
| `--physics {gazebo,simulink}` | Physics backend (default: gazebo) |
| `--gazebo` | Show the Gazebo GUI (default: headless) |
| `--chase-cam` | Also display the chase camera (3rd-person SDL2 window) |
| `--no-video` | Skip video pipeline |

### Drone settings

| Argument | Purpose |
|---|---|
| `--drone {rocket_drone,iris}` | Drone profile for parameter scaling (default: `rocket_drone`) |
| `--cam-pitch <deg>` | FPV camera pitch (default: -80). Patches model SDF and OSD crosshair. |
| `--ctw <ratio>` | Thrust-to-weight ratio — derives mass & inertia from reference calibration |
| `--standoff-height <m>` | Landing leg length in metres |
| `--linear-damping-x/y/z <val>` | ViscousDragPlugin per-axis linear damping |
| `--quadratic-damping-x/y/z <val>` | ViscousDragPlugin per-axis quadratic damping |
| `--angular-damping <val>` | ViscousDragPlugin angular damping |

### World settings

| Argument | Purpose |
|---|---|
| `--target-altitude <m>` | Collision-test target altitude (default: 20) |
| `--target-distance <m>` | Collision-test target horizontal distance (default: 20) |
| `--target-speed <rad/s>` | Park-chase orbit angular speed (default: 0.05) |

### Quick start

```bash
# Park chase (default)
python3 ~/betaflight-docker/betaloop/start.py --gazebo

# Collision test (upward-facing camera)
python3 ~/betaflight-docker/betaloop/start.py --world rocket_drone_collision_test.world --cam-pitch -90 --gazebo

# Iris FPV
python3 ~/betaflight-docker/betaloop/start.py --drone iris --gazebo --chase-cam

# Simulink dynamics backend
python3 ~/betaflight-docker/betaloop/start.py --world rocket_drone_park_chase_vis.sdf --physics simulink --gazebo --chase-cam

# Tune drone: higher CTW + more angular damping
python3 ~/betaflight-docker/betaloop/start.py --ctw 5 --angular-damping 0.05 --gazebo
```

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
