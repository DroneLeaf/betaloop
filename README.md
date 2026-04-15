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
| `--world <WORLD>` | World short name: `park_chase`, `patrol_park`, `collision_test`, `balloon_test` (default: `park_chase`) |
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
| `--target-altitude <m>` | Target altitude in metres (default: 50 park/patrol, 10 collision/balloon) |
| `--target-distance-x <m>` | Target X distance in metres (default: 30) |
| `--target-distance-y <m>` | Target Y distance in metres (default: 0) |
| `--target-speed <km/h>` | Park-chase orbit speed / patrol speed (default: 5.4 / 100) |
| `--target-orbit-radius <m>` | Park-chase orbit radius (default: 30) |
| `--patrol-length <m>` | Patrol-park track length (default: 2000) |
| `--sine-amplitude-xy <m>` | Patrol sine weave amplitude in Y (default: 0) |
| `--sine-period-xy <m>` | Patrol sine weave period along X (default: 200) |
| `--sine-amplitude-z <m>` | Patrol sine bob amplitude in Z (default: 0) |
| `--sine-period-z <m>` | Patrol sine bob period along X (default: 200) |
| `--wind-intensity <m>` | Balloon-test Lissajous drift amplitude (default: 2.0) |
| `--wind-randomness <σ>` | Balloon-test vertical bob amplitude (default: 1.0) |
| `--drift-speed <factor>` | Balloon-test Lissajous speed multiplier (default: 20.0) |
| `--hit-box-scale <factor>` | OSD hit-box scale multiplier (default: 1.0) |

### Quick start

```bash
# Park chase (default)
python3 ~/betaflight-docker/betaloop/start.py --gazebo

# Collision test (upward-facing camera)
python3 ~/betaflight-docker/betaloop/start.py --world collision_test --cam-pitch -90 --gazebo

# Patrol park
python3 ~/betaflight-docker/betaloop/start.py --world patrol_park --gazebo

# Balloon test with stochastic wind
python3 ~/betaflight-docker/betaloop/start.py --world balloon_test --gazebo

# Iris FPV
python3 ~/betaflight-docker/betaloop/start.py --drone iris --gazebo --chase-cam

# Simulink dynamics backend
python3 ~/betaflight-docker/betaloop/start.py --world park_chase --physics simulink --gazebo --chase-cam

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
