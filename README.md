# Betaloop

Simulation launcher and orchestrator for
[Betaflight](https://github.com/betaflight/betaflight) SITL +
[Gazebo Harmonic](https://gazebosim.org/). Manages the full stack: Gazebo,
Betaflight SITL, image bridges, SDL2 video display, and the virtual radio.

## Features

1. Uses real Betaflight firmware (SITL target)
2. FPV camera with OSD overlay (battery, attitude, flight mode, timer)
3. Chase camera (3rd-person SDL2 window)
4. Zero-latency SDL2 display via `gz_image_bridge --display`
5. Optional POSIX shared memory output (`--shm`) for external trackers

## Launchers

| Script | World | Description |
|---|---|---|
| `start_fpv.py` | Iris FPV demo | Lightweight quad with obstacles and gate |
| `start_rocket_drone_fpv.py` | Rocket drone | Heavy quad chasing an orbiting target |
| `start_rocket_drone_collision.py` | Collision test | Heavy quad with static target above |
| `start_rocket_drone_fpv_park.py` | Rocket drone park | Heavy quad over park terrain with Shahed target |
| `start_simulink_park.py` | Simulink rocket drone park | Simulink dynamics over park terrain with Shahed target |

### Quick start

```bash
# Iris FPV (SDL2 display + shared memory)
python3 start_fpv.py --display --shm --gazebo --chase-cam --osd

# Rocket drone
python3 start_rocket_drone_fpv.py --display --shm --gazebo --chase-cam --osd

# Simulink dynamics backend
python3 start_simulink.py --display --shm
```

### Key flags

| Flag | Effect |
|---|---|
| `--gazebo` | Show the Gazebo GUI (default: headless) |
| `--chase-cam` | Also display the chase camera (3rd-person SDL2 window) |
| `--osd` | Enable Betaflight OSD telemetry overlay |
| `--display` | SDL2 direct display in gz_image_bridge (default: enabled) |
| `--shm` | Expose frames via POSIX shared memory at /dev/shm |

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
