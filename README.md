# Betaloop

Simulation launcher and orchestrator for
[Betaflight](https://github.com/betaflight/betaflight) SITL +
[Gazebo Harmonic](https://gazebosim.org/). Manages the full stack: Gazebo,
Betaflight SITL, image bridges, ffmpeg video pipelines, and the virtual radio.

## Features

1. Uses real Betaflight firmware (SITL target)
2. FPV camera with OSD overlay (battery, attitude, flight mode, timer)
3. Chase camera (3rd-person view)
4. Low-latency `--raw` mode (bypasses ffmpeg, pipes directly to ffplay)
5. NVENC hardware encoding when an NVIDIA GPU is available
6. UDP, TCP, and RTSP streaming modes

## Launchers

| Script | World | Description |
|---|---|---|
| `start_fpv.py` | Iris FPV demo | Lightweight quad with obstacles and gate |
| `start_rocket_drone_fpv.py` | Rocket drone | Heavy quad chasing an orbiting target |
| `start_rocket_drone_collision.py` | Collision test | Heavy quad with static target above |

### Quick start

```bash
# Iris FPV (lowest latency — raw mode)
python3 start_fpv.py --gazebo --chase-cam --osd --raw

# Rocket drone (encoded stream)
python3 start_rocket_drone_fpv.py --gazebo --chase-cam --osd

# Watch encoded streams in a separate terminal:
ffplay -probesize 32 -analyzeduration 0 -fflags nobuffer -flags low_delay -framedrop -sync ext udp://@:8554
```

### Key flags

| Flag | Effect |
|---|---|
| `--gazebo` | Show the Gazebo GUI (default: headless) |
| `--chase-cam` | Also stream the chase camera on port 8555 |
| `--osd` | Enable Betaflight OSD telemetry overlay |
| `--raw` | Bypass ffmpeg — pipe raw frames to ffplay (lowest latency) |
| `--rtsp` | Use RTSP streaming via mediamtx |
| `--fps N` | Output FPS (default: 30) |

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
