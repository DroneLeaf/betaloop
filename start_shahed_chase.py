#!/usr/bin/env python3
"""Shahed Drone Chase Camera Launcher — Gazebo + Camera → RTSP/TCP/UDP stream.

Starts Gazebo with the shahed_chase_park world and streams the Shahed drone's
chase camera feed.
"""

import argparse
import logging
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import fcntl
import glob

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("start_shahed_chase")

# ── Defaults ──────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)

def _default_path(env_var, repo_relative, docker_absolute):
    if os.environ.get(env_var):
        return os.environ[env_var]
    repo_path = os.path.join(_REPO_ROOT, repo_relative)
    if os.path.exists(repo_path):
        return repo_path
    return docker_absolute

AEROLOOP_HOME = _default_path("AEROLOOP_HOME", "aeroloop_gazebo", "/opt/aeroloop_gazebo")
TARGET_WORLD = "shahed_chase_park.world"
IMAGE_BRIDGE = os.path.join(AEROLOOP_HOME, "plugins", "build", "gz_image_bridge")
STREAM_PORT = 8554
TOPIC_MODEL_HINT_DEFAULT = "shahed_drone"
MAX_STREAM_RESTARTS = 5

_LAST_LOG_TIMES = {}


def _log_rate_limited(key, interval_s, level, message, *args):
    """Log recurring status lines no more than once per interval."""
    now = time.time()
    last = _LAST_LOG_TIMES.get(key, 0.0)
    if now - last < interval_s:
        return
    _LAST_LOG_TIMES[key] = now
    getattr(log, level)(message, *args)

def _is_container():
    return os.path.exists("/.dockerenv") or os.environ.get("container") == "docker"


class ProcessManager:
    def __init__(self):
        self.procs = []

    def spawn(self, args, **kwargs):
        log.info("Starting: %s", " ".join(args[:4]) + (" ..." if len(args) > 4 else ""))
        p = subprocess.Popen(args, **kwargs)
        self.procs.append(p)
        return p

    def shutdown(self):
        log.info("Shutting down %d processes …", len(self.procs))
        for p in reversed(self.procs):
            if p.poll() is None:
                p.terminate()
        for p in reversed(self.procs):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()


def setup_gazebo_env():
    def _prepend(var, *paths):
        existing = os.environ.get(var, "")
        os.environ[var] = os.pathsep.join(list(paths) + [existing])

    models = os.path.join(AEROLOOP_HOME, "models")
    plugins = os.path.join(AEROLOOP_HOME, "plugins", "build")
    worlds = os.path.join(AEROLOOP_HOME, "worlds")

    _prepend("SDF_PATH", models, "/usr/share/gz/gz-sim8/models")
    _prepend("GZ_SIM_RESOURCE_PATH", worlds, "/usr/share/gz/gz-sim8")
    _prepend("GZ_SIM_SYSTEM_PLUGIN_PATH", plugins, "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins")
    _prepend("LD_LIBRARY_PATH", "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins")
    
    # Add GStreamer plugin path for gz_image_bridge
    _prepend("GST_PLUGIN_PATH", "/usr/lib/x86_64-linux-gnu/gstreamer-1.0", "/usr/local/lib/gstreamer-1.0")

    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


def wait_for_gazebo_ready(timeout=90):
    """Wait for Gazebo to be responsive and initialized."""
    log.info("Waiting for Gazebo to be ready (timeout: %ds)...", timeout)
    start_time = time.time()
    attempt = 0
    last_log_time = start_time
    
    while time.time() - start_time < timeout:
        attempt += 1
        try:
            # Use a longer timeout for gz commands during startup
            result = subprocess.run(
                ["gz", "topic", "-l"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=5  # Increased from 2 to 5 seconds
            )
            
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                # Check for key topics that indicate world is loaded
                # Just check if we have any topics at all - Gazebo is responding
                if len(topics) > 5:  # More lenient check
                    # Try to verify model is loaded
                    try:
                        model_result = subprocess.run(
                            ["timeout", "5", "gz", "model", "--list"],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            text=True,
                            timeout=6
                        )
                        if "shahed_drone" in model_result.stdout:
                            log.info("Gazebo is ready! Models loaded and responsive (%d attempts)", attempt)
                            return True
                    except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as e:
                        log.debug("Model list check failed during startup: %s", e)
                    
                elapsed = time.time() - start_time
                if elapsed - (last_log_time - start_time) >= 10:  # Log every 10 seconds
                    log.info("Gazebo initializing... (%d topics found, %.1fs elapsed)", len(topics), elapsed)
                    last_log_time = time.time()
            
        except subprocess.TimeoutExpired:
            elapsed = time.time() - start_time
            if elapsed - (last_log_time - start_time) >= 10:
                log.info("Gazebo still initializing (timeout in subprocess, %.1fs elapsed)", elapsed)
                last_log_time = time.time()
        except OSError as e:
            log.debug("Gazebo readiness probe failed: %s", e)
        
        time.sleep(1)  # Increased from 0.5 to 1 second
    
    log.error("Gazebo did not become ready within %ds", timeout)
    return False


def cleanup_before_start():
    """Clean up any existing processes from previous runs."""
    log.info("Cleaning up from previous runs...")

    cleanup_cmds = [
        ("existing Gazebo processes", "pkill -9 -f 'gz sim' 2>/dev/null || true"),
        ("existing ffmpeg processes", "pkill -9 ffmpeg 2>/dev/null || true"),
        ("existing image bridge processes", "pkill -9 -f 'gz_image_bridge' 2>/dev/null || true"),
        ("existing Betaflight SITL processes", "pkill -9 -f 'betaflight_SITL.elf' 2>/dev/null || true"),
        ("existing MSP Virtual Radio processes", "pkill -9 -f 'msp_virtualradio/index.js' 2>/dev/null || true"),
        ("existing mediamtx processes", "pkill -9 -f mediamtx 2>/dev/null || true"),
        ("existing Xvfb processes", "pkill -9 Xvfb 2>/dev/null || true"),
    ]

    for label, cmd in cleanup_cmds:
        try:
            subprocess.run(["bash", "-c", cmd], capture_output=True, timeout=5)
            log.debug("Stopped %s", label)
        except (subprocess.TimeoutExpired, OSError) as e:
            log.debug("Cleanup command failed for %s: %s", label, e)
        time.sleep(0.2)

    log.info("Cleanup complete")


def has_nvidia_gpu():
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=5,
        )
        return result.returncode == 0 and result.stdout.strip() != ""
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def _fix_dri_permissions():
    render_nodes = glob.glob("/dev/dri/renderD*") + glob.glob("/dev/dri/card*")
    for node in render_nodes:
        if os.access(node, os.R_OK | os.W_OK):
            continue
        log.info("Fixing permissions on %s", node)
        subprocess.run(["sudo", "chmod", "666", node], capture_output=True)


def list_camera_topics(name_hint=None):
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True, text=True, timeout=10,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []

    topics = []
    for line in result.stdout.strip().splitlines():
        line = line.strip()
        if not line.endswith("/image"):
            continue
        if name_hint and name_hint not in line:
            continue
        topics.append(line)
    return sorted(set(topics))


def discover_camera_topic(name_hint="chase_cam", timeout=30, model_hint=None):
    deadline = time.time() + timeout
    while time.time() < deadline:
        topics = list_camera_topics(name_hint=name_hint)
        if topics:
            if model_hint:
                preferred = [topic for topic in topics if model_hint in topic]
                if preferred:
                    return preferred[0]
            return topics[0]
        time.sleep(2)
    return None


def read_image_meta(proc, timeout=30):
    import select
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        ready, _, _ = select.select([proc.stderr], [], [], 1.0)
        if ready:
            try:
                chunk = proc.stderr.read(4096)
            except BlockingIOError:
                chunk = None
            if chunk is None or len(chunk) == 0:
                if proc.poll() is not None:
                    break
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode("utf-8", errors="replace").strip()
                if line.startswith("IMGMETA "):
                    parts = line.split()
                    return int(parts[1]), int(parts[2]), parts[3]
        if proc.poll() is not None:
            break
    return None, None, None


def _latest_captured_frame_index(frames_dir):
    """Return highest frame index from frame_XXXXXX.jpg files, or None if none exist."""
    max_idx = None
    for frame_path in glob.glob(os.path.join(frames_dir, "frame_*.jpg")):
        name = os.path.basename(frame_path)
        try:
            idx = int(name.split("_")[1].split(".")[0])
        except (IndexError, ValueError):
            continue
        if max_idx is None or idx > max_idx:
            max_idx = idx
    return max_idx


def _sorted_frame_paths(frames_dir):
    frame_paths = glob.glob(os.path.join(frames_dir, "frame_*.jpg"))

    def _frame_idx(path):
        name = os.path.basename(path)
        try:
            return int(name.split("_")[1].split(".")[0])
        except (IndexError, ValueError):
            return float("inf")

    return sorted(frame_paths, key=_frame_idx)


def _select_frames_until_time(frames_dir, stop_time=None, trim_back_frames=0):
    """Select frame paths up to stop_time (mtime <= stop_time), then trim back N frames."""
    all_frames = _sorted_frame_paths(frames_dir)
    if not all_frames:
        return []

    if stop_time is None:
        selected = list(all_frames)
    else:
        selected = [p for p in all_frames if os.path.getmtime(p) <= stop_time]
        if not selected:
            # Fallback in case of coarse mtime timing edge case.
            selected = [all_frames[0]]

    trim_n = max(0, int(trim_back_frames or 0))
    if trim_n > 0 and len(selected) > trim_n:
        selected = selected[:-trim_n]

    return selected


def _frame_index_from_path(frame_path):
    name = os.path.basename(frame_path)
    try:
        return int(name.split("_")[1].split(".")[0])
    except (IndexError, ValueError):
        return None


def _monitor_pose_and_stop_recording(
    png_proc,
    return_threshold=3,
    confirmation_frames=1,
    frames_dir=None,
    stop_info=None,
):
    """Monitors SHAHED DRONE geranium_link pose via /world/default/pose/info topic.

    Recording starts immediately (as soon as first position received).
    Stops INSTANTLY when drone returns to home within threshold,
    but only after it first leaves the home zone.

    Parameters:
      return_threshold: Distance in meters to detect return to home (default 0.33m)
      away_threshold: Computed as max(return_threshold * 1.5, return_threshold + 0.5)
                      to prevent immediate false stop near takeoff.
      confirmation_frames: Deprecated — termination is now instant (kept for API compatibility)

    Parses protobuf pose message format to extract geranium_link position (actual drone body).
    Format:
        pose {
          name: "geranium_link"
          position {
            x: ...
            y: ...
            z: ...
          }
        }
    """
    home_pos = None
    message_count = 0
    pose_proc = None
    last_logged_dist = None
    recording_start_time = None  # Time when first position received (recording began)
    has_left_home_zone = False
    away_threshold = max(return_threshold * 1.5, return_threshold + 0.5)

    log.info("Pose monitor started for geranium_link")
    log.info("Return threshold=%.2fm, arm threshold=%.2fm", return_threshold, away_threshold)

    import re

    try:
        # Subscribe to world pose info that broadcasts all entities
        pose_proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", "/world/default/pose/info"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1  # Line-buffered for real-time data
        )

        buffer = []

        for line in pose_proc.stdout:
            line = line.rstrip()  # Keep leading whitespace for structure

            # Accumulate lines into buffer
            buffer.append(line)

            # Check if we've hit the end of a pose block (closing brace at start of line)
            if line.strip() == "}":
                full_text = "\n".join(buffer)

                # Check if this pose block is for geranium_link
                if 'name: "geranium_link"' in full_text:
                    try:
                        # Extract x, y, z coordinates from position block
                        x_match = re.search(r'x:\s*([-\d.eE+]+)', full_text)
                        y_match = re.search(r'y:\s*([-\d.eE+]+)', full_text)
                        z_match = re.search(r'z:\s*([-\d.eE+]+)', full_text)

                        if x_match and y_match and z_match:
                            x = float(x_match.group(1))
                            y = float(y_match.group(1))
                            z = float(z_match.group(1))

                            message_count += 1
                            current_pos = (x, y, z)

                            if home_pos is None:
                                # First pose: set home position and start time, recording active now
                                home_pos = current_pos
                                recording_start_time = time.time()
                                log.info("HOME POSITION SET: (%.2f, %.2f, %.2f) - RECORDING STARTED", *home_pos)
                            else:
                                dist = math.sqrt((x-home_pos[0])**2 + (y-home_pos[1])**2 + (z-home_pos[2])**2)
                                elapsed = time.time() - recording_start_time

                                # Log distance changes (only when change is significant)
                                if last_logged_dist is None or abs(dist - last_logged_dist) >= 0.10:
                                    _log_rate_limited(
                                        "pose_distance",
                                        3,
                                        "info",
                                        "Distance from home: %.3fm (%.1fs)",
                                        dist,
                                        elapsed,
                                    )
                                    last_logged_dist = dist

                                if not has_left_home_zone and dist > away_threshold:
                                    has_left_home_zone = True
                                    log.info("Drone left home zone at %.3fm (return-stop armed)", dist)

                                # Check for return - INSTANT termination only after leaving home zone once
                                if has_left_home_zone and dist <= return_threshold:
                                    stop_frame = _latest_captured_frame_index(frames_dir) if frames_dir else None
                                    if stop_info is not None:
                                        stop_info["triggered"] = True
                                        stop_info["stop_frame"] = stop_frame
                                        stop_info["stop_time"] = time.time()
                                    log.info("Drone returned to home zone (%.3fm, %.1fs), stopping capture", dist, elapsed)
                                    if stop_frame is not None:
                                        log.debug("Marked stop frame index: %d", stop_frame)
                                    png_proc.terminate()
                                    try:
                                        png_proc.wait(timeout=5)
                                    except subprocess.TimeoutExpired:
                                        png_proc.kill()
                                    return

                    except (ValueError, AttributeError) as e:
                        log.debug("Parse error: %s", e)

                # Clear buffer after processing pose block
                buffer = []

            # Safety: Check if JPEG capture process died
            if png_proc.poll() is not None:
                log.info("JPEG capture process exited, pose monitor ending")
                return

        log.warning("Pose stream ended (received %d messages)", message_count)

    except Exception as e:
        log.error("Pose monitor error: %s", e, exc_info=True)
    finally:
        if pose_proc and pose_proc.poll() is None:
            pose_proc.terminate()
            try:
                pose_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pose_proc.kill()
        log.info("Pose monitor stopped (messages=%d)", message_count)


def _encode_pngs_to_mp4(frames_dir, output_file, fps=30, selected_frames=None):
    """Convert JPG frames to MP4 file, optionally using an explicit selected frame list."""
    jpg_count = len([f for f in os.listdir(frames_dir) if f.endswith(".jpg")])
    log.info("Encoding %d JPG frames to MP4: %s", jpg_count, output_file)
    try:
        if selected_frames:
            stop_frame = _frame_index_from_path(selected_frames[-1])
            if stop_frame is not None and stop_frame > 0:
                log.info("Encoding selected frame window via image sequence: 1..%d (%d selected)", stop_frame, len(selected_frames))
                ffmpeg_cmd = [
                    "ffmpeg", "-y",
                    "-framerate", str(fps),
                    "-start_number", "1",
                    "-i", os.path.join(frames_dir, "frame_%06d.jpg"),
                    "-frames:v", str(stop_frame),
                ]
            else:
                log.warning("Could not parse selected stop frame; falling back to full sequence encode")
                ffmpeg_cmd = [
                    "ffmpeg", "-y",
                    "-framerate", str(fps),
                    "-i", os.path.join(frames_dir, "frame_%06d.jpg"),
                ]
        else:
            ffmpeg_cmd = [
                "ffmpeg", "-y",
                "-framerate", str(fps),
                "-i", os.path.join(frames_dir, "frame_%06d.jpg"),
            ]

        ffmpeg_cmd += [
            "-pix_fmt", "yuv420p",
            "-c:v", "libx264", "-preset", "fast",
            "-crf", "20",
            output_file
        ]
        result = subprocess.run(ffmpeg_cmd, capture_output=True, timeout=300)
        if result.returncode == 0:
            log.info("MP4 encoding completed successfully: %s", output_file)
            return True
        else:
            log.error("ffmpeg encoding failed: %s", result.stderr.decode())
            return False
    except Exception as e:
        log.error("Error encoding MP4: %s", e)
        return False


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(description="Shahed Chase Camera Streaming Launcher")
    parser.add_argument("--world", default=TARGET_WORLD,
                        help=f"World SDF file (default: {TARGET_WORLD})")
    parser.add_argument("--port", type=int, default=STREAM_PORT,
                        help="Stream output port (default: 8554)")
    parser.add_argument("--rtsp", action="store_true",
                        help="Use RTSP streaming via mediamtx")
    parser.add_argument("--output", default=None,
                        help="Override output: 'udp' (default), 'tcp', 'rtsp', or 'file:<path>'")
    parser.add_argument("--gazebo", action="store_true",
                        help="Show the Gazebo GUI (default: headless)")
    parser.add_argument("--fps", type=int, default=30,
                        help="Output stream FPS (default: 30)")
    parser.add_argument("--raw", action="store_true",
                        help="Skip ffmpeg entirely, pipe raw frames directly into ffplay")
    parser.add_argument("--chase-topic", default=None,
                        help="Use this exact Gazebo image topic for chase-cam (skip auto-discovery)")
    parser.add_argument("--topic-model-hint", default=TOPIC_MODEL_HINT_DEFAULT,
                        help="Prefer topics that contain this model path segment")
    parser.add_argument("--record", default=None,
                        help="Optional: Output MP4 file path. Automatically stops when drone returns home.")
    parser.add_argument("--trim-back-frames", type=int, default=3,
                        help="Trim this many frames before stop trigger when encoding (default: 3)")
    return parser.parse_args()


def _resolve_output_mode(args):
    if args.output:
        return args.output
    if args.rtsp:
        return "rtsp"
    return "udp"


def _configure_runtime(args, pm):
    log.info("Setting up Gazebo environment")
    setup_gazebo_env()

    gpu_available = has_nvidia_gpu()
    in_container = _is_container()

    if gpu_available:
        log.info("NVIDIA GPU detected, using GPU rendering")
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
        nvidia_icd = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.isfile(nvidia_icd):
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_icd
        if in_container:
            os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
            _fix_dri_permissions()
    else:
        log.info("No GPU detected, using software rendering")
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    if args.gazebo:
        if not os.environ.get("DISPLAY"):
            log.error("No DISPLAY set, Gazebo GUI needs a display")
            sys.exit(1)
        os.environ.pop("__GLX_VENDOR_LIBRARY_NAME", None)
        log.info("Using display %s for Gazebo GUI", os.environ["DISPLAY"])
    elif not in_container and os.environ.get("DISPLAY"):
        log.info("Using native display %s", os.environ["DISPLAY"])
    else:
        for lockfile in ["/tmp/.X99-lock", "/tmp/.X11-unix/X99"]:
            try:
                os.remove(lockfile)
            except OSError:
                pass
        subprocess.run(["pkill", "-9", "Xvfb"], capture_output=True)
        time.sleep(0.3)

        log.info("Starting Xvfb virtual display :99")
        xvfb = pm.spawn(
            ["Xvfb", ":99", "-screen", "0", "1280x720x24", "-ac"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(1)
        if xvfb.poll() is not None:
            log.error("Xvfb failed to start, camera rendering requires a display")
            sys.exit(1)
        os.environ["DISPLAY"] = ":99"

    return gpu_available


def _start_optional_rtsp_server(output_mode, args, pm):
    if output_mode != "rtsp" or args.record:
        return
    mediamtx = "/usr/local/bin/mediamtx"
    if os.path.isfile(mediamtx):
        log.info("Starting mediamtx RTSP server")
        pm.spawn([mediamtx], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)


def _start_gazebo(args, pm):
    world_path = args.world
    if not os.path.isabs(world_path):
        world_path = os.path.join(AEROLOOP_HOME, "worlds", world_path)

    gz_args = ["gz", "sim"]
    if not args.gazebo:
        gz_args.append("-s")
    # Keep Gazebo output quiet: hide info/warning messages.
    gz_args.extend(["-r", "-v", "1", world_path])

    log.info("Starting Gazebo: %s", os.path.basename(world_path))
    pm.spawn(gz_args)
    if not wait_for_gazebo_ready(timeout=90):
        log.error("Gazebo failed to initialize")
        pm.shutdown()
        sys.exit(1)


def _resolve_chase_topic(args, pm):
    if args.chase_topic:
        log.info("Using explicit chase camera topic: %s", args.chase_topic)
        return args.chase_topic

    log.info("Discovering chase camera image topic")
    chase_candidates = list_camera_topics(name_hint="chase_cam")
    if chase_candidates:
        log.debug("Chase candidates: %s", ", ".join(chase_candidates))
    chase_topic = discover_camera_topic(
        name_hint="chase_cam",
        timeout=30,
        model_hint=args.topic_model_hint,
    )
    if not chase_topic:
        log.error("Chase camera topic not found")
        pm.shutdown()
        sys.exit(1)
    log.info("Using chase camera topic: %s", chase_topic)
    return chase_topic


def _build_png_capture_cmd(cw, ch, cpf, fps, frames_dir):
    return [
        "ffmpeg", "-y",
        "-f", "rawvideo", "-pix_fmt", cpf,
        "-s", f"{cw}x{ch}", "-r", str(fps),
        "-i", "pipe:0",
        "-f", "image2",
        # JPEG quality scale: 2 is best quality/largest file, 31 is lowest quality.
        "-q:v", "3",
        os.path.join(frames_dir, "frame_%06d.jpg"),
    ]


def _has_nvenc_encoder(gpu_available):
    if not gpu_available:
        return False
    try:
        probe = subprocess.run(
            ["ffmpeg", "-hide_banner", "-encoders"],
            capture_output=True, text=True, timeout=5,
        )
        return "h264_nvenc" in probe.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def _build_encoder_opts(use_nvenc):
    opts = ["-pix_fmt", "yuv420p"]
    if use_nvenc:
        opts += [
            "-c:v", "h264_nvenc",
            "-preset", "p4",
            "-tune", "hq",
            "-rc", "vbr",
            "-b:v", "10M",
            "-maxrate", "15M",
            "-g", "30", "-bf", "0",
            "-delay", "0", "-zerolatency", "1",
            "-aud", "1",
        ]
    else:
        opts += [
            "-c:v", "libx264", "-preset", "ultrafast",
            "-crf", "18",
            "-tune", "zerolatency", "-g", "1",
        ]
    opts += ["-flush_packets", "1"]
    return opts


def _build_stream_ffmpeg_cmd(cw, ch, cpf, args, output_mode, use_nvenc):
    cmd = [
        "ffmpeg", "-y",
        "-f", "rawvideo", "-pix_fmt", cpf,
        "-s", f"{cw}x{ch}", "-r", str(args.fps),
        "-i", "pipe:0",
    ]

    cmd += _build_encoder_opts(use_nvenc)

    if output_mode == "udp":
        cmd += [
            "-muxdelay", "0", "-muxpreload", "0",
            "-mpegts_copyts", "1",
            "-f", "mpegts", f"udp://127.0.0.1:{args.port}?pkt_size=1316",
        ]
        viewer_url = f"udp://@:{args.port}"
    elif output_mode == "tcp":
        cmd += [
            "-muxdelay", "0", "-muxpreload", "0",
            "-f", "mpegts", f"tcp://0.0.0.0:{args.port}?listen=1&tcp_nodelay=1",
        ]
        viewer_url = f"tcp://<host>:{args.port}"
    elif output_mode == "rtsp":
        cmd += ["-f", "rtsp", f"rtsp://127.0.0.1:{args.port}/chase"]
        viewer_url = f"rtsp://<host>:{args.port}/chase"
    else:
        cmd += ["-f", "mp4", output_mode[5:]]
        viewer_url = output_mode

    if args.record:
        cmd += [
            "-pix_fmt", "yuv420p",
            "-c:v", "libx264", "-preset", "ultrafast", "-crf", "23",
            "-f", "mp4", args.record,
        ]

    return cmd, viewer_url


def _log_stream_summary(output_mode, args, chase_viewer_url, chase_topic, cw, ch):
    log.info("Shahed chase stream ready")
    log.info("Chase stream: %s", chase_viewer_url)
    if args.record:
        log.info("Recording file: %s (auto-stop on return)", args.record)
    log.info("Camera topic: %s", chase_topic)
    log.info("Resolution: %dx%d @ %d fps", cw, ch, args.fps)
    if output_mode == "udp":
        log.info("View: ffplay -loglevel quiet -fflags nobuffer -flags low_delay -framedrop udp://@:%d", args.port)
    elif output_mode == "tcp":
        log.info("View: ffplay -loglevel quiet -fflags nobuffer -flags low_delay -framedrop tcp://<host>:%d", args.port)
    elif output_mode == "rtsp":
        log.info("View: ffplay -loglevel quiet -fflags nobuffer -flags low_delay -framedrop rtsp://<host>:%d/chase", args.port)
    log.info("Streaming active. Press Ctrl-C to stop")


def _run_recording_mode(args, pm, chase_topic):
    log.info("Recording mode enabled")

    record_dir = os.path.dirname(os.path.abspath(args.record))
    frames_dir = os.path.join(record_dir, "frames_temp")
    os.makedirs(frames_dir, exist_ok=True)

    try:
        png_bridge_proc = pm.spawn(
            [IMAGE_BRIDGE, chase_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except Exception as e:
        log.error("Failed to start image bridge: %s", e)
        pm.shutdown()
        sys.exit(1)

    png_flags = fcntl.fcntl(png_bridge_proc.stderr, fcntl.F_GETFL)
    fcntl.fcntl(png_bridge_proc.stderr, fcntl.F_SETFL, png_flags | os.O_NONBLOCK)

    log.info("Waiting for first frame metadata")
    cw, ch, cpf = read_image_meta(png_bridge_proc, timeout=30)
    if cw is None:
        try:
            _, stderr = png_bridge_proc.communicate(timeout=2)
            log.debug("Image bridge stderr: %s", stderr.decode() if stderr else "No details")
        except Exception as e:
            log.debug("Could not read image bridge stderr: %s", e)
        log.warning("No image metadata from bridge, using 1920x1080 rgb24")
        cw, ch, cpf = 1920, 1080, "rgb24"

    if png_bridge_proc.poll() is not None:
        log.error("Image bridge exited early with return code %d", png_bridge_proc.returncode)
        pm.shutdown()
        sys.exit(1)

    png_cmd = _build_png_capture_cmd(cw, ch, cpf, args.fps, frames_dir)
    png_capture_proc = pm.spawn(
        png_cmd,
        stdin=png_bridge_proc.stdout,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    time.sleep(2)
    if png_capture_proc.poll() is not None:
        log.error("JPEG capture process exited immediately")
        pm.shutdown()
        sys.exit(1)

    stop_info = {}
    monitor_thread = threading.Thread(
        target=_monitor_pose_and_stop_recording,
        kwargs={
            "png_proc": png_capture_proc,
            "frames_dir": frames_dir,
            "stop_info": stop_info,
        },
        daemon=False,
    )
    monitor_thread.start()

    start_recording_time = time.time()
    last_frame_count = 0
    while monitor_thread.is_alive():
        time.sleep(3)
        frame_files = [f for f in os.listdir(frames_dir) if f.endswith(".jpg")]
        current_count = len(frame_files)
        if current_count > last_frame_count:
            elapsed = time.time() - start_recording_time
            fps = current_count / elapsed if elapsed > 0 else 0
            _log_rate_limited(
                "record_progress",
                8,
                "info",
                "Recording progress: %d frames (%.1f fps)",
                current_count,
                fps,
            )
            last_frame_count = current_count
        elif last_frame_count > 0:
            _log_rate_limited(
                "record_stalled",
                15,
                "warning",
                "No new frames captured, still at %d",
                current_count,
            )

    monitor_thread.join()

    try:
        png_capture_proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        png_capture_proc.kill()

    frame_files = sorted([f for f in os.listdir(frames_dir) if f.startswith("frame_") and f.endswith(".jpg")])
    log.info("Captured %d JPG frames", len(frame_files))

    if not frame_files:
        log.error("No frames were captured")
        pm.shutdown()
        sys.exit(1)

    selected_frames = None
    if stop_info.get("triggered"):
        stop_time = stop_info.get("stop_time")
        selected_frames = _select_frames_until_time(
            frames_dir,
            stop_time=stop_time,
            trim_back_frames=args.trim_back_frames,
        )
        if selected_frames:
            log.info("Encoding trimmed frame window (%d frames)", len(selected_frames))

    success = _encode_pngs_to_mp4(frames_dir, args.record, args.fps, selected_frames=selected_frames)
    if success:
        log.info("Recording complete: %s", args.record)
        try:
            import shutil
            shutil.rmtree(frames_dir)
            log.info("Removed temporary frames directory: %s", frames_dir)
        except OSError as e:
            log.warning("Could not remove temporary frames directory %s: %s", frames_dir, e)
    else:
        log.error("Failed to encode MP4")

    pm.shutdown()
    sys.exit(0)


def _run_stream_mode(args, pm, chase_topic, output_mode, gpu_available):
    use_nvenc = _has_nvenc_encoder(gpu_available)
    if use_nvenc:
        log.info("Using NVENC hardware encoder")
    else:
        log.info("Using libx264 software encoder")

    png_bridge_proc = pm.spawn(
        [IMAGE_BRIDGE, chase_topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    png_flags = fcntl.fcntl(png_bridge_proc.stderr, fcntl.F_GETFL)
    fcntl.fcntl(png_bridge_proc.stderr, fcntl.F_SETFL, png_flags | os.O_NONBLOCK)

    log.info("Waiting for first chase camera frame")
    cw, ch, cpf = read_image_meta(png_bridge_proc, timeout=30)
    if cw is None:
        log.warning("No chase camera metadata, using 1920x1080 rgb24")
        cw, ch, cpf = 1920, 1080, "rgb24"
    else:
        log.info("Camera metadata: %dx%d %s", cw, ch, cpf)

    if png_bridge_proc.poll() is not None:
        log.error("Chase camera bridge exited with return code %d", png_bridge_proc.returncode)
        pm.shutdown()
        sys.exit(1)

    if args.raw:
        png_ffmpeg_cmd = [
            "ffplay",
            "-loglevel", "quiet",
            "-f", "rawvideo",
            "-pixel_format", cpf,
            "-video_size", f"{cw}x{ch}",
            "-framerate", str(args.fps),
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-framedrop",
            "-sync", "video",
            "-window_title", f"Chase RAW {cw}x{ch}",
            "-",
        ]
        chase_viewer_url = "(raw ffplay window)"
    else:
        png_ffmpeg_cmd, chase_viewer_url = _build_stream_ffmpeg_cmd(
            cw,
            ch,
            cpf,
            args,
            output_mode,
            use_nvenc,
        )

    png_ffmpeg_proc = pm.spawn(
        png_ffmpeg_cmd,
        stdin=png_bridge_proc.stdout,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    if png_ffmpeg_proc.poll() is not None:
        log.error("Chase stream process failed to start")
        pm.shutdown()
        sys.exit(1)

    if args.record:
        monitor_thread = threading.Thread(
            target=_monitor_pose_and_stop_recording,
            args=(png_ffmpeg_proc, 0.6, 1),
            daemon=False,
        )
        monitor_thread.start()

    _log_stream_summary(output_mode, args, chase_viewer_url, chase_topic, cw, ch)
    return png_bridge_proc, png_ffmpeg_proc, png_ffmpeg_cmd


def _supervise_stream(pm, png_bridge_proc, png_ffmpeg_proc, png_ffmpeg_cmd):
    restart_attempts = 0
    while True:
        if png_bridge_proc and png_bridge_proc.poll() is not None:
            log.error("Camera bridge exited with code %d", png_bridge_proc.returncode)
            break

        if png_ffmpeg_proc and png_ffmpeg_cmd and png_ffmpeg_proc.poll() is not None:
            rc = png_ffmpeg_proc.returncode
            if restart_attempts >= MAX_STREAM_RESTARTS:
                log.error("Chase stream exited (code %d) and restart limit reached", rc)
                break

            restart_attempts += 1
            backoff_s = min(8, 2 ** (restart_attempts - 1))
            log.warning(
                "Chase stream exited (code %d), restart %d/%d in %ds",
                rc,
                restart_attempts,
                MAX_STREAM_RESTARTS,
                backoff_s,
            )
            if png_ffmpeg_proc in pm.procs:
                pm.procs.remove(png_ffmpeg_proc)
            time.sleep(backoff_s)
            png_ffmpeg_proc = pm.spawn(
                png_ffmpeg_cmd,
                stdin=png_bridge_proc.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if png_ffmpeg_proc.poll() is None:
                log.info("Chase stream restarted")

        _log_rate_limited("stream_health", 20, "debug", "Stream health check: bridge and stream alive")
        time.sleep(2)


def main():
    args = parse_args()

    cleanup_before_start()
    output_mode = _resolve_output_mode(args)

    pm = ProcessManager()

    def on_signal(sig, frame):
        pm.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    gpu_available = _configure_runtime(args, pm)
    _start_optional_rtsp_server(output_mode, args, pm)
    _start_gazebo(args, pm)
    chase_topic = _resolve_chase_topic(args, pm)

    if args.record:
        _run_recording_mode(args, pm, chase_topic)

    png_bridge_proc, png_ffmpeg_proc, png_ffmpeg_cmd = _run_stream_mode(
        args,
        pm,
        chase_topic,
        output_mode,
        gpu_available,
    )

    try:
        _supervise_stream(pm, png_bridge_proc, png_ffmpeg_proc, png_ffmpeg_cmd)
    except KeyboardInterrupt:
        log.info("Received interrupt signal")

    pm.shutdown()
    log.info("Simulation stopped cleanly")
    return

if __name__ == "__main__":
    main()
