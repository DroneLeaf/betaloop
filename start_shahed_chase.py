#!/usr/bin/env python3
"""Shahed Drone Chase Camera Launcher — Gazebo + Camera → RTSP/TCP/UDP stream.

Starts Gazebo with the rocket_drone_park world and streams the Shahed drone's
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
TARGET_WORLD = "rocket_drone_park.world"
IMAGE_BRIDGE = os.path.join(AEROLOOP_HOME, "plugins", "build", "gz_image_bridge")
STREAM_PORT = 8554
TOPIC_MODEL_HINT_DEFAULT = "shahed_drone"

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
                    except:
                        pass  # Model check failed, keep waiting
                    
                elapsed = time.time() - start_time
                if elapsed - (last_log_time - start_time) >= 10:  # Log every 10 seconds
                    log.info("Gazebo initializing... (%d topics found, %.1fs elapsed)", len(topics), elapsed)
                    last_log_time = time.time()
            
        except subprocess.TimeoutExpired:
            elapsed = time.time() - start_time
            if elapsed - (last_log_time - start_time) >= 10:
                log.info("Gazebo still initializing (timeout in subprocess, %.1fs elapsed)", elapsed)
                last_log_time = time.time()
        except Exception as e:
            pass
        
        time.sleep(1)  # Increased from 0.5 to 1 second
    
    log.error("Gazebo did not become ready within %ds", timeout)
    return False


def cleanup_before_start():
    """Clean up any existing processes and temporary files."""
    log.info("Cleaning up from previous runs...")
    
    # Kill any existing gz sim processes (parent=1 means don't inherit from this script)
    try:
        subprocess.run(["bash", "-c", "pkill -9 -f 'gz sim' 2>/dev/null || true"], capture_output=True, timeout=5)
        log.info("Stopped any existing Gazebo processes")
    except:
        pass
    
    time.sleep(0.5)
    
    # Kill any existing ffmpeg image capture processes
    try:
        subprocess.run(["bash", "-c", "pkill -9 ffmpeg 2>/dev/null || true"], capture_output=True, timeout=5)
        log.info("Stopped any existing ffmpeg processes")
    except:
        pass
    
    time.sleep(0.5)
    
    # Kill any existing gz_image_bridge processes
    try:
        subprocess.run(["bash", "-c", "pkill -9 -f 'gz_image_bridge' 2>/dev/null || true"], capture_output=True, timeout=5)
        log.info("Stopped any existing image bridge processes")
    except:
        pass
    
    # Remove frames_temp directory if it exists
    try:
        frames_dir = os.path.expanduser("~/frames_temp")
        if os.path.exists(frames_dir):
            import shutil
            shutil.rmtree(frames_dir)
            log.info("Cleaned up existing frames directory: %s", frames_dir)
    except Exception as e:
        log.warning("Could not clean frames directory: %s", e)
    
    time.sleep(0.5)  # Give processes time to fully terminate
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


def _monitor_pose_and_stop_recording(jpeg_proc, return_threshold=1, confirmation_frames=1):
    """Monitors SHAHED DRONE geranium_link pose via /world/default/pose/info topic.
    
    Recording starts immediately (as soon as first position received).
    Enforces minimum 3 second recording duration before allowing stop detection.
    Stops when drone returns to home within threshold.
    
    Parameters:
      return_threshold: Distance in meters to detect return to home (default 0.6m)
      confirmation_frames: Number of consecutive frames below threshold before stopping (default 1)
    
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
    log.info("Pose monitor started - tracking SHAHED DRONE geranium_link")
    log.info("  Recording starts immediately (frame received)")
    log.info("  Minimum duration: 3 seconds before stop detection allowed")
    log.info("  Return threshold: %.2fm (stops when within this distance)", return_threshold)
    log.info("Listening to world pose topic: /world/default/pose/info")
    
    home_pos = None
    message_count = 0
    pose_proc = None
    close_frames = 0  # Counter for consecutive frames below threshold
    last_logged_dist = None
    recording_start_time = None  # Time when first position received (recording began)
    
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
                                log.info("🏠 HOME POSITION SET: (%.2f, %.2f, %.2f) — RECORDING STARTED", *home_pos)
                            else:
                                dist = math.sqrt((x-home_pos[0])**2 + (y-home_pos[1])**2 + (z-home_pos[2])**2)
                                elapsed = time.time() - recording_start_time
                                
                                # Log distance changes (only when change is significant)
                                if last_logged_dist is None or abs(dist - last_logged_dist) >= 0.05:
                                    log.info("Distance: %.3fm (%.1fs recording, close_frames: %d/%d)", dist, elapsed, close_frames, confirmation_frames)
                                    last_logged_dist = dist
                                
                                # Only check for return after 3 seconds of recording
                                if elapsed >= 3.0:
                                    if dist < return_threshold:
                                        close_frames += 1
                                        if close_frames >= confirmation_frames:
                                            log.info("✈️  DRONE RETURNED (%.3fm from home, %.1fs recording). Stopping.", dist, elapsed)
                                            jpeg_proc.terminate()
                                            try:
                                                jpeg_proc.wait(timeout=5)
                                            except subprocess.TimeoutExpired:
                                                jpeg_proc.kill()
                                            return
                                    else:
                                        # Reset counter if drone moves away
                                        if close_frames > 0:
                                            log.info("Drone moved away (%.3fm) — confirming return cancelled", dist)
                                        close_frames = 0
                        
                    except (ValueError, AttributeError) as e:
                        log.debug("Parse error: %s", e)
                
                # Clear buffer after processing pose block
                buffer = []
            
            # Safety: Check if JPEG process died
            if jpeg_proc.poll() is not None:
                log.info("JPEG process exited, pose monitor ending")
                return
        
        log.warning("Pose stream ended (received %d messages)", message_count)
            
    except Exception as e:
        log.error("Pose monitor error: %s", e, exc_info=True)
    finally:
        if pose_proc and pose_proc.poll() is None:
            pose_proc.terminate()
            try:
                pose_proc.wait(timeout=2)
            except:
                pose_proc.kill()
        log.info("Pose monitor stopped (received %d messages total)", message_count)


def _encode_jpegs_to_mp4(frames_dir, output_file, fps=30):
    """Convert JPEG frames to MP4 file."""
    log.info("Encoding %d JPEG frames to MP4: %s", len(os.listdir(frames_dir)), output_file)
    try:
        ffmpeg_cmd = [
            "ffmpeg", "-y",
            "-framerate", str(fps),
            "-i", os.path.join(frames_dir, "frame_%06d.jpg"),
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

def main():
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
    args = parser.parse_args()

    # Clean up any existing processes and temporary files before starting
    cleanup_before_start()

    # Determine output mode
    if args.output:
        output_mode = args.output
    elif args.rtsp:
        output_mode = "rtsp"
    else:
        output_mode = "udp"

    pm = ProcessManager()

    def on_signal(sig, frame):
        pm.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # ── 1. Environment ──
    log.info("Setting up Gazebo environment")
    setup_gazebo_env()

    # ── 1b. GPU detection & rendering setup ──
    gpu_available = has_nvidia_gpu()
    in_container = _is_container()

    if gpu_available:
        log.info("NVIDIA GPU detected — using GPU-accelerated rendering")
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
        nvidia_icd = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.isfile(nvidia_icd):
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_icd
        if in_container:
            os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
            _fix_dri_permissions()
    else:
        log.info("No GPU detected — using software rendering (llvmpipe)")
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    # ── 1c. Display setup ──
    if args.gazebo:
        if not os.environ.get("DISPLAY"):
            log.error("No DISPLAY set — the Gazebo GUI needs a display.")
            sys.exit(1)
        os.environ.pop("__GLX_VENDOR_LIBRARY_NAME", None)
        log.info("Using display %s for Gazebo GUI", os.environ["DISPLAY"])
    elif not in_container and os.environ.get("DISPLAY"):
        log.info("Using native display %s (low-latency host mode)", os.environ["DISPLAY"])
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
            log.error("Xvfb failed to start — camera rendering requires a display")
            sys.exit(1)
        os.environ["DISPLAY"] = ":99"

    # ── 2. RTSP server (optional) ──
    if output_mode == "rtsp" and not args.record:
        mediamtx = "/usr/local/bin/mediamtx"
        if os.path.isfile(mediamtx):
            log.info("Starting mediamtx RTSP server")
            pm.spawn([mediamtx], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(2)

    # ── 3. Gazebo ──
    world_path = args.world
    if not os.path.isabs(world_path):
        world_path = os.path.join(AEROLOOP_HOME, "worlds", world_path)

    gz_args = ["gz", "sim"]
    if not args.gazebo:
        gz_args.append("-s")  # headless by default
    gz_args.extend(["-r", "-v", "3", world_path])

    log.info("Starting Gazebo%s: %s", " (GUI)" if args.gazebo else " (headless)", os.path.basename(world_path))
    pm.spawn(gz_args)
    
    # Wait for Gazebo to be fully initialized and responsive
    if not wait_for_gazebo_ready(timeout=90):
        log.error("Gazebo failed to initialize properly")
        pm.shutdown()
        sys.exit(1)

    # ── 4. Discover camera topic ──
    if args.chase_topic:
        chase_topic = args.chase_topic
        log.info("Using explicit chase camera topic: %s", chase_topic)
    else:
        log.info("Discovering chase camera image topic …")
        chase_candidates = list_camera_topics(name_hint="chase_cam")
        if chase_candidates:
            log.info("Chase candidates: %s", ", ".join(chase_candidates))
        chase_topic = discover_camera_topic(
            name_hint="chase_cam",
            timeout=30,
            model_hint=args.topic_model_hint,
        )
        if not chase_topic:
            log.error("Chase camera topic not found.")
            pm.shutdown()
            sys.exit(1)
        else:
            log.info("Found chase camera topic: %s", chase_topic)


    # ── RECORDING MODE (jpeg capture) ──
    if args.record:
        log.info("Recording mode: jpeg capture pipeline")
        
        # Create frames directory
        record_dir = os.path.dirname(os.path.abspath(args.record))
        frames_dir = os.path.join(record_dir, "frames_temp")
        os.makedirs(frames_dir, exist_ok=True)
        log.info("Frames will be saved to: %s", frames_dir)
        
        # Start image bridge for jpeg capture only
        log.info("Starting image bridge for jpeg capture")
        try:
            jpeg_bridge_proc = pm.spawn(
                [IMAGE_BRIDGE, chase_topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except Exception as e:
            log.error("Failed to start image bridge: %s", e)
            pm.shutdown()
            sys.exit(1)
            
        jpeg_flags = fcntl.fcntl(jpeg_bridge_proc.stderr, fcntl.F_GETFL)
        fcntl.fcntl(jpeg_bridge_proc.stderr, fcntl.F_SETFL, jpeg_flags | os.O_NONBLOCK)
        
        log.info("Waiting for first frame metadata …")
        cw, ch, cpf = read_image_meta(jpeg_bridge_proc, timeout=30)
        if cw is None:
            # Try to read stderr for more details on the error
            try:
                _, stderr = jpeg_bridge_proc.communicate(timeout=2)
                log.error("Image bridge stderr: %s", stderr.decode() if stderr else "No error details")
            except:
                pass
            log.warning("No image metadata from bridge. Falling back to defaults (1920x1080 rgb24)")
            log.warning("Image bridge may not be running correctly (missing GStreamer plugins?)")
            # Fallback to standard camera defaults
            cw, ch, cpf = 1920, 1080, "rgb24"
        else:
            log.info("✓ Camera metadata received: %dx%d %s", cw, ch, cpf)
        
        # Verify image bridge is still alive
        if jpeg_bridge_proc.poll() is not None:
            returncode = jpeg_bridge_proc.returncode
            log.error("✗ Image bridge crashed with return code %d", returncode)
            try:
                stderr_data = jpeg_bridge_proc.stderr.read()
                if stderr_data:
                    log.error("Bridge stderr: %s", stderr_data.decode('utf-8', errors='ignore'))
            except:
                pass
        else:
            log.info("✓ Image bridge is running")
        
        # Start jpeg capture using ffmpeg
        jpeg_cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo", "-pix_fmt", cpf,
            "-s", f"{cw}x{ch}", "-r", str(args.fps),
            "-i", "pipe:0",
            "-f", "image2", "-q:v", "5",
            os.path.join(frames_dir, "frame_%06d.jpg")
        ]
        log.info("Starting JPEG frame capture: %s/frame_*.jpg (quality 5)", frames_dir)
        jpeg_capture_proc = pm.spawn(
            jpeg_cmd,
            stdin=jpeg_bridge_proc.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        
        # Give it a moment to start capturing
        time.sleep(2)
        
        # Check if ffmpeg process is alive and frames are being created
        if jpeg_capture_proc.poll() is not None:
            log.error("✗ JPEG capture process died immediately")
            pm.shutdown()
            sys.exit(1)
        else:
            log.info("✓ JPEG capture process started")
        
        # Check if any frames have been created yet
        initial_frames = len([f for f in os.listdir(frames_dir) if f.endswith(".jpg")])
        log.info("Initial frame count: %d frames in %s", initial_frames, frames_dir)
        
        # Wait for drone to return home, then stop jpeg capture
        log.info("Recording in progress (waiting for drone to return)...")
        monitor_thread = threading.Thread(
            target=_monitor_pose_and_stop_recording,
            args=(jpeg_capture_proc,),  # Uses defaults: return_threshold=0.6m, confirmation_frames=1
            daemon=False
        )
        monitor_thread.start()
        
        # Monitor frame count in background during recording
        start_recording_time = time.time()
        last_frame_count = 0
        while monitor_thread.is_alive():
            time.sleep(3)
            frame_files = [f for f in os.listdir(frames_dir) if f.endswith(".jpg")]
            current_count = len(frame_files)
            if current_count > last_frame_count:
                elapsed = time.time() - start_recording_time
                fps = current_count / elapsed if elapsed > 0 else 0
                log.info("Recording: %d frames (%d new, %.1f fps actual)", current_count, current_count - last_frame_count, fps)
                last_frame_count = current_count
            elif last_frame_count > 0:
                log.warning("⚠ No new frames captured in last 3 seconds (stuck at %d)", current_count)
        
        monitor_thread.join()
        
        # Wait for jpeg process to finish
        try:
            jpeg_capture_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            jpeg_capture_proc.kill()
        
        # Count captured frames and encode to MP4
        frame_files = sorted([f for f in os.listdir(frames_dir) if f.startswith("frame_") and f.endswith(".jpg")])
        log.info("Total captured: %d JPEG frames", len(frame_files))
        
        if len(frame_files) == 0:
            log.error("✗ No frames were captured! Check:")
            log.error("  1. Image bridge is running and receiving camera data")
            log.error("  2. FFmpeg process stayed alive during recording")
            log.error("  3. GStreamer plugins installed (gstreamer1.0-plugins-bad, etc)")
            pm.shutdown()
            sys.exit(1)
        
        if frame_files:
            log.info("Converting jpeg frames to MP4: %s", args.record)
            success = _encode_jpegs_to_mp4(frames_dir, args.record, args.fps)
            if success:
                log.info("Recording complete: %s", args.record)
            else:
                log.error("Failed to encode MP4")
        else:
            log.error("No frames captured")
        
        pm.shutdown()
        sys.exit(0)
    
    # ── STREAMING MODE ──
    log.info("Streaming mode: live broadcast pipeline")
    
    # Try NVENC hardware encoder first (RTX/GTX GPUs), fall back to libx264.
    use_nvenc = False
    if gpu_available:
        try:
            probe = subprocess.run(
                ["ffmpeg", "-hide_banner", "-encoders"],
                capture_output=True, text=True, timeout=5,
            )
            if "h264_nvenc" in probe.stdout:
                use_nvenc = True
                log.info("Using NVENC hardware encoder (h264_nvenc)")
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass
    
    # ── 5. Chase camera pipeline ──
    log.info("Starting chase camera bridge")
    chase_bridge_proc = pm.spawn(
        [IMAGE_BRIDGE, chase_topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    chase_flags = fcntl.fcntl(chase_bridge_proc.stderr, fcntl.F_GETFL)
    fcntl.fcntl(chase_bridge_proc.stderr, fcntl.F_SETFL, chase_flags | os.O_NONBLOCK)

    log.info("Waiting for first chase camera frame …")
    cw, ch, cpf = read_image_meta(chase_bridge_proc, timeout=30)
    if cw is None:
        log.warning("No chase camera metadata — using fallback defaults (1920x1080 rgb24)")
        log.warning("Camera bridge may not be functioning correctly")
        cw, ch, cpf = 1920, 1080, "rgb24"
    else:
        log.info("✓ Camera metadata received: %dx%d %s", cw, ch, cpf)
    
    # Verify chase bridge is still alive
    if chase_bridge_proc.poll() is not None:
        log.error("✗ Chase camera bridge crashed with return code %d", chase_bridge_proc.returncode)
        pm.shutdown()
        sys.exit(1)
    else:
        log.info("✓ Chase camera bridge is running")

        if args.raw:
            chase_ffplay_cmd = [
                "ffplay",
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
            log.info("Starting chase camera ffplay (raw)")
            chase_ffmpeg_proc = pm.spawn(
                chase_ffplay_cmd,
                stdin=chase_bridge_proc.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            chase_ffmpeg_cmd = chase_ffplay_cmd
            chase_viewer_url = "(raw ffplay window)"
        else:
            # Build base ffmpeg input (rawvideo -> yuv420p)
            chase_ffmpeg_input = [
                "ffmpeg", "-y",
                "-f", "rawvideo", "-pix_fmt", cpf,
                "-s", f"{cw}x{ch}", "-r", str(args.fps),
                "-i", "pipe:0",
            ]

            # Define encoder options (will apply to first output)
            encoder_opts = ["-pix_fmt", "yuv420p"]
            if use_nvenc:
                log.info("✓ Using NVENC encoder (h264_nvenc)")
                encoder_opts += [
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
                log.info("✓ Using libx264 software encoder (ultrafast preset)")
                encoder_opts += [
                    "-c:v", "libx264", "-preset", "ultrafast",
                    "-crf", "18",
                    "-tune", "zerolatency", "-g", "1",
                ]
            encoder_opts += ["-flush_packets", "1"]

            # Build streaming output
            streaming_opts = encoder_opts.copy()
            if output_mode == "udp":
                log.info("Streaming mode: UDP/MPEGTS to 127.0.0.1:%d", args.port)
                streaming_opts += [
                    "-muxdelay", "0", "-muxpreload", "0",
                    "-mpegts_copyts", "1",
                    "-f", "mpegts", f"udp://127.0.0.1:{args.port}?pkt_size=1316",
                ]
                chase_viewer_url = f"udp://@:{args.port}"
            elif output_mode == "tcp":
                log.info("Streaming mode: TCP/MPEGTS listening on port %d", args.port)
                streaming_opts += [
                    "-muxdelay", "0", "-muxpreload", "0",
                    "-f", "mpegts", f"tcp://0.0.0.0:{args.port}?listen=1&tcp_nodelay=1",
                ]
                chase_viewer_url = f"tcp://<host>:{args.port}"
            elif output_mode == "rtsp":
                log.info("Streaming mode: RTSP on rtsp://127.0.0.1:%d/chase", args.port)
                streaming_opts += ["-f", "rtsp", f"rtsp://127.0.0.1:{args.port}/chase"]
                chase_viewer_url = f"rtsp://<host>:{args.port}/chase"
            else:
                log.info("Streaming mode: MP4 file output to %s", output_mode[5:])
                streaming_opts += ["-f", "mp4", output_mode[5:]]
                chase_viewer_url = output_mode

            # Build complete ffmpeg command
            chase_ffmpeg_cmd = chase_ffmpeg_input + streaming_opts

            # Add recording output if specified
            if args.record:
                # Separate encoder for MP4 output (different quality settings)
                record_opts = [
                    "-pix_fmt", "yuv420p",
                    "-c:v", "libx264", "-preset", "ultrafast", "-crf", "23",
                    "-f", "mp4", args.record
                ]
                chase_ffmpeg_cmd += record_opts
                log.info("Secondary output: Recording to %s", args.record)
            
            log.info("Starting Chase camera FFmpeg pipeline (input: %dx%d %s)", cw, ch, cpf)
            chase_ffmpeg_proc = pm.spawn(
                chase_ffmpeg_cmd,
                stdin=chase_bridge_proc.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            
            if chase_ffmpeg_proc.poll() is None:
                log.info("✓ FFmpeg streaming process started")
            else:
                log.error("✗ FFmpeg process failed to start")
            
            if args.record:
                log.info("Starting pose monitor for auto-stop on return to home")
                monitor_thread = threading.Thread(
                    target=_monitor_pose_and_stop_recording,
                    args=(chase_ffmpeg_proc, 0.6, 1),  # return_threshold=0.6m, confirmation_frames=1
                    daemon=False
                )
                monitor_thread.start()

    # ── 10. Print connection info ──
    log.info("═" * 60)
    log.info("SHAHED CHASE SIMULATION RUNNING")
    log.info("═" * 60)
    log.info("Chase stream    : %s", chase_viewer_url)
    if args.record:
        log.info("Recording file  : %s (Auto-stops on return)", args.record)
    log.info("Camera topic    : %s", chase_topic)
    log.info("Resolution      : %dx%d @ %d fps", cw, ch, args.fps)
    if output_mode == "udp":
        log.info("View command    : ffplay -fflags nobuffer -flags low_delay -framedrop udp://@:%d", args.port)
    elif output_mode == "tcp":
        log.info("View command    : ffplay -fflags nobuffer -flags low_delay -framedrop tcp://<host>:%d", args.port)
    elif output_mode == "rtsp":
        log.info("View command    : ffplay -fflags nobuffer -flags low_delay -framedrop rtsp://<host>:%d/chase", args.port)
    log.info("═" * 60)
    log.info("Streaming active. Press Ctrl-C to stop")
    log.info("═" * 60)
    
    print()
    print("=" * 60)
    print("  Shahed Chase Simulation Running")
    print("=" * 60)
    print()
    if chase_viewer_url:
        print(f"  Chase stream : {chase_viewer_url}")
    if args.record:
        print(f"  Recording to : {args.record} (Auto-stops on return)")
    print(f"  Chase topic  : {chase_topic}")
    print(f"  Resolution   : {cw}x{ch} @ {args.fps} fps")
    print()
    low_lat = "-loglevel quiet -fflags nobuffer -flags low_delay -framedrop"
    if output_mode == "udp":
        if chase_viewer_url:
            print(f"  Chase view: ffplay {low_lat} udp://@:{args.port}")
    elif output_mode == "tcp":
        if chase_viewer_url:
            print(f"  Chase view: ffplay {low_lat} tcp://<host>:{args.port}")
    elif output_mode == "rtsp":
        if chase_viewer_url:
            print(f"  Chase view: ffplay {low_lat} rtsp://<host>:{args.port}/chase")
    print()
    print("  Press Ctrl-C to stop")
    print("=" * 60)
    print()

    # ── 11. Keep alive (auto-restart ffmpeg on viewer disconnect) ──
    last_health_check = time.time()
    health_check_interval = 5  # Log health every 5 seconds
    try:
        while True:
            # Periodic health check logging
            now = time.time()
            if now - last_health_check >= health_check_interval:
                if chase_ffmpeg_proc and chase_ffmpeg_proc.poll() is None:
                    log.debug("✓ Chase FFmpeg process alive")
                if chase_bridge_proc and chase_bridge_proc.poll() is None:
                    log.debug("✓ Camera bridge process alive")
                last_health_check = now
            
            # Auto-restart chase camera ffmpeg
            if chase_ffmpeg_proc and chase_ffmpeg_cmd:
                if chase_bridge_proc and chase_bridge_proc.poll() is not None:
                    log.error("✗ Chase camera bridge exited with code %d", chase_bridge_proc.returncode)
                    break
                elif chase_ffmpeg_proc.poll() is not None:
                    rc = chase_ffmpeg_proc.returncode
                    log.warning("⚠ Chase FFmpeg exited (code %d) — attempting restart …", rc)
                    if chase_ffmpeg_proc in pm.procs:
                        pm.procs.remove(chase_ffmpeg_proc)
                    time.sleep(1)
                    log.info("Restarting Chase FFmpeg pipeline")
                    chase_ffmpeg_proc = pm.spawn(
                        chase_ffmpeg_cmd,
                        stdin=chase_bridge_proc.stdout,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    if chase_ffmpeg_proc.poll() is None:
                        log.info("✓ FFmpeg restarted successfully")

            time.sleep(2)
    except KeyboardInterrupt:
        log.info("Received interrupt signal")

    pm.shutdown()
    log.info("═" * 60)
    log.info("Simulation stopped cleanly")
    log.info("═" * 60)

if __name__ == "__main__":
    main()
