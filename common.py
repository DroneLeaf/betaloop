"""Shared utilities for betaloop simulation launchers (start.py / start_px4.py).

Centralises Gazebo environment setup, process management, display configuration,
camera topic discovery, image-bridge metadata reading, cleanup, and target
trajectory generation so that both BF and PX4 launchers stay in sync.
"""

import fcntl
import filecmp
import glob
import logging
import math
import os
import random
import select
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time

log = logging.getLogger(__name__)

# ── Paths ─────────────────────────────────────────────────────────────────────

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)


def default_path(env_var: str, repo_relative: str) -> str:
    """Return *env_var* override, or *repo_relative* joined to the repo root."""
    if os.environ.get(env_var):
        return os.environ[env_var]
    return os.path.join(_REPO_ROOT, repo_relative)


AEROLOOP_HOME = default_path("AEROLOOP_HOME", "aeroloop_gazebo")
SIMULINK_LIB = default_path(
    "SIMULINK_LIB",
    os.path.join(
        "HEAR_Simulations",
        "Compiled_models",
        "rocket_drone_quad_SITL",
        "libinterface_simulink.so",
    ),
)
IMAGE_BRIDGE = os.path.join(AEROLOOP_HOME, "plugins", "build", "gz_image_bridge")
TOPIC_MODEL_HINT_DEFAULT = "rocket_drone"
DEFAULT_DRONE = "rocket_drone"

# Per-drone reference calibration.  max_thrust is the total static thrust from
# all 4 rotors at vel_cmd_max.  *_per_kg are the moment-of-inertia values
# normalised to 1 kg so we can scale linearly with mass.
DRONE_REFS = {
    "rocket_drone": {
        # Gazebo-physics model removed; vis-only model below drives the camera rig.
        "model_vis_sdf": "rocket_drone_vis/model.sdf",
        "model_uri": "model://rocket_drone",
        "model_vis_uri": "model://rocket_drone_vis",
        "max_thrust": 22.0,       # N — calibrated: 0.5 kg → CTW ≈ 4.5
        "ixx_per_kg": 0.019417,   # 0.029125 / 1.5
        "iyy_per_kg": 0.019417,
        "izz_per_kg": 0.036817,   # 0.055225 / 1.5
        "default_ctw": 4.5,
        "default_standoff": 0.20,
        "leg_attach_offset": 0.025,
        "default_damping": {
            "linear_x": 0.1, "linear_y": 0.4, "linear_z": 0.03,
            "quadratic_x": 0.001, "quadratic_y": 0.03, "quadratic_z": 0.001,
            "angular": 0.03,
        },
    },
    "thaqib_1_prototype": {
        "model_vis_sdf": "thaqib_1_prototype_vis/model.sdf",
        "model_uri": "model://thaqib_1_prototype",
        "model_vis_uri": "model://thaqib_1_prototype_vis",
        "max_thrust": 22.0,
        "ixx_per_kg": 0.019417,
        "iyy_per_kg": 0.019417,
        "izz_per_kg": 0.036817,
        "default_ctw": 4.5,
        "default_standoff": 0.20,
        "leg_attach_offset": 0.025,
        "default_damping": {
            "linear_x": 0.1, "linear_y": 0.4, "linear_z": 0.03,
            "quadratic_x": 0.001, "quadratic_y": 0.03, "quadratic_z": 0.001,
            "angular": 0.03,
        },
    },
    "iris": {
        "model_vis_sdf": "iris_vis/model.sdf",
        "model_uri": "model://betaloop_iris_with_standoffs",
        "model_vis_uri": "model://iris_vis",
        "max_thrust": 8.0,        # N — estimated: 0.4 kg → CTW ≈ 2
        "ixx_per_kg": 0.005,      # 0.002 / 0.4
        "iyy_per_kg": 0.010,      # 0.004 / 0.4
        "izz_per_kg": 0.01125,    # 0.0045 / 0.4
        "default_ctw": 2.0,
        "default_standoff": 0.17,
        "leg_attach_offset": 0.025,
        "default_damping": {
            "linear_x": 0.3, "linear_y": 0.3, "linear_z": 0.3,
            "quadratic_x": 0, "quadratic_y": 0, "quadratic_z": 0,
            "angular": 0.005,
        },
    },
}

# Selectable target-drone models — swap the tracked target without editing worlds.
#   mesh_uri    — glb for worlds that inline the visual (park_chase, patrol_park)
#   model_uri   — full model for worlds that <include> the target (collision_test)
#   visual_pose — mesh orientation offset (world forward-flight convention)
#   bbox        — proximity OBB half-extents "X,Y,Z" (m) for the TARGET REACHED OSD
TARGET_REFS = {
    "shahed": {
        "label": "Shahed",
        "mesh_uri": "model://shahed_drone/meshes/shahed.glb",
        "model_uri": "model://shahed_drone",
        "visual_pose": "0 0 0 1.57079 0 1.5708",
        # base_scale = the uniform scale at which `bbox` was measured;
        # default_scale = the applied scale (shahed is fixed at 1.0).
        "base_scale": 1.0,
        "default_scale": 1.0,
        "bbox": "0.792,1.047,0.186",
    },
    "stingjet": {
        "label": "StingJet",
        "mesh_uri": "model://stingjet/stingjet.glb",
        "model_uri": "model://stingjet",
        "visual_pose": "0 0 0 1.57079 0 1.5708",
        # bbox measured at 0.1x (the raw mesh is fighter-jet sized at 1.0x);
        # default applied scale 0.1x, UI/CLI-overridable via --target-scale.
        "base_scale": 0.1,
        "default_scale": 0.1,
        "bbox": "0.94,0.54,0.16",
    },
    "balloon": {
        # A primitive (no mesh): worlds that inline the visual draw a <sphere>
        # instead of a <mesh>; worlds that <include> use models/balloon. Used as
        # the default target of the wind (drifting) and shake worlds.
        "label": "Balloon",
        "primitive": "sphere",
        "radius": 0.5,                 # metres at scale 1.0
        "color": "0.9 0.1 0.1",        # red (ambient/diffuse)
        "mesh_uri": "model://balloon/balloon.glb",   # unused (primitive), kept for parity
        "model_uri": "model://balloon",
        "visual_pose": "0 0 0 0 0 0",
        "base_scale": 1.0,
        "default_scale": 1.0,
        "bbox": "0.5,0.5,0.5",
    },
}
DEFAULT_TARGET_DRONE = "shahed"


# ── Terrain themes ────────────────────────────────────────────────────────────
# The baylands terrain DAE hardcodes its ground texture filenames (Grass.png,
# Sand.png, DirtPath.png), so a theme is applied by copying that theme's texture
# set from models/baylands_terrain/media/Textures/themes/<theme>/ over the live
# files before Gazebo starts (apply_terrain_theme). `ground_color` additionally
# tints the giant ground plane in worlds that template it (moving_target).
TERRAIN_THEMES = {
    "desert": {
        "label": "Desert",
        "ground_color": "0.76 0.60 0.42",   # sandy tan
    },
    "lush": {
        "label": "Lush Green",
        "ground_color": "0.13 0.20 0.09",   # dark lush green
    },
}
DEFAULT_TERRAIN_THEME = "desert"
_TERRAIN_TEXTURE_NAMES = ("Grass.png", "Sand.png", "DirtPath.png")

# Sky brightness (1.0 = bright noon … lower = late afternoon). A single scalar
# drives the sun light color/intensity, the sun elevation (longer shadows), the
# scene background, and the skybox <time> in worlds that template them.
DEFAULT_SKY_BRIGHTNESS = 1.0
_SKY_BRIGHTNESS_MIN = 0.15   # keep the scene readable — never pitch black


def _sky_vars(brightness: float | None) -> dict:
    """Derive sun/sky template vars from a 0.15–1.0 brightness scalar.

    As brightness drops the light warms toward orange, the sun lowers toward
    the horizon, the background darkens, and the skybox time advances from
    mid-day toward ~17:00 (late afternoon).

    NOTE (Gazebo Harmonic): ogre2 renders the <sky> as a STATIC cubemap
    (skybox.dds) — <sky><time>/<clouds>/<humidity> are parsed but ignored, and
    gz-sim 8's sensor path (RenderUtil) has no cubemap_uri support either. The
    skybox itself is therefore dimmed by ensure_sky_media(), which builds a
    GZ_RENDERING_RESOURCE_PATH override tree with a brightness-scaled
    skybox.dds. The vars below still drive the sun light, background color and
    ground shading (all honored).
    """
    b = DEFAULT_SKY_BRIGHTNESS if brightness is None else float(brightness)
    b = min(1.0, max(_SKY_BRIGHTNESS_MIN, b))
    w = 1.0 - b                       # warm/afternoon factor
    lerp = lambda a, c: a + (c - a) * w
    intensity = 0.35 + 0.65 * b       # overall light-level floor
    rgb = lambda noon, dusk, k=1.0: " ".join(
        f"{lerp(n, d) * k:.3f}" for n, d in zip(noon, dusk))
    return {
        "sky_brightness": b,
        # sky shader time-of-day: 10 (current mid-day look) → ~17 (late afternoon)
        "sky_time": round(10.0 + 7.0 * w, 2),
        "sun_diffuse": rgb((0.95, 0.85, 0.70), (1.00, 0.50, 0.25), intensity),
        "sun_specular": rgb((0.30, 0.25, 0.20), (0.35, 0.20, 0.10), intensity),
        # lower sun as brightness drops (shallower -Z) → longer shadows
        "sun_direction": f"-0.5 0.1 {-lerp(0.9, 0.25):.3f}",
        "background_color": rgb((0.50, 0.70, 0.90), (0.60, 0.50, 0.45),
                                0.40 + 0.60 * b),
    }


def _dxt1_faces(src_path: str):
    """Decode a single-level DXT1 cubemap .dds → (faces float32 [n,h,w,3], caps2).

    Vectorised numpy DXT1 decode (endpoints + 2-bit palette indices per 4×4
    block); the source must be mip-less like gz-rendering's skybox.dds.
    """
    import numpy as np

    raw = np.fromfile(src_path, dtype=np.uint8)
    hdr = raw[:128]
    fourcc = hdr[84:88].tobytes()
    height = int.from_bytes(hdr[12:16], "little")
    width = int.from_bytes(hdr[16:20], "little")
    caps2 = int.from_bytes(hdr[112:116], "little")
    if hdr[:4].tobytes() != b"DDS " or fourcc != b"DXT1":
        raise ValueError(f"not a DXT1 DDS: {src_path} (fourcc={fourcc!r})")

    blocks = raw[128:].view("<u2").reshape(-1, 4)
    c565 = blocks[:, :2].astype(np.uint32)
    ep = np.empty((len(blocks), 2, 3), np.float32)          # endpoint RGB888
    ep[:, :, 0] = ((c565 >> 11) & 0x1F) * (255.0 / 31.0)
    ep[:, :, 1] = ((c565 >> 5) & 0x3F) * (255.0 / 63.0)
    ep[:, :, 2] = (c565 & 0x1F) * (255.0 / 31.0)
    opaque = (c565[:, 0] > c565[:, 1])[:, None]
    pal = np.empty((len(blocks), 4, 3), np.float32)
    pal[:, 0] = ep[:, 0]
    pal[:, 1] = ep[:, 1]
    pal[:, 2] = np.where(opaque, (2 * ep[:, 0] + ep[:, 1]) / 3, (ep[:, 0] + ep[:, 1]) / 2)
    pal[:, 3] = np.where(opaque, (ep[:, 0] + 2 * ep[:, 1]) / 3, 0.0)
    idx_bits = blocks[:, 2].astype(np.uint32) | (blocks[:, 3].astype(np.uint32) << 16)
    shifts = np.arange(16, dtype=np.uint32) * 2
    idx = (idx_bits[:, None] >> shifts[None, :]) & 0x3      # N×16 texel indices
    texels = pal[np.arange(len(blocks))[:, None], idx]      # N×16×3

    bw, bh = width // 4, height // 4
    faces = texels.reshape(-1, bh, bw, 4, 4, 3).transpose(0, 1, 3, 2, 4, 5)
    return faces.reshape(-1, height, width, 3).copy(), caps2


def _cube_dirs(h: int, w: int):
    """Unit view directions for each cubemap texel, D3D/DDS face order."""
    import numpy as np

    vv, uu = np.meshgrid((np.arange(h) + 0.5) / h * 2 - 1,
                         (np.arange(w) + 0.5) / w * 2 - 1, indexing="ij")
    one = np.ones_like(uu)
    faces = [(one, -vv, -uu), (-one, -vv, uu),       # +X, -X
             (uu, one, vv), (uu, -one, -vv),         # +Y, -Y
             (uu, -vv, one), (-uu, -vv, -one)]       # +Z, -Z
    d = np.stack([np.stack(f, axis=-1) for f in faces]).astype(np.float32)
    return d / np.linalg.norm(d, axis=-1, keepdims=True)


def _thin_clouds(faces, density: float):
    """Fade the skybox's baked cloud layer toward a clear-sky gradient.

    Harmonic's ogre2 sky is a static cubemap — <clouds><humidity> is a dead SDF
    param — so cloud density is implemented here in texture space. A single
    GLOBAL elevation→color gradient is fitted from the saturated-blue sky
    texels across all faces (seamless by construction — a per-face fill shows
    seams and goes gray on mostly-cloudy faces), then every texel's deviation
    from it (bright cloud AND dark cloud-shadow mottling) is scaled by
    `density`. 1.0 = stock clouds, 0.0 ≈ smooth clear sky. The sun glow and
    dark ground silhouettes are protected.
    """
    import numpy as np

    d = min(1.0, max(0.0, density))
    if d >= 0.999:
        return faces

    n, h, w = faces.shape[:3]
    mx = faces.max(axis=3)
    mn = faces.min(axis=3)
    sat = (mx - mn) / np.maximum(mx, 1.0)
    v = mx / 255.0
    blue = faces[..., 2] >= mx - 1.0

    # Texel elevation: up axis = opposite of the darkest (ground) face's axis.
    dirs = _cube_dirs(h, w)
    axes = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0],
                     [0, -1, 0], [0, 0, 1], [0, 0, -1]], np.float32)
    ground = int(np.argmin(mx.reshape(n, -1).mean(axis=1)))
    up = -axes[ground]
    elev = dirs @ up                                   # [n,h,w] in [-1,1]

    # Global clear-sky ramp: mean sky color per elevation bin (interpolated
    # across cloud-only bins, lightly smoothed).
    sky = (sat > 0.26) & blue & (v > 0.30) & (elev > 0.03)
    nbins = 96
    bin_idx = np.clip(((elev + 1.0) * 0.5 * nbins).astype(np.int32), 0, nbins - 1)
    flat_idx = bin_idx[sky]
    counts = np.bincount(flat_idx, minlength=nbins)
    ramp = np.empty((nbins, 3), np.float32)
    for c in range(3):
        sums = np.bincount(flat_idx, weights=faces[..., c][sky], minlength=nbins)
        ramp[:, c] = np.divide(sums, counts, out=np.zeros(nbins, np.float32),
                               where=counts > 0)
    good = counts >= 50
    if not good.any():
        return faces                                   # no sky found — keep stock
    centers = np.arange(nbins) + 0.5
    centers_elev = centers / nbins * 2.0 - 1.0
    clear = np.empty(faces.shape, np.float32)
    for c in range(3):
        ramp[:, c] = np.interp(centers, centers[good], ramp[good, c])
        ramp[1:-1, c] = (ramp[:-2, c] + ramp[1:-1, c] + ramp[2:, c]) / 3.0
        # Continuous evaluation — nearest-bin lookup leaves faint concentric
        # elevation rings on the zenith.
        clear[..., c] = np.interp(elev.ravel(), centers_elev,
                                  ramp[:, c]).reshape(elev.shape)

    # Density = cloud COVERAGE, not opacity: keep the strongest `d` fraction of
    # cloud energy at full contrast and replace the rest with the clear
    # gradient. (Uniformly fading deviations instead leaves flat ghost patches
    # at low density — tried, looks unnatural.) Cloud strength = smoothed
    # absolute deviation from the gradient; threshold at the energy quantile.
    def blur(img, radius, passes=2):
        for _ in range(passes):
            for axis in (1, 2):
                m = img.shape[axis]
                cs = np.cumsum(img, axis=axis, dtype=np.float64)
                cs = np.concatenate([np.zeros_like(np.take(cs, [0], axis=axis)), cs],
                                    axis=axis)
                idx = np.arange(m)
                lo = np.clip(idx - radius, 0, m)
                hi = np.clip(idx + radius + 1, 0, m)
                shape = [1, 1, 1]
                shape[axis] = m
                img = ((np.take(cs, hi, axis=axis) - np.take(cs, lo, axis=axis))
                       / (hi - lo).reshape(shape))
        return img

    dev = faces.mean(axis=3) - clear.mean(axis=3)
    strength = blur(np.abs(dev), 12)
    region = elev > 0.03
    vals = np.sort(strength[region])[::-1]
    total = vals.sum()
    if total <= 1e-6:
        return faces
    cut = np.searchsorted(np.cumsum(vals), d * total)
    t = vals[min(cut, len(vals) - 1)]
    keep = np.clip((strength - 0.85 * t) / (0.30 * max(t, 1e-6)), 0.0, 1.0)
    a = 1.0 - keep
    a *= 1.0 - np.clip((v - 0.78) / 0.14, 0.0, 1.0)    # keep the sun
    a *= np.clip((v - 0.25) / 0.10, 0.0, 1.0)          # keep dark ground
    a *= np.clip(elev / 0.05, 0.0, 1.0)                # keep below-horizon
    a = a[..., None]
    return np.clip(faces * (1.0 - a) + clear * a, 0, 255)


def _write_argb_cubemap(dst_path: str, faces, caps2: int) -> None:
    """Write float [n,h,w,3] faces as an uncompressed A8R8G8B8 cubemap DDS.

    Uncompressed on purpose: re-quantizing a dimmed sky into DXT1's 5/6-bit
    endpoints causes severe banding.
    """
    import numpy as np
    import struct as _struct

    faces = np.clip(np.rint(faces), 0, 255).astype(np.uint8)
    n, height, width = faces.shape[:3]
    DDSD = 0x1 | 0x2 | 0x4 | 0x1000 | 0x8                   # caps|h|w|pf|pitch
    pf = _struct.pack("<2I4s5I", 32, 0x41, b"\0\0\0\0", 32,  # DDPF_RGB|ALPHA
                      0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000)
    header = (b"DDS " + _struct.pack("<7I", 124, DDSD, height, width,
                                     width * 4, 0, 0) + b"\0" * 44 + pf +
              _struct.pack("<4I", 0x1008, caps2, 0, 0) + b"\0" * 4)
    assert len(header) == 128
    bgra = np.empty((n, height, width, 4), np.uint8)
    bgra[..., 0] = faces[..., 2]
    bgra[..., 1] = faces[..., 1]
    bgra[..., 2] = faces[..., 0]
    bgra[..., 3] = 255
    with open(dst_path, "wb") as f:
        f.write(header)
        f.write(bgra.tobytes())


def ensure_sky_media(brightness: float | None,
                     cloud_density: float | None = None) -> str | None:
    """Build (and cache) a GZ_RENDERING_RESOURCE_PATH tree with a custom skybox.

    Gazebo Harmonic's ogre2 sky is the static cubemap
    <gz-rendering media>/ogre2/media/materials/textures/skybox.dds; nothing in
    SDF can dim it or change its cloud cover (see _sky_vars / _thin_clouds).
    This mirrors the stock media tree via symlinks into
    aeroloop_gazebo/media_cache/ (gitignored), materializing only a transformed
    skybox.dds (cloud thinning by `cloud_density`, then brightness/warm
    scaling), and returns the tree root to export as
    GZ_RENDERING_RESOURCE_PATH. Returns None when both knobs are stock
    (brightness 1.0, density 1.0). Brightness quantizes to 5% steps, density to
    10%, so repeated launches reuse the cache.
    """
    b = DEFAULT_SKY_BRIGHTNESS if brightness is None else float(brightness)
    b = min(1.0, max(_SKY_BRIGHTNESS_MIN, b))
    key = int(round(b * 20)) * 5          # percent, 5% steps
    d = 1.0 if cloud_density is None else min(1.0, max(0.0, float(cloud_density)))
    dkey = int(round(d * 10)) * 10        # percent, 10% steps
    if key >= 100 and dkey >= 100:
        return None

    # Source = the stock gz media tree. An inherited GZ_RENDERING_RESOURCE_PATH
    # is honoured as source ONLY if it isn't one of our own override trees —
    # otherwise a rebuilt override would try to transform its own (uncompressed)
    # output as input.
    src_root = os.environ.get("GZ_RENDERING_RESOURCE_PATH")
    cache_base = os.path.abspath(os.path.join(AEROLOOP_HOME, "media_cache"))
    if src_root and (not os.path.isdir(src_root)
                     or os.path.abspath(src_root).startswith(cache_base)):
        src_root = None
    if not src_root:
        candidates = sorted(glob.glob("/usr/share/gz/gz-rendering*"))
        src_root = candidates[-1] if candidates else None
    skybox_rel = os.path.join("ogre2", "media", "materials", "textures", "skybox.dds")
    if not src_root or not os.path.isfile(os.path.join(src_root, skybox_rel)):
        log.warning("gz-rendering media not found — sky brightness limited to "
                    "sun/background dimming (skybox stays stock)")
        return None

    src_skybox = os.path.join(src_root, skybox_rel)
    dst_root = os.path.join(AEROLOOP_HOME, "media_cache",
                            f"gz_rendering_b{key:03d}_c{dkey:03d}")
    dst_skybox = os.path.join(dst_root, skybox_rel)
    src_stat = os.stat(src_skybox)
    stamp = f"v9 {src_root} {src_stat.st_size} {src_stat.st_mtime_ns} b{key} c{dkey}"
    stamp_file = os.path.join(dst_root, ".skybox_stamp")
    try:
        if os.path.isfile(dst_skybox) and open(stamp_file).read() == stamp:
            return dst_root
    except OSError:
        pass

    # Mirror the stock tree as symlinks (real dirs, linked files) so every
    # shader/material resolves; only skybox.dds is a real, transformed file.
    if os.path.isdir(dst_root):
        shutil.rmtree(dst_root)
    shutil.copytree(src_root, dst_root, symlinks=True,
                    copy_function=lambda s, d: os.symlink(os.path.abspath(s), d))
    os.remove(dst_skybox)

    import numpy as np
    faces, caps2 = _dxt1_faces(src_skybox)
    faces = _thin_clouds(faces, dkey / 100.0)
    frac = key / 100.0
    w = 1.0 - frac
    intensity = 0.35 + 0.65 * frac        # match _sky_vars' light floor
    # ogre2's PBR pipeline decodes the cubemap as sRGB and tone-maps, so an
    # on-screen ratio R needs a texture-space factor of ~R^2.2 (measured:
    # texture 0.48 → rendered 0.71 ≈ 0.48^(1/2.2)). The warm shift is kept
    # subtle so the sky stays bluish instead of turning olive.
    faces = faces * np.asarray((
        intensity ** 2.2,                 # red kept highest — warm shift
        (intensity * (1.0 - 0.10 * w)) ** 2.2,
        (intensity * (1.0 - 0.18 * w)) ** 2.2,
    ), np.float32)
    _write_argb_cubemap(dst_skybox, faces, caps2)
    with open(stamp_file, "w") as f:
        f.write(stamp)
    log.info("Sky brightness %d%% / cloud density %d%%: built skybox override %s",
             key, dkey, dst_root)
    return dst_root


def apply_terrain_theme(theme: str | None) -> None:
    """Copy the terrain theme's texture set over the live baylands textures.

    No-op per file when the live texture already matches (content compare), so
    repeated launches with the same theme don't rewrite ~10 MB of PNGs. Unknown
    themes fall back to the default (desert).
    """
    theme = theme if theme in TERRAIN_THEMES else DEFAULT_TERRAIN_THEME
    tex_dir = os.path.join(AEROLOOP_HOME, "models", "baylands_terrain",
                           "media", "Textures")
    theme_dir = os.path.join(tex_dir, "themes", theme)
    if not os.path.isdir(theme_dir):
        log.warning("Terrain theme dir missing, keeping current textures: %s",
                    theme_dir)
        return
    for name in _TERRAIN_TEXTURE_NAMES:
        src = os.path.join(theme_dir, name)
        dst = os.path.join(tex_dir, name)
        if not os.path.isfile(src):
            log.warning("Terrain theme '%s' is missing %s — skipped", theme, name)
            continue
        if os.path.isfile(dst) and filecmp.cmp(src, dst, shallow=False):
            continue
        shutil.copyfile(src, dst)
        log.info("Terrain theme '%s': applied %s", theme, name)


# ── Jinja2 Template Rendering ─────────────────────────────────────────────────

def render_template(j2_path: str, variables: dict) -> None:
    """Render a Jinja2 template (.j2) to the corresponding output file."""
    from jinja2 import Environment, FileSystemLoader

    template_dir = os.path.dirname(j2_path)
    template_name = os.path.basename(j2_path)
    output_path = j2_path[:-3]  # strip ".j2"
    env = Environment(loader=FileSystemLoader(template_dir), keep_trailing_newline=True)
    template = env.get_template(template_name)
    rendered = template.render(**variables)
    with open(output_path, "w") as f:
        f.write(rendered)
    log.info("Rendered %s", os.path.relpath(output_path, AEROLOOP_HOME))


# Fisheye lens custom-function (Gazebo wideanglecamera): r = c1*f*fun(theta/c2 + c3).
# scale_to_hfov=true makes f auto and cx/cy = image centre, so only c1/c2/c3/fun are
# tuned. `fun` is one of LENS_FUNS. Presets seed (c1, c2, c3, fun); LENS_DEFAULT is the
# prior hardcoded "custom" value (so the default render is byte-for-byte unchanged).
LENS_FUNS = ("tan", "sin", "id")
LENS_DEFAULT = (1.05, 4.0, 0.0, "tan")
LENS_PRESETS = {
    "stereographic": (1.0, 2.0, 0.0, "tan"),
    "equidistant":   (1.0, 1.0, 0.0, "id"),
    "equisolid":     (1.0, 2.0, 0.0, "sin"),
    "orthographic":  (1.0, 1.0, 0.0, "sin"),
}


def sanitize_lens_fun(v) -> str:
    """Clamp a lens mapping-function name to a valid Gazebo value (else 'tan')."""
    return v if v in LENS_FUNS else "tan"


# Per-camera fisheye lens intrinsics shared by both launchers (start.py / start_px4.py).
_LENS_CAMERAS = (
    ("tracker-wide",   "tracker_wide",   "Wide tracker"),
    ("tracker-narrow", "tracker_narrow", "Narrow tracker"),
    ("thermal",        "thermal",        "Thermal"),
    ("utility",        "utility",        "Utility"),
)


def add_lens_args(group) -> None:
    """Add --<cam>-lens-c1/c2/c3/fun CLI args for every fisheye-capable camera."""
    c1d, c2d, c3d, fund = LENS_DEFAULT
    for flag, _dest, label in _LENS_CAMERAS:
        group.add_argument(f"--{flag}-lens-c1", type=float, default=c1d,
                           help=f"{label} fisheye lens c1 gain — r=c1*f*fun(theta/c2+c3) (default: {c1d})")
        group.add_argument(f"--{flag}-lens-c2", type=float, default=c2d,
                           help=f"{label} fisheye lens c2 angle scale (default: {c2d})")
        group.add_argument(f"--{flag}-lens-c3", type=float, default=c3d,
                           help=f"{label} fisheye lens c3 angle offset (default: {c3d})")
        group.add_argument(f"--{flag}-lens-fun", type=str, default=fund, choices=LENS_FUNS,
                           help=f"{label} fisheye lens mapping function (default: {fund})")


def lens_kwargs_from_args(args) -> dict:
    """Collect the per-camera fisheye lens intrinsics from parsed CLI args."""
    c1d, c2d, c3d, fund = LENS_DEFAULT
    out = {}
    for _flag, dest, _label in _LENS_CAMERAS:
        out[f"{dest}_lens_c1"] = getattr(args, f"{dest}_lens_c1", c1d)
        out[f"{dest}_lens_c2"] = getattr(args, f"{dest}_lens_c2", c2d)
        out[f"{dest}_lens_c3"] = getattr(args, f"{dest}_lens_c3", c3d)
        out[f"{dest}_lens_fun"] = getattr(args, f"{dest}_lens_fun", fund)
    return out


def compute_model_vars(
    drone: str,
    ctw: float | None = None,
    cam_pitch: float = -80.0,
    standoff: float | None = None,
    damping_overrides: dict | None = None,
    pilot_cam_enabled: bool = True,
    fpv_hfov_deg: float = 114.6,
    fpv_vfov_deg: float = 98.9,
    fpv_cam_width: int = 640,
    tracker_wide_cam_enabled: bool = True,
    tracker_wide_cam_pitch: float = -80.0,
    tracker_wide_cam_roll: float = 0.0,
    tracker_wide_hfov_deg: float = 114.6,
    tracker_wide_vfov_deg: float = 98.9,
    tracker_wide_cam_width: int = 640,
    tracker_wide_cam_fps: int = 30,
    tracker_wide_fisheye: bool = True,
    tracker_narrow_enabled: bool = False,
    tracker_narrow_cam_pitch: float = -80.0,
    tracker_narrow_cam_roll: float = 0.0,
    tracker_narrow_hfov_deg: float = 45.0,
    tracker_narrow_vfov_deg: float = 34.0,
    tracker_narrow_cam_width: int = 640,
    tracker_narrow_cam_fps: int = 30,
    tracker_narrow_fisheye: bool = False,
    thermal_cam_enabled: bool = False,
    thermal_cam_pitch: float = -80.0,
    thermal_cam_roll: float = 0.0,
    thermal_hfov_deg: float = 114.6,
    thermal_vfov_deg: float = 98.9,
    thermal_cam_width: int = 640,
    thermal_cam_fps: int = 30,
    thermal_fisheye: bool = False,
    utility_cam_enabled: bool = False,
    utility_cam_pitch: float = -80.0,
    utility_cam_roll: float = 0.0,
    utility_hfov_deg: float = 114.6,
    utility_vfov_deg: float = 98.9,
    utility_cam_width: int = 640,
    utility_cam_fps: int = 30,
    utility_fisheye: bool = True,
    # Fisheye lens custom-function intrinsics (r = c1*f*fun(theta/c2 + c3); f auto via
    # scale_to_hfov). Per fisheye-capable camera; defaults = the prior hardcoded values.
    tracker_wide_lens_c1: float = 1.05,
    tracker_wide_lens_c2: float = 4.0,
    tracker_wide_lens_c3: float = 0.0,
    tracker_wide_lens_fun: str = "tan",
    tracker_narrow_lens_c1: float = 1.05,
    tracker_narrow_lens_c2: float = 4.0,
    tracker_narrow_lens_c3: float = 0.0,
    tracker_narrow_lens_fun: str = "tan",
    thermal_lens_c1: float = 1.05,
    thermal_lens_c2: float = 4.0,
    thermal_lens_c3: float = 0.0,
    thermal_lens_fun: str = "tan",
    utility_lens_c1: float = 1.05,
    utility_lens_c2: float = 4.0,
    utility_lens_c3: float = 0.0,
    utility_lens_fun: str = "tan",
    chase_cam_enabled: bool = False,
) -> dict:
    """Compute model template variables from drone ref and overrides.

    Returns a dict suitable for rendering model SDF templates.
    """
    ref = DRONE_REFS[drone]
    _ctw = ctw if ctw is not None else ref["default_ctw"]
    mass = ref["max_thrust"] / (_ctw * 9.81)
    ixx = ref["ixx_per_kg"] * mass
    iyy = ref["iyy_per_kg"] * mass
    izz = ref["izz_per_kg"] * mass

    _standoff = standoff if standoff is not None else ref["default_standoff"]
    leg_z = -(_standoff / 2 + ref["leg_attach_offset"])

    fpv_cam_pitch_rad = math.radians(cam_pitch)
    tracker_wide_cam_pitch_rad = math.radians(tracker_wide_cam_pitch)
    tracker_wide_cam_roll_rad = math.radians(tracker_wide_cam_roll)
    tracker_narrow_cam_pitch_rad = math.radians(tracker_narrow_cam_pitch)
    tracker_narrow_cam_roll_rad = math.radians(tracker_narrow_cam_roll)

    # Derive source heights from HFOV/VFOV (stretched to output later).
    # Source heights derive from HFOV/VFOV; fisheye vs rectilinear is per-camera (*_fisheye).
    fpv_hfov_rad = math.radians(fpv_hfov_deg)
    tracker_wide_hfov_rad = math.radians(tracker_wide_hfov_deg)
    tracker_narrow_hfov_rad = math.radians(tracker_narrow_hfov_deg)
    fpv_img_width = max(64, int(fpv_cam_width))
    tracker_wide_img_width = max(64, int(tracker_wide_cam_width))
    tracker_narrow_img_width = max(64, int(tracker_narrow_cam_width))
    fpv_img_height = round(fpv_img_width * math.tan(math.radians(fpv_vfov_deg) / 2)
                           / math.tan(fpv_hfov_rad / 2))
    tracker_wide_img_height = round(tracker_wide_img_width * math.tan(math.radians(tracker_wide_vfov_deg) / 2)
                                    / math.tan(tracker_wide_hfov_rad / 2))
    tracker_narrow_img_height = round(tracker_narrow_img_width * math.tan(math.radians(tracker_narrow_vfov_deg) / 2)
                                      / math.tan(tracker_narrow_hfov_rad / 2))

    # Thermal: optional sensor (fisheye/rectilinear per thermal_fisheye); white-hot downstream.
    thermal_cam_pitch_rad = math.radians(thermal_cam_pitch)
    thermal_cam_roll_rad = math.radians(thermal_cam_roll)
    thermal_hfov_rad = math.radians(thermal_hfov_deg)
    thermal_img_width = max(64, int(thermal_cam_width))
    thermal_img_height = round(thermal_img_width * math.tan(math.radians(thermal_vfov_deg) / 2)
                               / math.tan(thermal_hfov_rad / 2))

    # Utility camera: clone of the wide tracker (fisheye/rectilinear per utility_fisheye).
    utility_cam_pitch_rad = math.radians(utility_cam_pitch)
    utility_cam_roll_rad = math.radians(utility_cam_roll)
    utility_hfov_rad = math.radians(utility_hfov_deg)
    utility_img_width = max(64, int(utility_cam_width))
    utility_img_height = round(utility_img_width * math.tan(math.radians(utility_vfov_deg) / 2)
                               / math.tan(utility_hfov_rad / 2))

    dd = ref["default_damping"]
    do = damping_overrides or {}
    model_vars = {
        "mass": mass, "ixx": ixx, "iyy": iyy, "izz": izz,
        # Pilot (FPV) camera — rectilinear
        "pilot_cam_enabled": bool(pilot_cam_enabled),
        "fpv_cam_pitch_rad": fpv_cam_pitch_rad,
        "fpv_hfov_rad": fpv_hfov_rad,
        "fpv_img_width": fpv_img_width,
        "fpv_img_height": fpv_img_height,
        # Tracker WIDE camera (fisheye by default; tracker_wide_fisheye)
        "tracker_wide_cam_enabled": bool(tracker_wide_cam_enabled),
        "tracker_wide_cam_pitch_rad": tracker_wide_cam_pitch_rad,
        "tracker_wide_cam_roll_rad": tracker_wide_cam_roll_rad,
        "tracker_wide_hfov_rad": tracker_wide_hfov_rad,
        "tracker_wide_img_width": tracker_wide_img_width,
        "tracker_wide_img_height": tracker_wide_img_height,
        "tracker_wide_cam_fps": max(1, int(tracker_wide_cam_fps)),
        "tracker_wide_fisheye": bool(tracker_wide_fisheye),
        # Tracker NARROW camera (rectilinear by default; tracker_narrow_fisheye), narrower FOV
        "tracker_narrow_enabled": bool(tracker_narrow_enabled),
        "tracker_narrow_cam_pitch_rad": tracker_narrow_cam_pitch_rad,
        "tracker_narrow_cam_roll_rad": tracker_narrow_cam_roll_rad,
        "tracker_narrow_hfov_rad": tracker_narrow_hfov_rad,
        "tracker_narrow_img_width": tracker_narrow_img_width,
        "tracker_narrow_img_height": tracker_narrow_img_height,
        "tracker_narrow_cam_fps": max(1, int(tracker_narrow_cam_fps)),
        "tracker_narrow_fisheye": bool(tracker_narrow_fisheye),
        # Thermal camera (rectilinear by default; thermal_fisheye), white-hot downstream
        "thermal_cam_enabled": bool(thermal_cam_enabled),
        "thermal_cam_pitch_rad": thermal_cam_pitch_rad,
        "thermal_cam_roll_rad": thermal_cam_roll_rad,
        "thermal_hfov_rad": thermal_hfov_rad,
        "thermal_img_width": thermal_img_width,
        "thermal_img_height": thermal_img_height,
        "thermal_cam_fps": max(1, int(thermal_cam_fps)),
        "thermal_fisheye": bool(thermal_fisheye),
        # Utility camera (clone of wide tracker; fisheye by default; utility_fisheye)
        "utility_cam_enabled": bool(utility_cam_enabled),
        "utility_cam_pitch_rad": utility_cam_pitch_rad,
        "utility_cam_roll_rad": utility_cam_roll_rad,
        "utility_hfov_rad": utility_hfov_rad,
        "utility_img_width": utility_img_width,
        "utility_img_height": utility_img_height,
        "utility_cam_fps": max(1, int(utility_cam_fps)),
        "utility_fisheye": bool(utility_fisheye),
        # Fisheye lens custom-function intrinsics (only used when *_fisheye is on)
        "tracker_wide_lens_c1": float(tracker_wide_lens_c1),
        "tracker_wide_lens_c2": float(tracker_wide_lens_c2),
        "tracker_wide_lens_c3": float(tracker_wide_lens_c3),
        "tracker_wide_lens_fun": sanitize_lens_fun(tracker_wide_lens_fun),
        "tracker_narrow_lens_c1": float(tracker_narrow_lens_c1),
        "tracker_narrow_lens_c2": float(tracker_narrow_lens_c2),
        "tracker_narrow_lens_c3": float(tracker_narrow_lens_c3),
        "tracker_narrow_lens_fun": sanitize_lens_fun(tracker_narrow_lens_fun),
        "thermal_lens_c1": float(thermal_lens_c1),
        "thermal_lens_c2": float(thermal_lens_c2),
        "thermal_lens_c3": float(thermal_lens_c3),
        "thermal_lens_fun": sanitize_lens_fun(thermal_lens_fun),
        "utility_lens_c1": float(utility_lens_c1),
        "utility_lens_c2": float(utility_lens_c2),
        "utility_lens_c3": float(utility_lens_c3),
        "utility_lens_fun": sanitize_lens_fun(utility_lens_fun),
        # Chase camera — rectilinear, 3rd-person
        "chase_cam_enabled": bool(chase_cam_enabled),
        "standoff_height": _standoff, "leg_z": leg_z,
        "linear_damping_x": do.get("linear_x", dd["linear_x"]),
        "linear_damping_y": do.get("linear_y", dd["linear_y"]),
        "linear_damping_z": do.get("linear_z", dd["linear_z"]),
        "quadratic_damping_x": do.get("quadratic_x", dd["quadratic_x"]),
        "quadratic_damping_y": do.get("quadratic_y", dd["quadratic_y"]),
        "quadratic_damping_z": do.get("quadratic_z", dd["quadratic_z"]),
        "angular_damping": do.get("angular", dd["angular"]),
    }

    log.info("CTW=%.1f mass=%.3fkg Ixx=%.6f Iyy=%.6f Izz=%.6f standoff=%.3fm cam_pitch=%.1f°",
             _ctw, mass, ixx, iyy, izz, _standoff, cam_pitch)
    log.info("Cameras: pilot=%s wide=%s narrow=%s thermal=%s utility=%s chase=%s",
             pilot_cam_enabled, tracker_wide_cam_enabled, tracker_narrow_enabled,
             thermal_cam_enabled, utility_cam_enabled, chase_cam_enabled)
    log.info("Fisheye: wide=%s narrow=%s thermal=%s utility=%s",
             tracker_wide_fisheye, tracker_narrow_fisheye, thermal_fisheye, utility_fisheye)
    log.info("Camera source sizes: FPV %dx%d, wide %dx%d, narrow %dx%d",
             fpv_img_width, fpv_img_height, tracker_wide_img_width, tracker_wide_img_height,
             tracker_narrow_img_width, tracker_narrow_img_height)
    return model_vars


# Per-world default target altitude (m) — single source of truth shared by
# compute_world_vars (launch spawn + static-target GT) and reset_world.py
# (RC/SPACE reset), so a bare reset puts the target at the same altitude the bare
# launch spawned it. Worlds not listed (collision_test, balloon_test) default to 10.
DEFAULT_TARGET_ALTITUDE = {
    "patrol_park": 100.0,
    "park_chase": 50.0,
    "moving_target": 50.0,
}


def default_target_altitude(world_name: str) -> float:
    """Per-world default target spawn altitude in metres (default 10)."""
    return DEFAULT_TARGET_ALTITUDE.get(world_name, 10.0)


def compute_world_vars(
    drone: str,
    world_name: str,
    target_altitude: float | None = None,
    target_speed: float | None = None,
    orbit_radius: float | None = None,
    orbit_center_x: float | None = None,
    orbit_center_y: float | None = None,
    orbit_theta_deg: float | None = None,
    patrol_length: float | None = None,
    target_x: float | None = None,
    target_y: float | None = None,
    clouds: bool = True,
    cloud_density: float = 0.7,
    pedestal_radius: float | None = None,
    pedestal_height: float | None = None,
    target_drone: str = DEFAULT_TARGET_DRONE,
    target_scale: float | None = None,
    traj_type: str | None = None,
    traj_rotation_deg: float | None = None,
    traj_offset_ew: float | None = None,
    traj_offset_ns: float | None = None,
    oval_ew_len: float | None = None,
    oval_ns_len: float | None = None,
    corner_radius: float | None = None,
    traj_start_pos: float | None = None,
    traj_reverse: bool = False,
    player_heading_deg: float | None = None,
    terrain_theme: str | None = None,
    sky_brightness: float | None = None,
) -> dict:
    """Compute world template variables from drone and world settings.

    Returns a dict suitable for rendering world SDF templates.
    """
    ref = DRONE_REFS[drone]
    tref = TARGET_REFS.get(target_drone, TARGET_REFS[DEFAULT_TARGET_DRONE])

    # Terrain theme (desert / lush): ground-plane tint here; the matching
    # baylands texture swap happens in render_vis_templates → apply_terrain_theme.
    _theme = terrain_theme if terrain_theme in TERRAIN_THEMES else DEFAULT_TERRAIN_THEME
    _ground_color = TERRAIN_THEMES[_theme]["ground_color"]

    # Target uniform scale. A single --target-scale multiplier applies to ANY
    # target; when unset each target uses its own default (shahed 1.0, stingjet
    # 0.1). The proximity bbox scales with the mesh so TARGET REACHED stays right.
    _tscale = float(target_scale) if target_scale is not None else float(tref["default_scale"])
    _tscale_factor = _tscale / float(tref["base_scale"])
    _target_scale_str = f"{_tscale:g} {_tscale:g} {_tscale:g}"
    _target_bbox = ",".join(f"{float(c) * _tscale_factor:g}" for c in tref["bbox"].split(","))
    _target_model_name = tref["model_uri"].replace("model://", "")

    # Primitive targets (e.g. balloon) are drawn as a <sphere> inline instead of
    # a <mesh>; worlds branch on `target_primitive`. The sphere radius scales with
    # the same factor as a mesh would.
    _target_primitive = tref.get("primitive", "")
    _target_radius = f"{float(tref.get('radius', 0.5)) * _tscale_factor:g}"
    _target_color = tref.get("color", "0.9 0.1 0.1")

    # Pedestal launch-stand dimensions (vis-only cylinder under the drone).
    # Defaults mirror the Simulink model_parameters (pedestal_radius=0.5,
    # pedestal_height=0.30); leaf-sim-ui overrides them from the per-drone
    # simulink params so the rendered cylinder matches the physics pedestal.
    _pedestal_radius = pedestal_radius if pedestal_radius is not None else 0.5
    _pedestal_height = pedestal_height if pedestal_height is not None else 0.30

    _orbit_radius = orbit_radius if orbit_radius is not None else 30.0
    _orbit_cx = orbit_center_x if orbit_center_x is not None else 0.0
    _orbit_cy = orbit_center_y if orbit_center_y is not None else 0.0
    _orbit_theta_deg = orbit_theta_deg if orbit_theta_deg is not None else 0.0
    _orbit_theta_rad = math.radians(_orbit_theta_deg)
    speed_kmh = target_speed if target_speed is not None else 5.4
    orbit_speed = (speed_kmh / 3.6) / _orbit_radius

    _patrol_length = patrol_length if patrol_length is not None else 500.0
    patrol_speed_kmh = target_speed if target_speed is not None else 20.0
    patrol_speed_ms = patrol_speed_kmh / 3.6

    _target_x = target_x if target_x is not None else 30.0
    _target_y = target_y if target_y is not None else 0.0
    _target_z = (target_altitude if target_altitude is not None
                 else default_target_altitude(world_name))

    # Parametric trajectory spawn pose (moving_target world): place the target
    # model at s=0 of the trajectory so it appears where the thread first drives
    # it. None args fall back to per-shape defaults (mirrors trajectory_sample).
    _def = lambda v, d: v if v is not None else d
    _player_heading = _def(player_heading_deg, 0.0)
    _ew, _ns, _cr = resolve_loop_geom(traj_type, oval_ew_len, oval_ns_len, corner_radius)
    _spawn_x, _spawn_y, _spawn_yaw = trajectory_start_pose(
        rotation_deg=_def(traj_rotation_deg, 0.0),
        offset_ew=_def(traj_offset_ew, 0.0),
        offset_ns=_def(traj_offset_ns, 0.0),
        oval_ew_len=_ew, oval_ns_len=_ns, corner_radius=_cr,
        start_pos=_def(traj_start_pos, 0.0), reverse=bool(traj_reverse),
    )

    return {
        "drone_uri": ref["model_uri"],
        "drone_vis_uri": ref["model_vis_uri"],
        "drone_name": drone,
        "orbit_speed": orbit_speed,
        "orbit_radius": _orbit_radius,
        "orbit_center_x": _orbit_cx,
        "orbit_center_y": _orbit_cy,
        "orbit_theta_deg": _orbit_theta_deg,
        "orbit_theta_rad": _orbit_theta_rad,
        "orbit_spawn_x": _orbit_cx + _orbit_radius * math.cos(_orbit_theta_rad),
        "orbit_spawn_y": _orbit_cy + _orbit_radius * math.sin(_orbit_theta_rad),
        "orbit_spawn_yaw": _orbit_theta_rad + math.pi / 2,
        "target_altitude": _target_z,
        "target_x": _target_x, "target_y": _target_y, "target_z": _target_z,
        # Parametric trajectory (moving_target world) spawn + player heading.
        "target_spawn_x": _spawn_x,
        "target_spawn_y": _spawn_y,
        "target_spawn_yaw": _spawn_yaw,
        "player_heading_rad": math.radians(_player_heading),
        "patrol_length": _patrol_length,
        "patrol_speed_ms": patrol_speed_ms,
        "clouds": clouds,
        "cloud_density": float(cloud_density),
        "terrain_theme": _theme,
        "ground_color": _ground_color,
        **_sky_vars(sky_brightness),
        "pedestal_radius": _pedestal_radius,
        "pedestal_height": _pedestal_height,
        "target_mesh_uri": tref["mesh_uri"],
        "target_model_uri": tref["model_uri"],
        "target_visual_pose": tref["visual_pose"],
        "target_scale": _target_scale_str,
        "target_bbox": _target_bbox,
        "target_model_name": _target_model_name,
        "target_primitive": _target_primitive,
        "target_radius": _target_radius,
        "target_color": _target_color,
    }


def render_vis_templates(
    drone: str,
    world_name: str,
    world_map: dict,
    model_vars: dict,
    world_vars: dict,
) -> None:
    """Render vis-only model + world templates (shared by BF and PX4 stacks)."""
    models_dir = os.path.join(AEROLOOP_HOME, "models")
    worlds_dir = os.path.join(AEROLOOP_HOME, "worlds")
    ref = DRONE_REFS[drone]

    # Swap the baylands ground textures to the selected terrain theme (the DAE
    # hardcodes the texture filenames, so themes are applied by file copy).
    apply_terrain_theme(world_vars.get("terrain_theme"))

    # Rebuild the ogre2 skybox cubemap for the requested sky brightness and
    # cloud density (SDF can't do either — see ensure_sky_media). The env var
    # is inherited by the gz sim processes. Failure (e.g. no numpy) degrades
    # to sun/background dimming only.
    try:
        sky_media = ensure_sky_media(world_vars.get("sky_brightness"),
                                     world_vars.get("cloud_density"))
    except Exception as exc:
        log.warning("skybox customisation unavailable (%s) — sky stays stock", exc)
        sky_media = None
    if sky_media:
        os.environ["GZ_RENDERING_RESOURCE_PATH"] = sky_media
    else:
        # Don't leak a stale override inherited from a previous launch.
        prev = os.environ.get("GZ_RENDERING_RESOURCE_PATH", "")
        cache_base = os.path.abspath(os.path.join(AEROLOOP_HOME, "media_cache"))
        if os.path.abspath(prev).startswith(cache_base):
            del os.environ["GZ_RENDERING_RESOURCE_PATH"]

    # Vis model SDF
    vis_j2 = os.path.join(models_dir, ref["model_vis_sdf"] + ".j2")
    if os.path.isfile(vis_j2):
        render_template(vis_j2, model_vars)

    # Vis world SDF
    wm = world_map[world_name]
    vis_wj2 = os.path.join(worlds_dir, wm["sim_world"] + ".j2")
    if os.path.isfile(vis_wj2):
        render_template(vis_wj2, world_vars)

    # Target model SDF (e.g. stingjet, whose scale is parameterised). shahed has
    # no template (fixed 1.0). Rendered with world_vars so `target_scale` applies.
    tgt_model = world_vars.get("target_model_name")
    if tgt_model:
        tgt_j2 = os.path.join(models_dir, tgt_model, "model.sdf.j2")
        if os.path.isfile(tgt_j2):
            render_template(tgt_j2, world_vars)


# ── Process Manager ───────────────────────────────────────────────────────────

class ProcessManager:
    """Track child processes for clean shutdown."""

    def __init__(self):
        self.procs: list[subprocess.Popen] = []

    def spawn(self, args, **kwargs):
        log.info("Starting: %s", " ".join(str(a) for a in args[:6]))
        p = subprocess.Popen(args, **kwargs)
        self.procs.append(p)
        return p

    def shutdown(self, extra_pkill_patterns: list[str] | None = None):
        log.info("Shutting down %d processes …", len(self.procs))
        for p in reversed(self.procs):
            if p.poll() is None:
                p.terminate()
        for p in reversed(self.procs):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                try:
                    p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
        # Safety net: kill orphaned simulation processes
        patterns = ["gz sim", "ruby.*gz", "gz_image_bridge"]
        if extra_pkill_patterns:
            patterns.extend(extra_pkill_patterns)
        for pattern in patterns:
            try:
                subprocess.run(
                    ["pkill", "-9", "-f", pattern],
                    capture_output=True, timeout=3,
                )
            except (subprocess.TimeoutExpired, OSError):
                pass


# ── Gazebo Environment ────────────────────────────────────────────────────────

def setup_gazebo_env():
    """Set Gazebo Harmonic environment variables."""

    def _prepend(var, *paths):
        existing = os.environ.get(var, "")
        os.environ[var] = os.pathsep.join(list(paths) + [existing])

    models = os.path.join(AEROLOOP_HOME, "models")
    plugins = os.path.join(AEROLOOP_HOME, "plugins", "build")
    worlds = os.path.join(AEROLOOP_HOME, "worlds")

    _prepend("SDF_PATH", models, "/usr/share/gz/gz-sim8/models")
    _prepend("GZ_SIM_RESOURCE_PATH", worlds, "/usr/share/gz/gz-sim8")
    _prepend("GZ_SIM_SYSTEM_PLUGIN_PATH", plugins,
             "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins")
    _prepend("LD_LIBRARY_PATH", "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins")

    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


# ── GPU & Display ─────────────────────────────────────────────────────────────

def has_nvidia_gpu() -> bool:
    """Check if an NVIDIA GPU is accessible."""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=5,
        )
        return result.returncode == 0 and result.stdout.strip() != ""
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def configure_display(args, pm: ProcessManager):
    """Set up GPU detection, rendering, and display (Xvfb if needed).

    *args* must have a ``.gazebo`` boolean attribute.
    """
    gpu_available = has_nvidia_gpu()

    if gpu_available:
        log.info("NVIDIA GPU detected — GPU-accelerated rendering")
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
        nvidia_icd = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.isfile(nvidia_icd):
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_icd
        # PRIME render offload — force GLX/OpenGL onto the NVIDIA GPU. On hybrid
        # (Optimus) laptops the X server on $DISPLAY runs on the Intel iGPU, and
        # OGRE2's GL3Plus backend renders via GLX on that display — so it lands on
        # Intel (measured ≈17 fps, RTX idle) even though the NVIDIA EGL device is
        # probed first. These route the GL context onto the RTX. Harmless on
        # single-NVIDIA hosts (GLX is already NVIDIA there).
        os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
        os.environ["__VK_LAYER_NV_optimus"] = "NVIDIA_only"
    else:
        log.info("No GPU detected — software rendering (llvmpipe)")
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    if args.gazebo:
        if not os.environ.get("DISPLAY"):
            log.error("No DISPLAY set — the Gazebo GUI needs a display.")
            sys.exit(1)
        # Keep the PRIME offload set above so the GUI also renders on the NVIDIA
        # GPU (its GL widget uses GLX just like the server's offscreen render).
        log.info("Using display %s for Gazebo GUI", os.environ["DISPLAY"])
    elif os.environ.get("DISPLAY"):
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
            log.error("Xvfb failed to start — camera rendering requires a display")
            sys.exit(1)
        os.environ["DISPLAY"] = ":99"


# ── Camera Topics ─────────────────────────────────────────────────────────────

def list_camera_topics(name_hint=None):
    """Return sorted camera image topics, optionally filtered by substring."""
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


def discover_camera_topic(name_hint="fpv_cam", timeout=30, model_hint=None):
    """Find a camera image topic matching *name_hint*."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        topics = list_camera_topics(name_hint=name_hint)
        if topics:
            if model_hint:
                preferred = [t for t in topics if model_hint in t]
                if preferred:
                    return preferred[0]
            return topics[0]
        time.sleep(2)
    return None


def read_image_meta(proc, timeout=30):
    """Read IMGMETA line from the bridge's stderr → (width, height, pix_fmt)."""
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


# ── Cleanup ───────────────────────────────────────────────────────────────────

def cleanup_before_start(extra_pkill_cmds: list[str] | None = None):
    """Kill stale processes from previous runs and remove orphaned resources."""
    log.info("Cleaning up from previous runs...")
    cmds = [
        "pkill -9 -f 'gz sim' 2>/dev/null || true",
        "pkill -9 -f gz_image_bridge 2>/dev/null || true",
        "pkill -9 Xvfb 2>/dev/null || true",
    ]
    if extra_pkill_cmds:
        cmds.extend(extra_pkill_cmds)
    for cmd in cmds:
        try:
            subprocess.run(["bash", "-c", cmd], capture_output=True, timeout=5)
        except (subprocess.TimeoutExpired, OSError):
            pass
        time.sleep(0.2)

    for path in glob.glob("/dev/shm/gz_cam_*"):
        try:
            os.remove(path)
            log.info("Removed stale SHM: %s", path)
        except OSError:
            pass
    log.info("Cleanup complete")


# ── TCP port wait ─────────────────────────────────────────────────────────────

def wait_for_port(host: str, port: int, timeout: float = 30) -> bool:
    """Block until a TCP port is accepting connections."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            s = socket.create_connection((host, port), timeout=1)
            s.close()
            return True
        except OSError:
            time.sleep(0.5)
    return False


# ── Video pipeline helpers ────────────────────────────────────────────────────

def start_fpv_bridge(args, pm: ProcessManager, osd_args=None):
    """Discover and start the FPV image bridge.

    Returns ``(proc, width, height)`` or ``(None, 0, 0)`` if ``--no-video``.
    *args* must have: ``no_video``, ``fpv_topic``, ``topic_model_hint``,
    ``no_display``.
    *osd_args*: optional list of OSD-related CLI flags for gz_image_bridge.
    When provided, replaces the default ``--no-osd``.
    """
    if args.no_video:
        log.info("Video pipeline disabled (--no-video)")
        return None, 0, 0

    if not getattr(args, "pilot_cam", True):
        log.info("Pilot camera disabled (--no-pilot-cam) — skipping FPV bridge")
        return None, 0, 0

    if not os.path.isfile(IMAGE_BRIDGE):
        log.error("gz_image_bridge not found: %s — run build_plugin.sh", IMAGE_BRIDGE)
        pm.shutdown()
        sys.exit(1)

    if args.fpv_topic:
        topic = args.fpv_topic
    else:
        log.info("Discovering FPV camera image topic …")
        topic = discover_camera_topic(
            name_hint="fpv_cam", timeout=30,
            model_hint=getattr(args, "topic_model_hint", TOPIC_MODEL_HINT_DEFAULT),
        )
        if not topic:
            log.error("Could not find FPV camera topic")
            pm.shutdown()
            sys.exit(1)
    log.info("FPV topic: %s", topic)

    # SHM + RTSP are always active; the SDL2 window (--display) is added ONLY
    # when not headless. --no-display runs the bridge truly headless (no SDL2
    # window/overhead, no SDL2 build dependency) — better performance.
    bridge_cmd = [IMAGE_BRIDGE, topic]
    cam_width = int(getattr(args, "fpv_cam_width", getattr(args, "cam_width", 640)))
    cam_height = int(getattr(args, "fpv_cam_height", getattr(args, "cam_height", 480)))
    bridge_cmd.extend(["--out-width", str(cam_width), "--out-height", str(cam_height)])
    if osd_args:
        bridge_cmd.extend(osd_args)
    else:
        bridge_cmd.append("--no-osd")
    bridge_cmd.append("--no-display" if args.no_display else "--display")

    proc = pm.spawn(bridge_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    flags = fcntl.fcntl(proc.stderr, fcntl.F_GETFL)
    fcntl.fcntl(proc.stderr, fcntl.F_SETFL, flags | os.O_NONBLOCK)

    log.info("Waiting for first camera frame …")
    width, height, pix_fmt = read_image_meta(proc, timeout=30)
    if width is None:
        log.error("No image metadata from bridge — camera may not be rendering")
        pm.shutdown()
        sys.exit(1)
    log.info("Camera: %dx%d %s", width, height, pix_fmt)

    # Drain stderr in a background thread so the pipe buffer never fills up
    # and blocks the child process (which would deadlock its main loop).
    def _drain(f):
        try:
            while True:
                chunk = f.read(4096)
                if not chunk:
                    break
        except Exception:
            pass

    t = threading.Thread(target=_drain, args=(proc.stderr,), daemon=True)
    t.start()

    return proc, width, height


def start_chase_bridge(args, pm: ProcessManager):
    """Discover and start the chase camera image bridge.

    Returns the process or ``None``.
    *args* must have: ``chase_cam``, ``chase_topic``, ``topic_model_hint``.
    """
    if not getattr(args, "chase_cam", False):
        return None

    if args.chase_topic:
        chase_topic = args.chase_topic
    else:
        chase_topic = discover_camera_topic(
            name_hint="chase_cam", timeout=30,
            model_hint=getattr(args, "topic_model_hint", TOPIC_MODEL_HINT_DEFAULT),
        )
    if not chase_topic:
        log.warning("Chase camera topic not found — FPV only")
        return None

    log.info("Chase topic: %s", chase_topic)
    chase_cmd = [IMAGE_BRIDGE, chase_topic, "--no-osd"]
    # Hardcoded 4:3 resolution, independent of FPV/tracker cam settings.
    chase_cmd.extend(["--out-width", "640", "--out-height", "480"])
    chase_cmd.append("--no-display" if getattr(args, "no_display", False) else "--display")
    return pm.spawn(chase_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _append_rtsp(cmd, rtsp_url, fps, bitrate, crf, preset, tune, width, height, label, gop=0):
    """Append RTSP H.264 push flags to a gz_image_bridge command (no-op if URL empty)."""
    if not rtsp_url:
        return
    cmd.extend([
        "--rtsp", str(rtsp_url),
        "--stream-fps", str(fps),
        "--stream-bitrate", str(bitrate),
        "--stream-crf", str(int(crf)),
        "--stream-gop", str(int(gop)),
        "--stream-preset", str(preset),
        "--stream-tune", str(tune),
    ])
    w = int(width or 0)
    h = int(height or 0)
    if w > 0 and h > 0:
        cmd.extend(["--stream-width", str(w), "--stream-height", str(h)])
    log.info("%s RTSP stream → %s (%sfps, %s, crf=%s, gop=%s, preset=%s, tune=%s, res=%s)",
             label, rtsp_url, fps, bitrate, crf, gop or "auto", preset, tune,
             f"{w}x{h}" if w > 0 and h > 0 else "camera")


def start_tracker_bridges(args, pm: ProcessManager):
    """Spawn the optional clean (no-OSD) tracker feeds — shared by BF + PX4 stacks.

    Spawns a gz_image_bridge per enabled sensor: the wide and narrow tracker
    cameras and the white-hot thermal camera (each fisheye or rectilinear).
    Each is gated by its ``--…-cam`` toggle; missing topics are skipped, not fatal.
    """
    if getattr(args, "no_video", False):
        return
    if not os.path.isfile(IMAGE_BRIDGE):
        log.error("gz_image_bridge not found: %s — run build_plugin.sh", IMAGE_BRIDGE)
        return

    model_hint = getattr(args, "topic_model_hint", TOPIC_MODEL_HINT_DEFAULT)
    disp = "--no-display" if getattr(args, "no_display", False) else "--display"

    # (attr-name, sensor name_hint, extra bridge flags, rtsp prefix, label)
    feeds = [
        ("tracker_wide_cam",   "fpv_tracker_wide_cam",   [],            "tracker_wide",   "Wide tracker"),
        ("tracker_narrow_cam", "fpv_tracker_narrow_cam", [],            "tracker_narrow", "Narrow tracker"),
        ("thermal_cam",        "fpv_thermal_cam",        ["--thermal"], "thermal",        "Thermal"),
        ("utility_cam",        "fpv_utility_cam",        [],            "utility",        "Utility"),
    ]
    for enable_attr, name_hint, extra_flags, rp, label in feeds:
        if not getattr(args, enable_attr, False):
            continue
        log.info("Discovering %s camera topic …", label.lower())
        topic = discover_camera_topic(name_hint=name_hint, timeout=30, model_hint=model_hint)
        if not topic:
            log.warning("%s camera topic not found — skipping (is the sensor in the model?)", label)
            continue
        log.info("Found %s camera topic: %s", label.lower(), topic)
        # Per-feed attrs are uniform: <enable_attr>_width/_height, <rp>_cam_fps, <rp>_rtsp*.
        out_w = int(getattr(args, f"{enable_attr}_width", 640))
        out_h = int(getattr(args, f"{enable_attr}_height", 480))
        cmd = [IMAGE_BRIDGE, topic, "--no-osd", *extra_flags,
               "--out-width", str(out_w), "--out-height", str(out_h), disp]
        _append_rtsp(cmd, getattr(args, f"{rp}_rtsp", None),
                     getattr(args, f"{rp}_cam_fps", 30),
                     getattr(args, f"{rp}_rtsp_bitrate", "4M"),
                     getattr(args, f"{rp}_rtsp_crf", 23),
                     getattr(args, f"{rp}_rtsp_preset", "ultrafast"),
                     getattr(args, f"{rp}_rtsp_tune", "zerolatency"),
                     getattr(args, f"{rp}_rtsp_width", 0),
                     getattr(args, f"{rp}_rtsp_height", 0), label,
                     gop=getattr(args, f"{rp}_rtsp_gop", 0))
        pm.spawn(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ── Target Trajectory Threads ─────────────────────────────────────────────────

# UDP ports — must match ExternalPosePlugin <listen_port> in world SDF files.
TARGET_UDP_PORT = 9016
TARGET_RESET_PORT = 9017
# Mirror port: every VisualPosePacket sent to TARGET_UDP_PORT (which Gazebo's
# ExternalPosePlugin owns exclusively) is also sent here so sitl_redis_bridge
# and any other observers can read ground-truth target pose without colliding
# with Gazebo on 9016.
TARGET_MIRROR_PORT = 9018


def start_orbit_thread(
    stop_event: threading.Event,
    orbit_radius: float = 30.0,
    orbit_omega: float = 0.05,
    target_z: float = 50.0,
    udp_port: int = TARGET_UDP_PORT,
    reset_port: int = TARGET_RESET_PORT,
    orbit_center_x: float = 0.0,
    orbit_center_y: float = 0.0,
    orbit_theta_deg: float = 0.0,
) -> threading.Thread:
    """Spawn a daemon thread that drives a circular orbit via UDP.

    Sends a 72-byte VisualPosePacket at ~60 Hz to ``udp_port``.
    Listens on ``reset_port`` for reset signals.
    Returns the started thread.
    """

    def _orbit_loop():
        interval = 1.0 / 60
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = ("127.0.0.1", udp_port)
        mirror_addr = ("127.0.0.1", TARGET_MIRROR_PORT)
        packer = struct.Struct("<Qd3d4d")
        seq = 0
        t0 = time.monotonic()
        theta = math.radians(orbit_theta_deg)
        theta0 = theta
        t_prev = t0

        rst_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rst_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rst_sock.bind(("0.0.0.0", reset_port))
        rst_sock.setblocking(False)

        log.info("Orbit thread: R=%.1fm omega=%.4f rad/s alt=%.0fm center=(%.1f,%.1f) theta0=%.1fdeg (port %d)",
                 orbit_radius, orbit_omega, target_z,
                 orbit_center_x, orbit_center_y, orbit_theta_deg, udp_port)

        while not stop_event.is_set():
            try:
                while True:
                    rst_sock.recv(64)
                    theta = theta0
                    t_prev = time.monotonic()
                    log.info("Orbit thread: reset to theta=%.1fdeg", orbit_theta_deg)
            except BlockingIOError:
                pass

            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now
            theta += orbit_omega * dt

            x = orbit_center_x + orbit_radius * math.cos(theta)
            y = orbit_center_y + orbit_radius * math.sin(theta)
            yaw = theta + math.pi / 2
            qw = math.cos(yaw / 2)
            qz = math.sin(yaw / 2)

            pkt = packer.pack(seq, t_now - t0, x, y, target_z,
                              qw, 0.0, 0.0, qz)
            try:
                sock.sendto(pkt, addr)
            except OSError:
                pass
            try:
                sock.sendto(pkt, mirror_addr)
            except OSError:
                pass
            seq += 1
            stop_event.wait(timeout=interval)

        sock.close()
        rst_sock.close()

    t = threading.Thread(target=_orbit_loop, daemon=True, name="orbit")
    t.start()
    return t


def start_patrol_thread(
    stop_event: threading.Event,
    patrol_length: float = 500.0,
    launch_offset: float = 50.0,
    speed_ms: float = 5.56,
    target_z: float = 100.0,
    sine_amp_xy: float = 0.0,
    sine_period_xy: float = 200.0,
    sine_amp_z: float = 0.0,
    sine_period_z: float = 200.0,
    lateral_offset: float = 0.0,
    rotation_deg: float = 0.0,
    udp_port: int = TARGET_UDP_PORT,
    reset_port: int = TARGET_RESET_PORT,
) -> threading.Thread:
    """Spawn a daemon thread that drives a triangle-wave patrol via UDP.

    The target starts at x = -launch_offset (behind the player), flies forward
    the full patrol_length to x = patrol_length - launch_offset, then returns.

    ``lateral_offset`` shifts the whole patrol track sideways in metres (applied
    in the patrol-local Y before rotation) so the target does not fly directly
    overhead. ``rotation_deg`` rotates the entire patrol path clockwise when
    viewed from above (i.e. negative yaw in ENU). Defaults of 0 keep the
    legacy straight-ahead behaviour.

    Sends a 72-byte VisualPosePacket at ~60 Hz to ``udp_port``.
    Listens on ``reset_port`` for reset signals.
    Returns the started thread.
    """

    def _patrol_loop():
        interval = 1.0 / 60
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = ("127.0.0.1", udp_port)
        mirror_addr = ("127.0.0.1", TARGET_MIRROR_PORT)
        packer = struct.Struct("<Qd3d4d")
        seq = 0
        t0 = time.monotonic()
        t_prev = t0
        x_min = -launch_offset
        x_max = patrol_length - launch_offset
        x = x_min
        direction = 1.0

        # Clockwise (from above) in ENU = negative yaw about +Z.
        theta = -math.radians(rotation_deg)
        c_t, s_t = math.cos(theta), math.sin(theta)
        # Yaw quaternions about +Z for the two travel directions.
        half = theta * 0.5
        qw_fwd, qz_fwd = math.cos(half), math.sin(half)
        half_rev = (theta + math.pi) * 0.5
        qw_rev, qz_rev = math.cos(half_rev), math.sin(half_rev)
        qw, qz = qw_fwd, qz_fwd

        rst_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rst_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rst_sock.bind(("0.0.0.0", reset_port))
        rst_sock.setblocking(False)

        omega_xy = (2.0 * math.pi / sine_period_xy) if sine_period_xy > 0 else 0.0
        omega_z = (2.0 * math.pi / sine_period_z) if sine_period_z > 0 else 0.0

        log.info("Patrol thread: length=%.0fm offset=%.0fm [%.0f..%.0f] speed=%.1f m/s alt=%.0fm lat=%.1fm rot=%.1f° (port %d)",
                 patrol_length, launch_offset, x_min, x_max, speed_ms, target_z,
                 lateral_offset, rotation_deg, udp_port)

        while not stop_event.is_set():
            try:
                while True:
                    rst_sock.recv(64)
                    x = x_min
                    direction = 1.0
                    qw, qz = qw_fwd, qz_fwd
                    t_prev = time.monotonic()
                    log.info("Patrol thread: reset to x=%.0f", x_min)
            except BlockingIOError:
                pass

            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now

            x += direction * speed_ms * dt
            if x >= x_max:
                x = x_max
                direction = -1.0
                qw, qz = qw_rev, qz_rev
            elif x <= x_min:
                x = x_min
                direction = 1.0
                qw, qz = qw_fwd, qz_fwd

            y_local = lateral_offset
            if sine_amp_xy:
                y_local += sine_amp_xy * math.sin(omega_xy * x)
            z_offset = sine_amp_z * math.sin(omega_z * x) if sine_amp_z else 0.0

            # Rotate (x, y_local) clockwise from above by rotation_deg.
            x_world = x * c_t - y_local * s_t
            y_world = x * s_t + y_local * c_t

            pkt = packer.pack(seq, t_now - t0, x_world, y_world, target_z + z_offset,
                              qw, 0.0, 0.0, qz)
            try:
                sock.sendto(pkt, addr)
            except OSError:
                pass
            try:
                sock.sendto(pkt, mirror_addr)
            except OSError:
                pass
            seq += 1
            stop_event.wait(timeout=interval)

        sock.close()
        rst_sock.close()

    t = threading.Thread(target=_patrol_loop, daemon=True, name="patrol")
    t.start()
    return t


# ── Parametric target trajectory (unified moving_target world) ────────────────
#
# A single GENERAL LOOP drives the target: a straight E-W section (oval_ew_len) +
# a straight N-S section (oval_ns_len) + equal rounded corners (corner_radius),
# built in a LOCAL frame where the target starts at (0,0) heading +X, then rotated
# by `rotation_deg` from the east axis and translated by (offset_ew, offset_ns) in
# world ENU relative to the player (origin). oval / circle / line are just PRESETS
# of this one shape — circle = zero straights, line = zero N-S + ~zero corners
# (degenerates to an out-and-back along X). `trajectory_sample` is the single
# source of truth (launcher thread + world-spawn pose + the mirrored leaf-sim-ui
# live diagram).

TRAJ_TYPES = ("oval", "circle", "line")
# Preset → (oval_ew_len, oval_ns_len, corner_radius); presets just seed the three
# loop params, each independently overridable.
TRAJ_PRESETS = {
    "oval":   (100.0, 60.0, 30.0),
    "circle": (0.0,   0.0,  50.0),
    "line":   (200.0, 0.0,  0.0),
}


def resolve_loop_geom(traj_type=None, oval_ew_len=None, oval_ns_len=None,
                      corner_radius=None):
    """Resolve the three loop params, filling None from the preset for ``traj_type``."""
    pe, pn, pr = TRAJ_PRESETS.get(traj_type or "oval", TRAJ_PRESETS["oval"])
    return (pe if oval_ew_len is None else float(oval_ew_len),
            pn if oval_ns_len is None else float(oval_ns_len),
            pr if corner_radius is None else float(corner_radius))


def _traj_local_path(s, oval_ew_len, oval_ns_len, corner_radius):
    """(x, y, tangent_yaw) in the LOCAL frame at arc-length ``s`` (m) for the general
    loop. ``path(0) == (0, 0)``, target heading +X. Circle = zero straights;
    line = zero N-S + ~zero corners (out-and-back along X)."""
    a = max(float(oval_ew_len), 0.0)
    b = max(float(oval_ns_len), 0.0)
    r = max(float(corner_radius), 1e-3)
    qc = (math.pi / 2.0) * r                  # quarter-corner arc length
    perim = 2.0 * a + 2.0 * b + 4.0 * qc
    u = s % perim
    if u < a:
        return (u, 0.0, 0.0)
    u -= a
    if u < qc:                                # bottom-right corner, centre (a, r)
        ang = -math.pi / 2.0 + u / r
        return (a + r * math.cos(ang), r + r * math.sin(ang), ang + math.pi / 2.0)
    u -= qc
    if u < b:
        return (a + r, r + u, math.pi / 2.0)
    u -= b
    if u < qc:                                # top-right corner, centre (a, r+b)
        ang = u / r
        return (a + r * math.cos(ang), (r + b) + r * math.sin(ang), ang + math.pi / 2.0)
    u -= qc
    if u < a:
        return (a - u, 2.0 * r + b, math.pi)
    u -= a
    if u < qc:                                # top-left corner, centre (0, r+b)
        ang = math.pi / 2.0 + u / r
        return (r * math.cos(ang), (r + b) + r * math.sin(ang), ang + math.pi / 2.0)
    u -= qc
    if u < b:
        return (-r, (r + b) - u, -math.pi / 2.0)
    u -= b                                     # bottom-left corner, centre (0, r)
    ang = math.pi + u / r
    return (r * math.cos(ang), r + r * math.sin(ang), ang + math.pi / 2.0)


def trajectory_sample(s, rotation_deg=0.0, offset_ew=0.0, offset_ns=0.0,
                      oval_ew_len=100.0, oval_ns_len=60.0, corner_radius=30.0,
                      start_pos=0.0, reverse=False):
    """World-frame (x_east, y_north, yaw) of the target at arc-length ``s`` (m).

    The loop is rotated about its **geometric centre** (not the target start) and
    that centre is placed at (offset_ew, offset_ns) relative to the player at the
    origin. ``start_pos`` (0..1 of the perimeter) sets where the target begins;
    ``reverse`` flips the travel direction.
    """
    a = max(float(oval_ew_len), 0.0)
    b = max(float(oval_ns_len), 0.0)
    r = max(float(corner_radius), 1e-3)
    perim = 2.0 * a + 2.0 * b + 2.0 * math.pi * r
    cx, cy = a / 2.0, r + b / 2.0                 # geometric centre of the loop
    s0 = (float(start_pos) % 1.0) * perim
    sp = (s0 - s) if reverse else (s0 + s)
    lx, ly, lyaw = _traj_local_path(sp, a, b, r)
    px, py = lx - cx, ly - cy                     # rotate about the centre
    th = math.radians(rotation_deg)
    c, sn = math.cos(th), math.sin(th)
    yaw = lyaw + (math.pi if reverse else 0.0) + th
    return (offset_ew + px * c - py * sn,
            offset_ns + px * sn + py * c,
            yaw)


def trajectory_start_pose(**params):
    """(x_east, y_north, yaw) of the target at ``s = 0`` — used for the world-SDF
    spawn pose so the model appears exactly where the thread first drives it."""
    return trajectory_sample(0.0, **params)


def start_trajectory_thread(
    stop_event: threading.Event,
    traj_type: str = "oval",
    speed_ms: float = 5.0,
    target_z: float = 50.0,
    rotation_deg: float = 0.0,
    offset_ew: float = 0.0,
    offset_ns: float = 0.0,
    oval_ew_len: float | None = None,
    oval_ns_len: float | None = None,
    corner_radius: float | None = None,
    start_pos: float = 0.0,
    reverse: bool = False,
    udp_port: int = TARGET_UDP_PORT,
    reset_port: int = TARGET_RESET_PORT,
) -> threading.Thread:
    """Drive the target along a parametric loop via UDP (unified world). The three
    loop params fall back to the ``traj_type`` preset when None.

    Sends a 72-byte VisualPosePacket at ~60 Hz to ``udp_port`` (+ mirror) and
    listens on ``reset_port`` (a reset snaps the target back to ``s = 0``).
    """
    _ew, _ns, _cr = resolve_loop_geom(traj_type, oval_ew_len, oval_ns_len, corner_radius)
    geom = dict(rotation_deg=rotation_deg, offset_ew=offset_ew, offset_ns=offset_ns,
                oval_ew_len=_ew, oval_ns_len=_ns, corner_radius=_cr,
                start_pos=start_pos, reverse=reverse)

    def _traj_loop():
        interval = 1.0 / 60
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = ("127.0.0.1", udp_port)
        mirror_addr = ("127.0.0.1", TARGET_MIRROR_PORT)
        packer = struct.Struct("<Qd3d4d")
        seq = 0
        t0 = time.monotonic()
        t_prev = t0
        s = 0.0

        rst_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rst_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rst_sock.bind(("0.0.0.0", reset_port))
        rst_sock.setblocking(False)

        log.info("Trajectory thread: %s speed=%.1f m/s alt=%.0fm rot=%.1f° "
                 "offset=(%.0fE,%.0fN) (port %d)", traj_type, speed_ms, target_z,
                 rotation_deg, offset_ew, offset_ns, udp_port)

        while not stop_event.is_set():
            try:
                while True:
                    rst_sock.recv(64)
                    s = 0.0
                    t_prev = time.monotonic()
                    log.info("Trajectory thread: reset to start")
            except BlockingIOError:
                pass

            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now
            s += speed_ms * dt

            x, y, yaw = trajectory_sample(s, **geom)
            qw = math.cos(yaw / 2.0)
            qz = math.sin(yaw / 2.0)
            pkt = packer.pack(seq, t_now - t0, x, y, target_z, qw, 0.0, 0.0, qz)
            for dst in (addr, mirror_addr):
                try:
                    sock.sendto(pkt, dst)
                except OSError:
                    pass
            seq += 1
            stop_event.wait(timeout=interval)

        sock.close()
        rst_sock.close()

    t = threading.Thread(target=_traj_loop, daemon=True, name="trajectory")
    t.start()
    return t


BALLOON_UDP_PORT = 9014


def start_balloon_thread(
    stop_event: threading.Event,
    mean_x: float = 30.0,
    mean_y: float = 0.0,
    mean_z: float = 10.0,
    wind_intensity: float = 2.0,
    wind_randomness: float = 1.0,
    drift_speed: float = 20.0,
    harmonic: bool = False,
    amp_x: float = 10.0,
    amp_y: float = 10.0,
    rate_x: float = 0.05,
    rate_y: float = 0.05,
    phase_y_deg: float = 90.0,
    udp_port: int = BALLOON_UDP_PORT,
) -> threading.Thread:
    """Spawn a daemon thread that drives balloon motion at ~60 Hz.

    Sends a 72-byte VisualPosePacket to ``udp_port`` (Gazebo) and the GT mirror
    port. Two motion models, both centred on ``(mean_x, mean_y, mean_z)``:

    * ``harmonic=False`` (default): smooth multi-sine Lissajous *drift*
      (``wind_intensity`` horizontal amp, ``wind_randomness`` vertical bob,
      ``drift_speed`` rate).
    * ``harmonic=True``: simple per-axis simple-harmonic motion about the centre —
      ``x = cx + amp_x·sin(2π·rate_x·t)``,
      ``y = cy + amp_y·sin(2π·rate_y·t + phase_y)``, ``z = cz`` (constant).
      Rates are in Hz. Equal amp/rate with ``phase_y_deg=90`` traces a circle;
      ``0`` a diagonal line; unequal rates a Lissajous figure.

    Returns the started thread.
    """

    def _wind_loop():
        interval = 1.0 / 60  # 60 Hz
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = ("127.0.0.1", udp_port)
        mirror_addr = ("127.0.0.1", TARGET_MIRROR_PORT)
        packer = struct.Struct("<Qd3d4d")
        seq = 0
        t0 = time.monotonic()

        if harmonic:
            wx = 2.0 * math.pi * rate_x
            wy = 2.0 * math.pi * rate_y
            phy = math.radians(phase_y_deg)
            log.info("Balloon harmonic (UDP:%d, GT mirror:%d): centre=(%.1f,%.1f,%.1f) "
                     "amp=(%.1f,%.1f)m rate=(%.3f,%.3f)Hz phase_y=%.0f deg",
                     udp_port, TARGET_MIRROR_PORT, mean_x, mean_y, mean_z,
                     amp_x, amp_y, rate_x, rate_y, phase_y_deg)
        else:
            amp = wind_intensity
            amp_z = wind_randomness
            spd = drift_speed
            px = [random.uniform(0, 2 * math.pi) for _ in range(3)]
            py = [random.uniform(0, 2 * math.pi) for _ in range(3)]
            pz = [random.uniform(0, 2 * math.pi) for _ in range(3)]
            fx = [0.13 * spd, 0.31 * spd, 0.53 * spd]
            fy = [0.17 * spd, 0.41 * spd, 0.67 * spd]
            fz = [0.11 * spd, 0.29 * spd, 0.47 * spd]
            log.info("Balloon drift (UDP:%d, GT mirror:%d): amp=%.1f m  bob=%.1f m  speed=%.1fx",
                     udp_port, TARGET_MIRROR_PORT, amp, amp_z, spd)

        while not stop_event.is_set():
            t = time.monotonic() - t0

            if harmonic:
                bx = mean_x + amp_x * math.sin(wx * t)
                by = mean_y + amp_y * math.sin(wy * t + phy)
                bz = mean_z
            else:
                bx = mean_x + amp * (0.5 * math.sin(fx[0] * t + px[0])
                                   + 0.3 * math.sin(fx[1] * t + px[1])
                                   + 0.2 * math.sin(fx[2] * t + px[2]))
                by = mean_y + amp * (0.5 * math.sin(fy[0] * t + py[0])
                                   + 0.3 * math.sin(fy[1] * t + py[1])
                                   + 0.2 * math.sin(fy[2] * t + py[2]))
                bz = mean_z + amp_z * (0.4 * math.sin(fz[0] * t + pz[0])
                                     + 0.35 * math.sin(fz[1] * t + pz[1])
                                     + 0.25 * math.sin(fz[2] * t + pz[2]))

            # identity quaternion (w=1, x=0, y=0, z=0)
            pkt = packer.pack(seq, t, bx, by, bz, 1.0, 0.0, 0.0, 0.0)
            try:
                sock.sendto(pkt, addr)             # drive Gazebo's balloon ExternalPosePlugin
                sock.sendto(pkt, mirror_addr)      # mirror to sitl_redis_bridge for target:gps GT
            except OSError:
                pass
            seq += 1

            stop_event.wait(timeout=interval)

        sock.close()

    t = threading.Thread(target=_wind_loop, daemon=True, name="balloon_wind")
    t.start()
    return t


def start_static_target_thread(
    stop_event: threading.Event,
    x: float,
    y: float,
    z: float,
    udp_port: int = TARGET_MIRROR_PORT,
) -> threading.Thread:
    """Publish a FIXED target's ground-truth pose to the redis GT bridge.

    For worlds whose target is a static ``<include>``d model (e.g. collision_test)
    Gazebo already places the object from the world SDF — no ExternalPosePlugin
    drives it, so nothing would otherwise feed sitl_redis_bridge's ``target:gps``.
    This emits the known static ENU position ``(x, y, z)`` as a 72-byte
    VisualPosePacket at ~60 Hz to ``udp_port`` (the mirror/GT port only — it must
    NOT touch Gazebo's 9016/9014, which the moving targets own). Returns the thread.
    """

    def _static_loop():
        interval = 1.0 / 60
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = ("127.0.0.1", udp_port)
        packer = struct.Struct("<Qd3d4d")
        seq = 0
        t0 = time.monotonic()

        log.info("Static target GT (UDP:%d): pos=(%.1f, %.1f, %.1f) ENU", udp_port, x, y, z)

        while not stop_event.is_set():
            t = time.monotonic() - t0
            # identity quaternion (w=1, x=0, y=0, z=0); fixed position
            pkt = packer.pack(seq, t, x, y, z, 1.0, 0.0, 0.0, 0.0)
            try:
                sock.sendto(pkt, addr)
            except OSError:
                pass
            seq += 1
            stop_event.wait(timeout=interval)

        sock.close()

    th = threading.Thread(target=_static_loop, daemon=True, name="static_target_gt")
    th.start()
    return th
