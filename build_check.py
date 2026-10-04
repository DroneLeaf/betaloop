"""build_check.py — which compiled pieces of the sim are older than their source.

After a `git pull` the C/C++ sources move on but nothing rebuilds them, and
the sim then runs the OLD binaries without saying so: a stale
ShmCameraExportPlugin silently ignores the GPU-warp `<warp>` specs (the
trackers get un-warped frames), a missing OgreWorkerThreads shim leaves gz on
one Ogre worker per logical core — the 2026-10-04 "60 fps on a stronger
machine" was exactly this. An artifact is stale when it is missing or older
than the newest of its sources (mtime; a checkout that touches a source
without changing it just costs one unneeded rebuild).

Stdlib only: imported by betaloop/common.py (launcher log warning) and by
leaf-sim-ui (Initialize dialog with a "Rebuild now" button).
"""
from __future__ import annotations

import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

_PLUGINS = "aeroloop_gazebo/plugins"
_PLUGINS_BUILD = (f"cmake -S {_PLUGINS} -B {_PLUGINS}/build && "
                  f"cmake --build {_PLUGINS}/build -j4")
_BRIDGE_BUILD = ("cmake -S bf_sim_bridge -B bf_sim_bridge/build && "
                 "cmake --build bf_sim_bridge/build -j4")
_BF_BUILD = "make -C betaflight TARGET=SITL -j4"

# (artifact, source globs, rebuild command) — paths relative to REPO_ROOT.
CHECKS = (
    (f"{_PLUGINS}/build/libShmCameraExportPlugin.so",
     (f"{_PLUGINS}/ShmCameraExportPlugin.cc",), _PLUGINS_BUILD),
    (f"{_PLUGINS}/build/libOgreWorkerThreads.so",
     (f"{_PLUGINS}/OgreWorkerThreads.cc",), _PLUGINS_BUILD),
    (f"{_PLUGINS}/build/gz_image_bridge",
     (f"{_PLUGINS}/gz_image_bridge.cc", f"{_PLUGINS}/osd_font.h"), _PLUGINS_BUILD),
    (f"{_PLUGINS}/build/libExternalPosePlugin.so",
     (f"{_PLUGINS}/ExternalPosePlugin.cc",), _PLUGINS_BUILD),
    (f"{_PLUGINS}/build/libRotorVisualPlugin.so",
     (f"{_PLUGINS}/RotorVisualPlugin.cc",), _PLUGINS_BUILD),
    ("bf_sim_bridge/build/bf_sim_bridge",
     ("bf_sim_bridge/*.cpp", "bf_sim_bridge/*.h"), _BRIDGE_BUILD),
    ("betaflight/obj/main/betaflight_SITL.elf",
     ("betaflight/src/main/**/*.c", "betaflight/src/main/**/*.h"), _BF_BUILD),
)


def _newest(root: Path, patterns) -> tuple[float, Path | None]:
    best, best_path = 0.0, None
    for pat in patterns:
        for p in root.glob(pat):
            try:
                m = p.stat().st_mtime
            except OSError:
                continue
            if m > best:
                best, best_path = m, p
    return best, best_path


def stale_builds(root: Path | str = REPO_ROOT) -> list[dict]:
    """Every stale artifact: {artifact, reason, command} (paths relative to
    `root`). Empty list = everything is built and current. Artifacts whose
    sources are absent (a checkout without that submodule) are skipped."""
    root = Path(root)
    out = []
    for artifact, sources, command in CHECKS:
        src_m, src_p = _newest(root, sources)
        if src_p is None:
            continue
        art = root / artifact
        if not art.exists():
            out.append({"artifact": artifact, "reason": "not built", "command": command})
            continue
        if art.stat().st_mtime + 1.0 < src_m:
            out.append({"artifact": artifact,
                        "reason": f"older than {src_p.relative_to(root)}",
                        "command": command})
    return out


def rebuild_commands(stale: list[dict]) -> list[str]:
    """The distinct commands that rebuild `stale`, in CHECKS order."""
    seen, cmds = set(), []
    for s in stale:
        if s["command"] not in seen:
            seen.add(s["command"])
            cmds.append(s["command"])
    return cmds


if __name__ == "__main__":
    stale = stale_builds()
    for s in stale:
        print(f"STALE {s['artifact']}: {s['reason']}")
    for c in rebuild_commands(stale):
        print(f"  (cd {REPO_ROOT} && {c})")
    raise SystemExit(1 if stale else 0)
