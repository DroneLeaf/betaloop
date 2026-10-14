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

# The Controller Dashboard frontend the fleet drones serve (drone_stack
# dashboard-ui): its build lives outside this repo. nvm is sourced explicitly —
# SimControl may be started from the desktop launcher without it on PATH.
DASHBOARD_DIR = Path(os.environ.get("LEAF_DASHBOARD_WWW", "~/Controller-Dashboard/www")
                     ).expanduser().parent
_DASHBOARD_BUILD = ('[ -s "$HOME/.nvm/nvm.sh" ] && . "$HOME/.nvm/nvm.sh"; '
                    f'cd "{DASHBOARD_DIR}" && npm run build')
# The fleet LAUNCHERS' dashboard: the workspace's second Angular app
# (projects/leaf-launcher-controller-dashboard → www-launcher). It also
# compiles the shared src/ (racer pages + services), so both trees are sources.
_LAUNCHER_DASHBOARD_BUILD = ('[ -s "$HOME/.nvm/nvm.sh" ] && . "$HOME/.nvm/nvm.sh"; '
                             f'cd "{DASHBOARD_DIR}" && npx ng build leaf-launcher-controller-dashboard')
_LAUNCHER_APP = "projects/leaf-launcher-controller-dashboard/src"

# (artifact, source globs, rebuild command[, base dir, tag]) — paths relative to
# REPO_ROOT unless a base dir is given. Tagged checks only run when asked for
# (stale_builds(include=…)): "dashboard" matters only to a fleet with
# per-drone dashboards, "launcher-dashboard" to one with per-launcher ones.
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
    ("www/index.html", ("src/**/*.ts", "src/**/*.html", "src/**/*.scss", "src/**/*.json"),
     _DASHBOARD_BUILD, DASHBOARD_DIR, "dashboard"),
    ("www-launcher/index.html",
     tuple(f"{root}/**/*.{ext}" for root in (_LAUNCHER_APP, "src")
           for ext in ("ts", "html", "scss", "json")),
     _LAUNCHER_DASHBOARD_BUILD, DASHBOARD_DIR, "launcher-dashboard"),
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


def stale_builds(root: Path | str = REPO_ROOT, include=()) -> list[dict]:
    """Every stale artifact: {artifact, reason, command} (paths relative to
    `root`, or absolute for an external project). Empty list = everything is
    built and current. Artifacts whose sources are absent (a checkout without
    that submodule/project) are skipped, and so are tagged checks not in
    `include` (e.g. "dashboard", "launcher-dashboard")."""
    root = Path(root)
    out = []
    for check in CHECKS:
        artifact, sources, command = check[:3]
        base = Path(check[3]) if len(check) > 3 and check[3] else root
        tag = check[4] if len(check) > 4 else None
        if tag and tag not in include:
            continue
        src_m, src_p = _newest(base, sources)
        if src_p is None:
            continue
        art = base / artifact
        shown = artifact if base == root else str(art)
        if not art.exists():
            out.append({"artifact": shown, "reason": "not built", "command": command})
            continue
        if art.stat().st_mtime + 1.0 < src_m:
            out.append({"artifact": shown,
                        "reason": f"older than {src_p.relative_to(base)}",
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
