#!/usr/bin/env python3
"""Rocket drone launcher with collision-test world as default.

This is a thin wrapper around start_rocket_drone_fpv.py so we keep one
implementation of the FPV/chase streaming stack while giving collision tests a
dedicated entrypoint.
"""

import sys

import start_rocket_drone_fpv as rocket_fpv


def _has_world_arg(argv):
    for i, arg in enumerate(argv):
        if arg == "--world":
            return True
        if arg.startswith("--world="):
            return True
        # argparse short option isn't defined, but keep this for robustness.
        if arg == "-w" and i + 1 < len(argv):
            return True
    return False


def main():
    # Keep behavior identical to start_rocket_drone_fpv, but default to the
    # collision test world unless user explicitly passes --world.
    if not _has_world_arg(sys.argv[1:]):
        sys.argv.extend(["--world", "rocket_drone_collision_test.world"])
    rocket_fpv.main()


if __name__ == "__main__":
    main()

