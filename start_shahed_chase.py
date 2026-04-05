#!/usr/bin/env python3
"""DEPRECATED — use start.py instead.

Equivalent command:
    python3 start.py --world shahed_chase_park.world --gazebo
"""
import sys
print(
    "\n"
    "  ⚠  This script is deprecated.  Use the unified launcher instead:\n"
    "\n"
    "      python3 start.py --world shahed_chase_park.world --gazebo\n"
    "\n",
    file=sys.stderr,
)
sys.exit(1)
