#!/usr/bin/env python3
"""Executable script wrapper for CogProj Demonstration Live Status Monitor."""

import os
import sys

# Ensure cogproj_bringup is in python path
current_dir = os.path.dirname(os.path.abspath(__file__))
ws_dir = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(ws_dir, "src", "cogproj_bringup"))

from cogproj_bringup.demo_status import main

if __name__ == "__main__":
    main()

