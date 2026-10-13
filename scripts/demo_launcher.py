#!/usr/bin/env python3
"""CogProj Demonstration Launcher and Workflow Assistant.

Provides an easy, guided CLI launcher to run Mode A (Mock) or Mode B (Real)
with automatic preflight checking, clear diagnostics, and copy-pasteable
workstation viewer commands.
Does NOT publish robot actuation commands or move the robot.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
ws_dir = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(ws_dir, "src", "cogproj_bringup"))

from cogproj_bringup.preflight_check import run_preflight


def main() -> None:
    parser = argparse.ArgumentParser(description="CogProj Coordinated Demonstration Launcher")
    parser.add_argument(
        "--mode",
        choices=["mock", "real"],
        default="mock",
        help="Demonstration mode: 'mock' (synthetic) or 'real' (ONNX)",
    )
    parser.add_argument(
        "--camera-topic",
        default="/camera/image_raw",
        help="Camera input topic (default: /camera/image_raw)",
    )
    parser.add_argument(
        "--start-rviz",
        action="store_true",
        help="Launch RViz2 on the local machine (requires display)",
    )
    parser.add_argument(
        "--start-slam",
        action="store_true",
        help="Start SLAM Toolbox mapping node (default: false)",
    )
    parser.add_argument(
        "--demo-map",
        action="store_true",
        help="Publish isolated simulated demo map for RViz layout (mock mode only)",
    )
    parser.add_argument(
        "--skip-preflight",
        action="store_true",
        help="Skip preflight diagnostic audit",
    )
    args = parser.parse_args()

    # Step 1: Preflight Audit
    if not args.skip_preflight:
        print("[Launcher] Running pre-flight diagnostic check...")
        run_preflight(mode=args.mode)
        print("[Launcher] Pre-flight audit complete.")
        time.sleep(1.0)

    # Step 2: Build launch command
    launch_args = [
        "ros2",
        "launch",
        "cogproj_bringup",
        "demo_showcase.launch.py",
        f"detector_mode:={args.mode}",
        f"camera_input_topic:={args.camera_topic}",
        f"start_rviz:={'true' if args.start_rviz else 'false'}",
        f"start_slam:={'true' if args.start_slam else 'false'}",
        f"publish_simulated_demo_map:={'true' if args.demo_map else 'false'}",
    ]

    print("\n" + "=" * 78)
    print(f"LAUNCHING COGPROJ DEMONSTRATION IN MODE: {args.mode.upper()}")
    print("=" * 78)
    print(f"Command: {' '.join(launch_args)}\n")
    print("WORKSTATION VIEWING INSTRUCTIONS (Run on Ubuntu desktop):")
    print("  1. Image Viewer (Annotated Video):")
    print("     ros2 run rqt_image_view rqt_image_view /cogproj/image_annotated")
    print("  2. RViz Visualizer (Map, LiDAR, Markers, Frames):")
    print("     rviz2 -d /home/veerobot/CogProj/src/cogproj_bringup/rviz/cogproj_slam_ore.rviz")
    print("  3. Headless Terminal Dashboard (In another SSH window):")
    print("     python3 scripts/demo_status.py --mode " + args.mode)
    print("=" * 78 + "\n")
    print("Starting pipeline in foreground (Press Ctrl+C to stop)...\n")

    # Step 3: Run foreground process
    proc = None
    try:
        proc = subprocess.Popen(launch_args)
        proc.wait()
    except KeyboardInterrupt:
        print("\n[Launcher] Stopping demonstration pipeline gracefully...")
        if proc:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        print("[Launcher] All CogProj demonstration nodes stopped.")


if __name__ == "__main__":
    main()

