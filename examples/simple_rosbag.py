#!/usr/bin/env python3
"""
Simple Rosbag SLAM Example

Demonstrates the 2-line SDK usage for running SLAM with a rosbag file.
Automatically enables simulation time and publishes /clock.

Usage:
    python simple_rosbag.py
"""

from neuronav import Rosbag, RTABMapSLAM, run_slam

# Line 1: Choose your data source
sensor = Rosbag("/bags/your_recording.mcap")

# Line 2: Run SLAM with visualization
run_slam(sensor, RTABMapSLAM(), visualize=True)

# Open ws://localhost:8765 in Foxglove Studio to visualize
