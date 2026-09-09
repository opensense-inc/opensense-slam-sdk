# Copyright 2025 NeuroNav Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0

"""
NeuroNav SLAM SDK - Production-ready SLAM for robotics with 2-line API

Built on RTAB-Map by Mathieu Labbé & IntRoLab
Supports Intel RealSense and Luxonis OAK-D cameras

2-Line Usage:
    from neuronav import RealSenseSensor, RTABMapSLAM, run_slam

    sensor = RealSenseSensor()
    run_slam(sensor, RTABMapSLAM())

With Logging:
    from neuronav import configure_logging

    configure_logging(level="DEBUG", log_file="/var/log/neuronav.log")

Visualization:
    from neuronav import visualize

    visualize()  # Starts Foxglove Bridge on ws://localhost:8765
    # Then open https://foxglove.dev/studio and connect

Credits:
    - RTAB-Map SLAM: https://github.com/introlab/rtabmap
    - Intel RealSense: https://www.intelrealsense.com/
    - Luxonis OAK-D: https://www.luxonis.com/
    - See ACKNOWLEDGMENTS.md for complete credits
"""

# Version information
from .__version__ import __version__, __version_info__, __author__, __license__

# SDK Interface
from .sdk import run_slam
from .sensors import RealSenseSensor, OAKDSensor, SensorBase, SensorConfig
from .sensors.rosbag import Rosbag, RosbagConfig
from .algorithms import RTABMapSLAM
from .slam_base import SlamBase, SlamStatus, SlamConfig
from .visualization import visualize, FoxgloveVisualizer

# Logging and error handling
from .logger import get_logger, configure_logging
from .exceptions import (
    NeuroNavError,
    SensorError,
    SensorNotFoundError,
    SensorInitializationError,
    SlamError,
    SlamInitializationError,
    ConfigurationError,
    ProcessError,
)

__all__ = [
    # Version
    "__version__",
    "__version_info__",
    "__author__",
    "__license__",
    # Main SDK interface
    "run_slam",
    # Sensors
    "RealSenseSensor",
    "OAKDSensor",
    "Rosbag",
    "SensorBase",
    "SensorConfig",
    "RosbagConfig",
    # SLAM Algorithms
    "RTABMapSLAM",
    "SlamBase",
    "SlamStatus",
    "SlamConfig",
    # Visualization
    "visualize",
    "FoxgloveVisualizer",
    # Logging
    "get_logger",
    "configure_logging",
    # Exceptions
    "NeuroNavError",
    "SensorError",
    "SensorNotFoundError",
    "SensorInitializationError",
    "SlamError",
    "SlamInitializationError",
    "ConfigurationError",
    "ProcessError",
]
