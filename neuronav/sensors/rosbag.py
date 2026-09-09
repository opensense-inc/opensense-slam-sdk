# Copyright 2025 NeuroNav Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0

"""MCAP Rosbag playback sensor adapter."""

import re
import subprocess
from pathlib import Path
from typing import Dict, Optional, cast
from dataclasses import dataclass, field

from .base import SensorBase, SensorConfig
from ..process_manager import ManagedProcess, ProcessManager
from ..logger import get_logger
from ..exceptions import SensorInitializationError, ConfigurationError


@dataclass
class RosbagConfig(SensorConfig):
    """
    Configuration for MCAP rosbag playback.

    Attributes:
        bag_path: Path to MCAP rosbag file
        playback_rate: Playback speed multiplier (1.0 = normal speed)
        loop: Loop playback when bag ends
        topic_remapping: Manual topic remapping dictionary
    """

    bag_path: str = ""
    playback_rate: float = 1.0
    loop: bool = False
    topic_remapping: Dict[str, str] = field(default_factory=dict)

    def validate(self):
        """
        Validate rosbag configuration.

        Raises:
            ConfigurationError: If configuration is invalid
        """
        if not self.bag_path:
            raise ConfigurationError("bag_path", self.bag_path, "Bag path is required")

        bag_file = Path(self.bag_path)
        if not bag_file.exists():
            raise ConfigurationError(
                "bag_path", self.bag_path, f"Bag file not found: {self.bag_path}"
            )

        if bag_file.suffix not in [".mcap", ".db3"]:
            raise ConfigurationError(
                "bag_path", self.bag_path, f"Unsupported format: {bag_file.suffix}"
            )

        if self.playback_rate <= 0:
            raise ConfigurationError("playback_rate", self.playback_rate, "Must be > 0")


class Rosbag(SensorBase):
    """
    MCAP Rosbag playback as a sensor.

    Provides rosbag playback with automatic topic discovery, configurable
    playback rate, looping, and simulation time support.
    """

    def __init__(self, bag_path: str):
        """
        Initialize rosbag sensor.

        Args:
            bag_path: Path to MCAP rosbag file
        """
        super().__init__()
        self._bag_path = bag_path
        self._process_manager = ProcessManager()
        self._logger = get_logger("sensor.rosbag")
        self._discovered_topics: Dict[str, str] = {}
        self._config: Optional[RosbagConfig] = None

    def configure(self, config: Optional[RosbagConfig] = None) -> None:
        """
        Configure rosbag playback.

        Args:
            config: Rosbag configuration (creates default if not provided)

        Raises:
            ConfigurationError: If configuration is invalid
        """
        if config is None or not isinstance(config, RosbagConfig):
            config = RosbagConfig(bag_path=self._bag_path)
        elif not config.bag_path:
            config.bag_path = self._bag_path

        config.validate()
        self._config = config
        self._discover_topics()
        self._is_configured = True

        self._logger.info(
            f"Configured: {Path(config.bag_path).name} ({config.playback_rate}x, loop={config.loop})"
        )

    def _discover_topics(self) -> None:
        """
        Discover topics from bag metadata using ros2 bag info.

        Automatically maps topics to standard sensor interface:
        - RGB image: sensor_msgs/Image with 'rgb' or 'color' in name
        - Depth image: sensor_msgs/Image with 'depth' in name
        - Camera info: sensor_msgs/CameraInfo
        - IMU: sensor_msgs/Imu

        Raises:
            SensorInitializationError: If bag info cannot be read or required topics missing
        """
        config = cast(RosbagConfig, self._config)

        try:
            result = subprocess.run(
                ["ros2", "bag", "info", config.bag_path],
                capture_output=True,
                text=True,
                timeout=10,
            )

            if result.returncode != 0:
                raise SensorInitializationError(
                    "Rosbag", f"Failed to read bag info: {result.stderr}"
                )

            # Parse topics from output
            topics = {}
            for line in result.stdout.split("\n"):
                if "Topic:" in line and "Type:" in line:
                    topic_match = re.search(r"Topic:\s+(\S+)", line)
                    type_match = re.search(r"Type:\s+(\S+)", line)
                    if topic_match and type_match:
                        topics[topic_match.group(1)] = type_match.group(1)

            if not topics:
                raise SensorInitializationError("Rosbag", "No topics found in bag file")

            # Auto-map topics
            self._discovered_topics = self._map_topics(topics)

            # Apply manual remapping
            if config.topic_remapping:
                for key, topic in self._discovered_topics.items():
                    if topic in config.topic_remapping:
                        self._discovered_topics[key] = config.topic_remapping[topic]

            # Validate required topics
            required = ["rgb", "depth", "camera_info"]
            missing = [k for k in required if k not in self._discovered_topics]
            if missing:
                raise SensorInitializationError("Rosbag", f"Missing topics: {missing}")

        except subprocess.TimeoutExpired:
            raise SensorInitializationError("Rosbag", "Timeout reading bag info")
        except SensorInitializationError:
            raise
        except Exception as e:
            raise SensorInitializationError("Rosbag", f"Failed to discover topics: {e}")

    def _map_topics(self, topics: Dict[str, str]) -> Dict[str, str]:
        """
        Map discovered topics to standard sensor interface.

        Args:
            topics: Dictionary of topic_name -> message_type from bag

        Returns:
            Dictionary with keys: 'rgb', 'depth', 'camera_info', 'imu' (optional)
        """
        mapped = {}

        for topic_name, msg_type in topics.items():
            topic_lower = topic_name.lower()
            msg_lower = msg_type.lower()

            if "image" in msg_lower:
                if "rgb" in topic_lower or "color" in topic_lower:
                    if "rgb" not in mapped:
                        mapped["rgb"] = topic_name
                elif "depth" in topic_lower:
                    if "depth" not in mapped:
                        mapped["depth"] = topic_name
            elif "camerainfo" in msg_lower and "camera_info" not in mapped:
                mapped["camera_info"] = topic_name
            elif "imu" in msg_lower and "imu" not in mapped:
                mapped["imu"] = topic_name

        return mapped

    def start(self) -> None:
        """
        Start rosbag playback.

        Raises:
            RuntimeError: If sensor not configured
            SensorInitializationError: If playback fails to start
        """
        if not self._is_configured:
            raise RuntimeError("Sensor must be configured before starting")

        if self._is_running:
            return

        config = cast(RosbagConfig, self._config)

        try:
            cmd = [
                "ros2",
                "bag",
                "play",
                config.bag_path,
                "--rate",
                str(config.playback_rate),
                "--clock",
            ]

            if config.loop:
                cmd.append("--loop")

            # Only play discovered topics
            valid_topics = [t for t in self._discovered_topics.values() if " " not in t]
            if valid_topics:
                cmd.extend(["--topics"] + valid_topics)
            
            # Apply topic remappings to playback
            if config.topic_remapping:
                for old_topic, new_topic in config.topic_remapping.items():
                    cmd.extend(["--remap", f"{old_topic}:={new_topic}"])

            playback_process = ManagedProcess(
                name="rosbag_playback",
                command=cmd,
                startup_timeout=2.0,
                shutdown_timeout=5.0,
            )
            self._process_manager.add_process(playback_process)
            playback_process.start()

            self._is_running = True
            self._logger.info(f"Started playback at {config.playback_rate}x speed")

        except Exception as e:
            self._is_running = False
            raise SensorInitializationError("Rosbag", f"Failed to start playback: {e}")

    def stop(self) -> None:
        """Stop rosbag playback and cleanup."""
        if not self._is_running:
            return

        try:
            self._process_manager.stop_all()
            self._is_running = False
        except Exception as e:
            self._logger.error(f"Error stopping rosbag: {e}")
            self._process_manager.stop_all(force=True)
            self._is_running = False

    def get_sensor_name(self) -> str:
        """
        Get sensor name.

        Returns:
            Human-readable sensor name
        """
        bag_name = (
            Path(self._config.bag_path).name
            if self._config
            else Path(self._bag_path).name
        )
        return f"Rosbag: {bag_name}"

    def get_ros_topics(self) -> Dict[str, str]:
        """
        Get ROS2 topics from the bag.

        Returns:
            Dictionary with discovered topics mapped to standard names
        """
        return self._discovered_topics.copy()

    def health_check(self) -> bool:
        """
        Check if playback is healthy.

        Returns:
            True if playback process is running
        """
        return self._is_running and self._process_manager.health_check_all()
