# Copyright 2025 NeuroNav Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for rosbag sensor adapter."""

import pytest
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch
from neuronav.sensors.rosbag import Rosbag, RosbagConfig
from neuronav.exceptions import ConfigurationError, SensorInitializationError


def test_rosbag_config_validation():
    """Test RosbagConfig validation for path, format, and file existence."""
    # Empty path
    with pytest.raises(ConfigurationError) as exc_info:
        RosbagConfig(bag_path="")
    assert "bag_path" in str(exc_info.value)
    
    # Nonexistent file
    with pytest.raises(ConfigurationError) as exc_info:
        RosbagConfig(bag_path="/nonexistent/file.mcap")
    assert "not found" in str(exc_info.value).lower()
    
    # Invalid format
    with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as f:
        temp_path = f.name
    try:
        with pytest.raises(ConfigurationError) as exc_info:
            RosbagConfig(bag_path=temp_path)
        assert "format" in str(exc_info.value).lower()
    finally:
        Path(temp_path).unlink()


@patch("neuronav.sensors.rosbag.subprocess.run")
def test_rosbag_topic_discovery(mock_run):
    """Test topic discovery and validation."""
    with tempfile.NamedTemporaryFile(suffix=".mcap", delete=False) as f:
        temp_path = f.name

    try:
        # Successful topic discovery
        mock_result = Mock()
        mock_result.returncode = 0
        mock_result.stdout = """
        Topic: /camera/color/image_raw | Type: sensor_msgs/msg/Image
        Topic: /camera/depth/image_raw | Type: sensor_msgs/msg/Image
        Topic: /camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo
        Topic: /camera/imu | Type: sensor_msgs/msg/Imu
        """
        mock_run.return_value = mock_result

        rosbag = Rosbag(temp_path)
        rosbag.configure()
        topics = rosbag.get_ros_topics()
        
        assert "rgb" in topics
        assert "depth" in topics
        assert "camera_info" in topics
        assert "imu" in topics
        
        # Missing required topics
        mock_result.stdout = "Topic: /camera/color/image_raw | Type: sensor_msgs/msg/Image"
        rosbag2 = Rosbag(temp_path)
        with pytest.raises(SensorInitializationError) as exc_info:
            rosbag2.configure()
        assert "Missing topics" in str(exc_info.value)
        
        # Command failure
        mock_result.returncode = 1
        mock_result.stderr = "Command failed"
        rosbag3 = Rosbag(temp_path)
        with pytest.raises(SensorInitializationError) as exc_info:
            rosbag3.configure()
        assert "Failed to read bag info" in str(exc_info.value)
    finally:
        Path(temp_path).unlink()


@patch("neuronav.sensors.rosbag.subprocess.run")
@patch("neuronav.sensors.rosbag.ManagedProcess")
def test_rosbag_lifecycle(mock_process_class, mock_run):
    """Test rosbag start/stop lifecycle."""
    with tempfile.NamedTemporaryFile(suffix=".mcap", delete=False) as f:
        temp_path = f.name

    try:
        # Mock ros2 bag info
        mock_result = Mock()
        mock_result.returncode = 0
        mock_result.stdout = """
        Topic: /camera/color/image_raw | Type: sensor_msgs/msg/Image
        Topic: /camera/depth/image_raw | Type: sensor_msgs/msg/Image
        Topic: /camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo
        """
        mock_run.return_value = mock_result

        # Mock ManagedProcess
        mock_process = Mock()
        mock_process_class.return_value = mock_process

        # Test start without configure
        rosbag = Rosbag(temp_path)
        with pytest.raises(RuntimeError) as exc_info:
            rosbag.start()
        assert "must be configured" in str(exc_info.value).lower()

        # Test successful start
        rosbag.configure()
        rosbag.start()
        assert rosbag._is_running is True
        mock_process.start.assert_called_once()

        # Test stop
        rosbag.stop()
        assert rosbag._is_running is False
    finally:
        Path(temp_path).unlink()
