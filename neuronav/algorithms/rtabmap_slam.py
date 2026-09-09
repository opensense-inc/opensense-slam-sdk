"""
RTAB-Map SLAM algorithm implementation
"""

import subprocess
import time
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from typing import Optional, Tuple, Dict, Any
import numpy as np
from ..slam_base import SlamBase, SlamStatus, SlamConfig


class RTABMapSLAM(SlamBase):
    """RTAB-Map SLAM algorithm"""

    def __init__(self):
        super().__init__()
        self._odom_process: Optional[subprocess.Popen] = None
        self._slam_process: Optional[subprocess.Popen] = None
        self._viz_process: Optional[subprocess.Popen] = None
        self._sensor_topics: Optional[Dict[str, str]] = None
        self._current_pose: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._pose_subscriber = None
        self._rclpy_initialized = False

    def configure(self, config: SlamConfig) -> None:
        """Configure RTAB-Map SLAM"""
        self._config = config
        self._status = SlamStatus.INITIALIZING

    def set_sensor_topics(self, topics: Dict[str, str]) -> None:
        """
        Set sensor topics from sensor adapter

        Args:
            topics: Dictionary with keys 'rgb', 'depth', 'camera_info', 'imu' (optional)
        """
        self._sensor_topics = topics

    def start(self) -> None:
        """Start RTAB-Map SLAM"""
        if not self._sensor_topics:
            raise RuntimeError("Sensor topics not set. Call set_sensor_topics() first.")

        # Initialize ROS2 Python client for pose monitoring
        if not self._rclpy_initialized:
            rclpy.init()
            self._rclpy_initialized = True

        # Determine if sensor provides RGBD topics (from rgbd_sync)
        subscribe_rgbd = 'rgbd_image' in self._sensor_topics.get('rgb', '')

        # Build topic remappings - only for IMU
        remappings = []
        if 'imu' in self._sensor_topics:
            remappings.append(('imu', self._sensor_topics['imu']))

        # 1. Launch RGBD odometry
        odom_cmd = self._build_node_command('rtabmap_odom', 'rgbd_odometry', remappings, subscribe_rgbd)
        self._odom_process = subprocess.Popen(odom_cmd)
        time.sleep(2)

        # 2. Launch RTAB-Map SLAM
        slam_cmd = self._build_node_command('rtabmap_slam', 'rtabmap', remappings, subscribe_rgbd)

        # Add -d argument to delete database (not a ROS parameter!)
        slam_cmd.append('-d')

        # Add custom parameters
        if self._config.custom_params:
            for key, value in self._config.custom_params.items():
                slam_cmd.extend(['-p', f'{key}:={value}'])

        self._slam_process = subprocess.Popen(slam_cmd)
        time.sleep(2)

        # 3. Launch visualization if enabled
        if self._config.enable_visualization:
            viz_cmd = self._build_node_command('rtabmap_viz', 'rtabmap_viz', remappings, subscribe_rgbd)
            self._viz_process = subprocess.Popen(viz_cmd)

        self._status = SlamStatus.TRACKING

    def stop(self) -> None:
        """Stop RTAB-Map SLAM"""
        if self._viz_process:
            self._viz_process.terminate()
            self._viz_process.wait(timeout=5)
            self._viz_process = None

        if self._slam_process:
            self._slam_process.terminate()
            self._slam_process.wait(timeout=5)
            self._slam_process = None

        if self._odom_process:
            self._odom_process.terminate()
            self._odom_process.wait(timeout=5)
            self._odom_process = None

        if self._rclpy_initialized:
            rclpy.shutdown()
            self._rclpy_initialized = False

        self._status = SlamStatus.STOPPED

    def reset(self) -> None:
        """Reset RTAB-Map database"""
        # Stop SLAM
        was_running = self._slam_process is not None
        if was_running:
            self.stop()
            time.sleep(1)

        # Database is reset on next start with -d flag
        if was_running:
            self.start()

    def get_current_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Get current robot pose"""
        return self._current_pose

    def get_current_map(self) -> Optional[np.ndarray]:
        """Get current map as point cloud"""
        # TODO: Subscribe to /rtabmap/mapData and extract point cloud
        return None

    def save_map(self, filepath: str) -> bool:
        """Save map to file"""
        try:
            # RTAB-Map automatically saves to ~/.ros/rtabmap.db
            # Could trigger service call to save
            return True
        except Exception:
            return False

    def load_map(self, filepath: str) -> bool:
        """Load map from file"""
        try:
            # Would need to copy database file and restart
            return True
        except Exception:
            return False

    def get_algorithm_name(self) -> str:
        """Get algorithm name"""
        return "RTAB-Map"

    def get_version(self) -> str:
        """Get algorithm version"""
        return "0.21.0"  # TODO: Query actual version

    def _build_node_command(self, package: str, executable: str, remappings: list, subscribe_rgbd: bool = False) -> list:
        """Build ROS2 node launch command"""
        cmd = ['ros2', 'run', package, executable, '--ros-args']

        # Add remappings
        for old, new in remappings:
            cmd.extend(['-r', f'{old}:={new}'])

        # Add frame parameters based on working launch file
        cmd.extend([
            '-p', f'frame_id:=oak-d-base-frame',  # OAK-D specific frame
            '-p', f'subscribe_rgbd:={str(subscribe_rgbd).lower()}',
            '-p', 'subscribe_odom_info:=true',
            '-p', 'approx_sync:=false',  # Exact sync as in working launch file
            '-p', 'wait_imu_to_init:=true',
            '-p', f'use_sim_time:={str(self._config.use_sim_time).lower()}'  # Simulation time support
        ])

        return cmd

    def _pose_callback(self, msg: PoseStamped):
        """Callback for pose updates"""
        pos = msg.pose.position
        orient = msg.pose.orientation

        position = np.array([pos.x, pos.y, pos.z])
        quaternion = np.array([orient.x, orient.y, orient.z, orient.w])

        self._current_pose = (position, quaternion)

        if self._pose_callback:
            self._pose_callback(position, quaternion)
