"""
Base classes and interfaces for SLAM algorithms
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable, Dict, Any, Tuple
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2


class SlamStatus(Enum):
    """Status of the SLAM system"""
    UNINITIALIZED = "uninitialized"
    INITIALIZING = "initializing"
    TRACKING = "tracking"
    LOST = "lost"
    STOPPED = "stopped"
    ERROR = "error"


@dataclass
class SlamConfig:
    """Configuration for SLAM algorithms"""
    # ROS2 topic names
    rgb_topic: str = "/camera/rgb/image_raw"
    depth_topic: str = "/camera/depth/image_raw"
    camera_info_topic: str = "/camera/rgb/camera_info"
    imu_topic: Optional[str] = None  # For D435i IMU support
    odom_topic: Optional[str] = None
    
    # Frame IDs
    robot_base_frame: str = "base_footprint"
    global_frame: str = "map"
    odom_frame: str = "odom"
    
    # Visualization
    enable_visualization: bool = False  # Disable rtabmap_viz, use Rerun instead
    
    # Performance
    enable_loop_closing: bool = True
    map_publish_frequency_ms: int = 1000
    
    # Simulation
    use_sim_time: bool = False  # Use simulation time (required for rosbag playback)
    
    # Docker settings
    use_gpu: bool = False
    ros_domain_id: int = 55
    
    # Custom parameters
    custom_params: Dict[str, Any] = None
    
    def __post_init__(self):
        if self.custom_params is None:
            self.custom_params = {}


class SlamBase(ABC):
    """Abstract base class for SLAM algorithms"""
    
    def __init__(self):
        self._config: Optional[SlamConfig] = None
        self._status: SlamStatus = SlamStatus.UNINITIALIZED
        self._pose_callback: Optional[Callable[[np.ndarray, np.ndarray], None]] = None
        self._map_callback: Optional[Callable[[np.ndarray], None]] = None
        self._status_callback: Optional[Callable[[SlamStatus], None]] = None
    
    @abstractmethod
    def configure(self, config: SlamConfig) -> None:
        """
        Configure the SLAM algorithm
        
        Args:
            config: SLAM configuration
        """
        pass
    
    @abstractmethod
    def start(self) -> None:
        """Start the SLAM algorithm"""
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """Stop the SLAM algorithm"""
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """Reset the SLAM algorithm"""
        pass
    
    @property
    def status(self) -> SlamStatus:
        """Get current status"""
        return self._status
    
    @abstractmethod
    def get_current_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Get current robot pose
        
        Returns:
            Tuple of (position, quaternion) or None if not available
            position: [x, y, z]
            quaternion: [x, y, z, w]
        """
        pass
    
    @abstractmethod
    def get_current_map(self) -> Optional[np.ndarray]:
        """
        Get current map as point cloud
        
        Returns:
            Nx3 array of points or None if not available
        """
        pass
    
    @abstractmethod
    def save_map(self, filepath: str) -> bool:
        """
        Save map to file
        
        Args:
            filepath: Path to save the map
            
        Returns:
            True if saved successfully
        """
        pass
    
    @abstractmethod
    def load_map(self, filepath: str) -> bool:
        """
        Load map from file
        
        Args:
            filepath: Path to load the map from
            
        Returns:
            True if loaded successfully
        """
        pass
    
    def register_pose_callback(self, callback: Callable[[np.ndarray, np.ndarray], None]) -> None:
        """
        Register callback for pose updates
        
        Args:
            callback: Function to call with (position, quaternion)
        """
        self._pose_callback = callback
    
    def register_map_callback(self, callback: Callable[[np.ndarray], None]) -> None:
        """
        Register callback for map updates
        
        Args:
            callback: Function to call with point cloud
        """
        self._map_callback = callback
    
    def register_status_callback(self, callback: Callable[[SlamStatus], None]) -> None:
        """
        Register callback for status updates
        
        Args:
            callback: Function to call with status
        """
        self._status_callback = callback
    
    @abstractmethod
    def get_algorithm_name(self) -> str:
        """Get algorithm name"""
        pass
    
    @abstractmethod
    def get_version(self) -> str:
        """Get algorithm version"""
        pass
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop()
