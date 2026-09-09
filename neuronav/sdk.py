"""
NeuroNav SDK - Dead-simple 2-line SLAM runner

Usage:
    from neuronav import RealSenseSensor, RTABMapSLAM, run_slam

    sensor = RealSenseSensor()
    run_slam(sensor, RTABMapSLAM())
"""

import signal
import time
from typing import Optional
from .sensors.base import SensorBase
from .slam_base import SlamBase, SlamConfig
from .sensors import RealSenseSensor, OAKDSensor, SensorConfig
from .algorithms import RTABMapSLAM


def run_slam(
    sensor: SensorBase,
    slam_algorithm: SlamBase,
    sensor_config: Optional[SensorConfig] = None,
    slam_config: Optional[SlamConfig] = None,
    duration: Optional[float] = None,
    visualize: bool = False
) -> None:
    """
    Run SLAM with specified sensor and algorithm

    Args:
        sensor: Sensor instance (RealSenseSensor, OAKDSensor, Rosbag, etc.)
        slam_algorithm: SLAM algorithm instance (RTABMapSLAM, etc.)
        sensor_config: Optional sensor configuration (uses defaults if not provided)
        slam_config: Optional SLAM configuration (uses defaults if not provided)
        duration: Optional duration in seconds (runs indefinitely if not provided)
        visualize: If True, starts Foxglove Bridge visualization automatically

    Example:
        >>> from neuronav import RealSenseSensor, RTABMapSLAM, run_slam
        >>> sensor = RealSenseSensor()
        >>> run_slam(sensor, RTABMapSLAM(), visualize=True)
    """
    # Setup signal handling for clean shutdown
    shutdown = {'flag': False}

    def signal_handler(sig, frame):
        print("\n🛑 Shutting down...")
        shutdown['flag'] = True

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    viz_instance = None
    try:
        # Configure sensor
        if sensor_config is None:
            sensor_config = SensorConfig()
        sensor.configure(sensor_config)

        # Configure SLAM
        if slam_config is None:
            slam_config = SlamConfig()
        
        from .sensors.rosbag import Rosbag
        if isinstance(sensor, Rosbag):
            slam_config.use_sim_time = True
            print("📼 Detected rosbag playback - using simulation time")
        
        slam_algorithm.configure(slam_config)

        # Start sensor
        print(f"🎥 Starting {sensor.get_sensor_name()}...")
        sensor.start()
        time.sleep(1)

        # Connect sensor topics to SLAM
        if hasattr(slam_algorithm, 'set_sensor_topics'):
            slam_algorithm.set_sensor_topics(sensor.get_ros_topics())

        # Start SLAM
        print(f"🗺️  Starting {slam_algorithm.get_algorithm_name()}...")
        slam_algorithm.start()

        print(f"\n✅ SLAM is running!")
        print(f"   Sensor: {sensor.get_sensor_name()}")
        print(f"   Algorithm: {slam_algorithm.get_algorithm_name()} v{slam_algorithm.get_version()}")
        print(f"   Status: {slam_algorithm.status.value}")

        # Start visualization if requested
        if visualize:
            from .visualization import FoxgloveVisualizer
            viz_instance = FoxgloveVisualizer()
            viz_instance.start()
            print(f"   Visualization: ws://localhost:8765")

        print("\nPress Ctrl+C to stop\n")

        # Run for specified duration or until interrupted
        start_time = time.time()
        while not shutdown['flag']:
            time.sleep(0.1)

            if duration and (time.time() - start_time) >= duration:
                break

            # Check SLAM status
            # Could add periodic status updates here

    except Exception as e:
        print(f"❌ Error: {e}")
        raise

    finally:
        # Clean shutdown
        if viz_instance:
            print("Stopping visualization...")
            viz_instance.stop()

        print("Stopping SLAM...")
        slam_algorithm.stop()

        print("Stopping sensor...")
        sensor.stop()

        print("✅ Shutdown complete")


# Convenience exports for 2-line usage
__all__ = [
    'run_slam',
    'RealSenseSensor',
    'OAKDSensor',
    'RTABMapSLAM',
    'SensorConfig',
    'SlamConfig'
]
