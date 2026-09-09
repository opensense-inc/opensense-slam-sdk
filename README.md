<div align="center">

# Opensense SLAM SDK

**The SLAM Abstraction Layer for Robotics Platforms**

Write once, run on any sensor. Simple abstraction layer over ROS2 SLAM pipelines.


[![Documentation](https://img.shields.io/badge/📕_Documentation-blue?style=for-the-badge)](https://docs.neuronav.io)
[![Follow](https://img.shields.io/badge/Follow_@s4movar-black?style=for-the-badge&logo=x&logoColor=white)](https://x.com/s4movar)

[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)

</div>

## Quick Start

**RealSense Camera:**
```python
from neuronav import RealSenseSensor, RTABMapSLAM, run_slam

sensor = RealSenseSensor()
run_slam(sensor, RTABMapSLAM())
```

**OAK-D Camera:**
```python
from neuronav import OAKDSensor, RTABMapSLAM, run_slam

sensor = OAKDSensor()
run_slam(sensor, RTABMapSLAM())
```

**MCAP Rosbag Playback:**
```python
from neuronav import Rosbag, RTABMapSLAM, run_slam

sensor = Rosbag("/path/to/recording.mcap")
run_slam(sensor, RTABMapSLAM())
```

**With Visualization:**
```python
run_slam(sensor, RTABMapSLAM(), visualize=True)
# Opens Foxglove at ws://localhost:8765
```

## Features

- **2-Line API** - Start SLAM instantly
- **Sensor Agnostic** - RealSense, OAK-D, rosbags, or add your own
- **RTAB-Map Integration** - Production-ready visual SLAM
- **Foxglove Visualization** - Web-based 3D viewing
- **Rosbag Playback** - Test on recorded data without hardware
- **Extensible** - Add sensors in 20 min, SLAM algorithms in 1 hour

## Installation

**Docker (Recommended):**
```bash
./docker_build.sh
./docker_run.sh
```

**Local:**
```bash
pip install -e .
```

*Requires: ROS2 Humble, Python 3.8+, camera drivers ([setup guide](./CLAUDE.md))*

## Supported Hardware

**Sensors:**
- Intel RealSense D435i, D455, D415
- Luxonis OAK-D Pro/W
- MCAP rosbag files (.mcap)

**SLAM:**
- RTAB-Map RGB-D SLAM

## License

Licensed under [Apache 2.0](LICENSE).

## Credits

Built on [RTAB-Map](https://github.com/introlab/rtabmap) (Copyright (c) 2010-2025, Mathieu Labbé - IntRoLab - Université de Sherbrooke, BSD 3-Clause License).

See [ACKNOWLEDGMENTS.md](./ACKNOWLEDGMENTS.md) for complete credits and licenses.

---

**Issues?** → [GitHub Issues](https://github.com/neuronav/neuronav-slam-sdk/issues)
