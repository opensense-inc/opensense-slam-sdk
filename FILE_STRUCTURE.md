# PROJECT FILE STRUCTURE & ARCHITECTURAL WHITEPAPER

## 1. Directory Tree Layout

```
D:\workspace_orbslam3_bounty\
├── CMakeLists.txt                         # Low-level CMake build orchestration & library linking
├── package.xml                            # ROS2 Humble package manifest & dependency declarations
├── FILE_STRUCTURE.md                      # Physical file hierarchy and component specification
├── INTEGRATION_AND_RUN_GUIDE.md           # End-to-end deployment, IPC, and software-in-the-loop guide
├── ORBSLAM3_INTEGRATION_GUIDE.md          # Architectural whitepaper and sensor matrix
├── docker/
│   └── Dockerfile                         # NVIDIA CUDA 12.2 + ROS2 Humble + ORB-SLAM3 container recipe
├── neuronav/
│   └── algorithms/
│       └── orbslam3_slam.py               # Pure Python SDK algorithm wrapper implementing SlamBase
├── scripts/
│   └── download_vocab.py                  # Standalone zero-dependency Python vocabulary downloader
└── src/
    └── orbslam3_node.cpp                  # Native ROS2 C++ node with dynamic sensor profile switching
```

---

## 2. Component Whitepaper

### 2.1. `docker/Dockerfile`
* **Engineering Problem Solved**: Resolves severe version conflicts between Ubuntu 22.04 LTS system packages (GCC 11, OpenCV 4.5.4, modern Eigen3) and legacy dependencies of ORB-SLAM3 (DBoW2, g2o, Pangolin). Eliminates host system pollution by maintaining strict isolation on the `D:` drive.
* **Requirements Satisfied**:
  - Leverages official base container `nvidia/cuda:12.2.0-base-ubuntu22.04`.
  - Installs full `ros-humble-desktop`, `colcon`, and build toolchains.
  - Compiles Pangolin v0.6 with patched `<limits>` headers and disabled Python bindings (`-DBUILD_PANGOLIN_PYTHON=OFF`) to guarantee ABI compatibility with OpenCV 4 and ROS2.
  - Automatically clones and builds `DBoW2`, `g2o`, and `libORB_SLAM3.so` under `-std=c++14`.
  - Enforces capped compilation parallelism (`make -j2` for ORB_SLAM3 Core) to prevent Linux OOM-killer crashes during massive template instantiations in `Optimizer.cc`.

### 2.2. `scripts/download_vocab.py`
* **Engineering Problem Solved**: Eliminates redundant, slow, and error-prone manual downloads of the large (145 MB compressed, ~450 MB uncompressed) ORB-SLAM3 vocabulary file (`ORBvoc.txt`).
* **Requirements Satisfied**:
  - Written in pure Python 3 using standard library modules only (`urllib.request`, `tarfile`, `shutil`).
  - Idempotent execution: verifies existence and byte size of `/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt`.
  - Automatically fetches the archive from the official upstream mirror if absent, streams decompression to disk, and purges the temporary `.tar.gz` to preserve storage capacity.

### 2.3. `package.xml`
* **Engineering Problem Solved**: Integrates the custom C++ node into the standard ROS2 Humble packaging ecosystem (`ament_cmake`).
* **Requirements Satisfied**:
  - Defines the package identifier `neuronav_orbslam3`.
  - Declares build-time and runtime dependencies: `rclcpp`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `cv_bridge`, `image_transport`, and `message_filters`.
  - Enables clean discovery and execution via standard `ros2 run neuronav_orbslam3 orbslam3_node`.

### 2.4. `CMakeLists.txt`
* **Engineering Problem Solved**: Establishes correct compiler directives, header search paths, and static/shared library linking between ROS2 Humble, system OpenCV 4, Pangolin, and third-party modules bundled with ORB-SLAM3.
* **Requirements Satisfied**:
  - Enforces C++14 standard (`set(CMAKE_CXX_STANDARD 14)`).
  - Explicitly maps include paths to `/opt/ORB_SLAM3`, `/opt/ORB_SLAM3/include`, `/opt/ORB_SLAM3/Thirdparty/Sophus`, `/opt/ORB_SLAM3/Thirdparty/DBoW2`, and `/opt/ORB_SLAM3/Thirdparty/g2o`.
  - Dynamically links against `libORB_SLAM3.so`, `libDBoW2.so`, and `libg2o.so`.
  - Installs binaries and launch scripts into standard workspace lib destinations.

### 2.5. `src/orbslam3_node.cpp`
* **Engineering Problem Solved**: Eliminates hardcoded sensor constraints and prevents runtime segmentation faults when switching between different robotics sensor configurations or processing asynchronous ROS2 bags.
* **Requirements Satisfied**:
  - **Dynamic Sensor Mode Switching**: Maps string parameter `mode` to native `ORB_SLAM3::System::eSensor` enums:
    - `"Monocular"` -> `MONOCULAR`
    - `"Stereo"` -> `STEREO`
    - `"RGB-D"` -> `RGBD`
    - `"Monocular-Inertial"` -> `IMU_MONOCULAR`
    - `"Stereo-Inertial"` -> `IMU_STEREO`
    - `"RGB-D-Inertial"` -> `IMU_RGBD`
  - **Zero-Copy Image Intake**: Utilizes `cv_bridge::toCvShare` for memory mapping without heap re-allocation.
  - **Synchronized Temporal Alignment**: Leverages `message_filters::Synchronizer` with `ApproximateTime` policy for dual-camera and RGB-D streams.
  - **Inertial Buffering**: Thread-safe `std::mutex` queue for high-rate IMU streams (`/imu/data`).
  - **Telemetry Output**: Publishes `/orbslam3/camera_pose` (`geometry_msgs/PoseStamped`) and `/orbslam3/trajectory` (`nav_msgs/Path`).

### 2.6. `neuronav/algorithms/orbslam3_slam.py`
* **Engineering Problem Solved**: Bridges the low-level ROS2 C++ binary and client Python applications through a standardized, leak-free object-oriented API.
* **Requirements Satisfied**:
  - Inherits from the abstract base `SlamBase`.
  - Spawns the ROS2 subprocess using `subprocess.Popen` with `preexec_fn=os.setsid` to assign an isolated POSIX process group.
  - Implements reliable clean termination using `os.killpg(..., signal.SIGTERM)` followed by `signal.SIGKILL` fallback, completely preventing zombie processes in Linux memory.
  - Houses a non-blocking background daemon thread that caches the latest 6-DOF coordinates in memory, shielded by `threading.Lock()` for atomic queries via `get_current_pose()`.
