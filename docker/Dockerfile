FROM nvidia/cuda:12.2.0-base-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble

# 1. System packages, compilers, prerequisites, and locales
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    wget \
    tar \
    unzip \
    pkg-config \
    python3 \
    python3-pip \
    python3-dev \
    libglew-dev \
    libepoxy-dev \
    libeigen3-dev \
    libopencv-dev \
    libssl-dev \
    libboost-all-dev \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# 2. Install ROS2 Humble Desktop
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# 3. Build and install Pangolin
WORKDIR /opt
RUN git clone --depth 1 -b v0.6 https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin \
    && sed -i '1s/^/#include <limits>\n/' /opt/Pangolin/include/pangolin/gl/colour.h \
    && cd /opt/Pangolin \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PANGOLIN_PYTHON=OFF .. \
    && make -j4 \
    && make install \
    && ldconfig

# 4. Clone ORB_SLAM3 and unpack Vocabulary
WORKDIR /opt
RUN git clone --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git /opt/ORB_SLAM3 \
    && cd /opt/ORB_SLAM3/Vocabulary \
    && if [ -f "ORBvoc.txt.tar.gz" ]; then tar -xzf ORBvoc.txt.tar.gz && rm -f ORBvoc.txt.tar.gz; fi

# 5. Build DBoW2, g2o and compile ORB-SLAM3 Core with C++14 / OpenCV4 support
WORKDIR /opt/ORB_SLAM3
RUN sed -i 's/++11/++14/g' CMakeLists.txt \
    && cd Thirdparty/DBoW2 \
    && mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j4 \
    && cd ../../g2o \
    && mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j4 \
    && cd /opt/ORB_SLAM3 \
    && mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j2

ENV ORB_SLAM3_ROOT=/opt/ORB_SLAM3
ENV LD_LIBRARY_PATH=/opt/ORB_SLAM3/lib:/opt/ORB_SLAM3/Thirdparty/DBoW2/lib:/opt/ORB_SLAM3/Thirdparty/g2o/lib:/usr/local/lib:${LD_LIBRARY_PATH}

WORKDIR /workspace
CMD ["/bin/bash"]
