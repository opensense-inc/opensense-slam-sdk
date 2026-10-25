cmake_minimum_required(VERSION 3.10)
project(neuronav_orbslam3)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(Pangolin REQUIRED)

set(ORB_SLAM3_ROOT "/opt/ORB_SLAM3" CACHE PATH "Path to ORB_SLAM3 root directory")

include_directories(
  include
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${ORB_SLAM3_ROOT}/lib
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib
)

set(ORB_SLAM3_LIBS
  ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so
)

add_executable(orbslam3_node src/orbslam3_node.cpp)

ament_target_dependencies(orbslam3_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
  message_filters
)

target_link_libraries(orbslam3_node
  ${ORB_SLAM3_LIBS}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  -lpthread
)

install(TARGETS
  orbslam3_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
