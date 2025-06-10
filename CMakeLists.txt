cmake_minimum_required(VERSION 3.10)
project(ros2_orb_slam3)

# ─── C++ ────────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# ─── Module path ────────────────────────────────────────────────────────────────
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

# ─── Find dependencies ──────────────────────────────────────────────────────────
find_package(Python3           REQUIRED)
find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclpy              REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(cv_bridge          REQUIRED)
find_package(image_transport    REQUIRED)
find_package(OpenCV  4.2        REQUIRED)
find_package(Eigen3  3.3.0      REQUIRED)
find_package(Pangolin           REQUIRED)

# ─── Include directories ────────────────────────────────────────────────────────
include_directories(
  include                     # your public headers
  orb_slam3/include           # ORB-SLAM3 API
  orb_slam3/include/CameraModels
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ─── Build ORB-SLAM3 library ─────────────────────────────────────────────────────
add_library(orb_slam3_lib SHARED
  orb_slam3/src/System.cc
  orb_slam3/src/Tracking.cc
  orb_slam3/src/LocalMapping.cc
  orb_slam3/src/LoopClosing.cc
  orb_slam3/src/ORBextractor.cc
  orb_slam3/src/ORBmatcher.cc
  orb_slam3/src/FrameDrawer.cc
  orb_slam3/src/Converter.cc
  orb_slam3/src/MapPoint.cc
  orb_slam3/src/KeyFrame.cc
  orb_slam3/src/Atlas.cc
  orb_slam3/src/Map.cc
  orb_slam3/src/MapDrawer.cc
  orb_slam3/src/Optimizer.cc
  orb_slam3/src/Frame.cc
  orb_slam3/src/KeyFrameDatabase.cc
  orb_slam3/src/Sim3Solver.cc
  orb_slam3/src/Viewer.cc
  orb_slam3/src/ImuTypes.cc
  orb_slam3/src/G2oTypes.cc
  orb_slam3/src/CameraModels/Pinhole.cpp
  orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3/src/OptimizableTypes.cpp
  orb_slam3/src/MLPnPsolver.cpp
  orb_slam3/src/GeometricTools.cc
  orb_slam3/src/TwoViewReconstruction.cc
  orb_slam3/src/Config.cc
  orb_slam3/src/Settings.cc
)

ament_target_dependencies(orb_slam3_lib
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  Eigen3
  Pangolin
)

target_link_libraries(orb_slam3_lib
  ${OpenCV_LIBS}
  ${EIGEN3_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto
)

# ─── Install library & headers ───────────────────────────────────────────────────
install(TARGETS orb_slam3_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY orb_slam3/include/
  DESTINATION include/orb_slam3
)

# ─── Install 3rd-party shared libs ───────────────────────────────────────────────
set(DBoW2_PATH "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so")
set(g2o_PATH   "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so")

install(FILES ${DBoW2_PATH} DESTINATION lib)
install(FILES ${g2o_PATH}   DESTINATION lib)

# ─── Export for downstream packages ──────────────────────────────────────────────
ament_export_include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include"
)

ament_export_libraries(orb_slam3_lib)

ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  Eigen3
  Pangolin
)

# ─── Build the C++ node ─────────────────────────────────────────────────────────
add_executable(mono_node_cpp
  src/mono_example.cpp
  src/common.cpp
)

ament_target_dependencies(mono_node_cpp
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(mono_node_cpp PRIVATE orb_slam3_lib)

install(TARGETS mono_node_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ─── Python node ────────────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/mono_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ─── ament package ──────────────────────────────────────────────────────────────
ament_package()
