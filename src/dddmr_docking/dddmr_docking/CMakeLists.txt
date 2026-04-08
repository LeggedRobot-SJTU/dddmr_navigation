cmake_minimum_required(VERSION 3.8)
project(dddmr_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dddmr_sys_core REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core calib3d)
find_package(cv_bridge REQUIRED)
find_package(apriltag 3.2 REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  rclcpp_action
  dddmr_sys_core
  cv_bridge
)

include_directories(
  include
  ${dddmr_sys_core_INCLUDE_DIRS}
)

# database of tag functions
add_library(tags
  src/apriltag_lib/tag_functions.cpp
)
target_link_libraries(tags
  apriltag::apriltag
)
set_property(TARGET tags PROPERTY
    POSITION_INDEPENDENT_CODE ON
)

# conversion functions as template specialisation
add_library(conversion
  src/apriltag_lib/conversion.cpp
)
target_link_libraries(conversion
  apriltag::apriltag
  Eigen3::Eigen
  opencv_core
)
ament_target_dependencies(conversion
  ${dependencies}
)
set_property(TARGET conversion PROPERTY
    POSITION_INDEPENDENT_CODE ON
)

# pose estimation methods
add_library(pose_estimation
  src/apriltag_lib/pose_estimation.cpp
)
target_link_libraries(pose_estimation
  apriltag::apriltag
  Eigen3::Eigen
  opencv_calib3d
  conversion
)
ament_target_dependencies(pose_estimation
  ${dependencies}
)
set_property(TARGET pose_estimation PROPERTY
    POSITION_INDEPENDENT_CODE ON
)

add_library(apriltag_tracking SHARED
  src/apriltag_tracking.cpp
)
target_link_libraries(apriltag_tracking
  tags
  pose_estimation
)
ament_target_dependencies(apriltag_tracking
  ${dependencies}
)

add_executable(docking_node 
  src/docking_node.cpp
  src/mpc_docking.cpp
  src/tag_tracking.cpp
  src/trajectory_generator.cpp
)
target_link_libraries(docking_node
  apriltag_tracking
)
target_include_directories(docking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(docking_node PUBLIC c_std_99 cxx_std_17)  # Require C++17

ament_target_dependencies(docking_node
  ${dependencies}
)

install(TARGETS
  tags
  conversion
  pose_estimation
  apriltag_tracking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  docking_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/trigger_mpc_docking.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(tags conversion pose_estimation apriltag_tracking)
ament_export_dependencies(${dependencies})
ament_package()
