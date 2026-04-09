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
find_package(angles REQUIRED)
find_package(image_geometry REQUIRED)

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
  angles
  image_geometry
  cv_bridge
)

include_directories(
  include
  ${dddmr_sys_core_INCLUDE_DIRS}
)

# cit tag
add_library(cit_tag_detector
  src/apriltag_lib/cit_common_functions.cpp
)
target_link_libraries(cit_tag_detector
  apriltag::apriltag
  Eigen3::Eigen
  opencv_core
)
ament_target_dependencies(cit_tag_detector
  ${dependencies}
)
set_property(TARGET cit_tag_detector PROPERTY
    POSITION_INDEPENDENT_CODE ON
)

add_library(apriltag_tracking SHARED
  src/apriltag_tracking.cpp
)
target_link_libraries(apriltag_tracking
  cit_tag_detector
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
  cit_tag_detector
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
ament_export_libraries(cit_tag_detector apriltag_tracking)
ament_export_dependencies(${dependencies})
ament_package()
