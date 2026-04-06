cmake_minimum_required(VERSION 3.8)
project(dddmr_cmd2odom_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(cmd2odom_simulator_node src/cmd2odom_simulator_node.cpp)
target_compile_features(cmd2odom_simulator_node PUBLIC c_std_99 cxx_std_17)  # Require C++17

ament_target_dependencies(
  cmd2odom_simulator_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
)

install(TARGETS
  cmd2odom_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
