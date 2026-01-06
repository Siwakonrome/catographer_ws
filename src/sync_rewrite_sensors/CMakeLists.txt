cmake_minimum_required(VERSION 3.8)
project(sync_rewrite_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Add executable
add_executable(sync_rewrite_sensors_and_tf src/sync_rewrite_sensors_and_tf_node.cpp)

# Specify dependencies
ament_target_dependencies(sync_rewrite_sensors_and_tf
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_msgs
)

# Install C++ executable
install(TARGETS
  sync_rewrite_sensors_and_tf
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  sync_rewrite_sensors/sync_rewrite_sensors_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sync_rewrite_sensors_node
)

install(PROGRAMS
  sync_rewrite_sensors/sync_rewrite_sensors_and_tf_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sync_rewrite_sensors_and_tf_node_py
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
