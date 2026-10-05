cmake_minimum_required(VERSION 3.8)
project(hello_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

add_executable(
    hello_node
    src/hello_node.cpp
)
add_executable(
    string_publisher
    src/string_publisher.cpp
)
add_executable(
    string_subscriber
    src/string_subscriber.cpp
)
add_executable(
    fake_lidar_node
    src/fake_lidar_node.cpp
)
add_executable(
    scan_subscriber
    src/scan_subscriber.cpp)

ament_target_dependencies(
    hello_node
    rclcpp
)
ament_target_dependencies(
    string_publisher
    rclcpp
    std_msgs
)
ament_target_dependencies(
    string_subscriber
    rclcpp
    std_msgs
)
ament_target_dependencies(
    fake_lidar_node
    rclcpp
    sensor_msgs
)
ament_target_dependencies(
    scan_subscriber
    rclcpp
    sensor_msgs
)

install(TARGETS
  hello_node
  string_publisher
  string_subscriber
  fake_lidar_node
  scan_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()