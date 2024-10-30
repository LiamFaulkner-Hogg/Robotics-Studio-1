cmake_minimum_required(VERSION 3.8)
project(sprint4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(EnvNavigation src/EnvNavigation.cpp)
ament_target_dependencies(EnvNavigation rclcpp rclcpp_action nav2_msgs geometry_msgs)

add_executable(spot_navigation src/spot_navigation.cpp)
ament_target_dependencies(spot_navigation rclcpp rclcpp_action nav2_msgs geometry_msgs)

add_executable(circle_detector src/object_detection.cpp)
ament_target_dependencies(circle_detector rclcpp sensor_msgs geometry_msgs nav_msgs map_msgs cv_bridge OpenCV tf2 tf2_ros tf2_geometry_msgs)

add_executable(spot_the_circles src/spot_the_circles.cpp)
ament_target_dependencies(spot_the_circles rclcpp sensor_msgs geometry_msgs nav_msgs map_msgs cv_bridge OpenCV tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  spot_the_circles
  EnvNavigation
  circle_detector
  spot_navigation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})


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
