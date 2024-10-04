cmake_minimum_required(VERSION 3.8)
project(sprint1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(nth_point src/nth_point.cpp)
ament_target_dependencies(nth_point rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(lidar src/Lidar.cpp)
ament_target_dependencies(lidar rclcpp sensor_msgs cv_bridge OpenCV)

# Install executables
install(TARGETS
  nth_point
  lidar
  DESTINATION lib/${PROJECT_NAME})

ament_package()

