cmake_minimum_required(VERSION 3.10)
project(disparity_to_depth)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Executable
add_executable(disparity_to_depth_node src/disparity_to_depth_node.cpp)

ament_target_dependencies(disparity_to_depth_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

install(TARGETS
  disparity_to_depth_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
