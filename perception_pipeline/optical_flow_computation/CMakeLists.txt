cmake_minimum_required(VERSION 3.8)
project(optical_flow_computation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
  include
)

# Specify custom OpenCV path (your custom build)
#set(OpenCV_DIR set(OpenCV_DIR /usr/local/lib/cmake/opencv)
#)  # Update with your OpenCV install path
find_package(OpenCV REQUIRED)

# Print the OpenCV version
message("Found OpenCV version: ${OpenCV_VERSION}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Add executable
add_executable(${PROJECT_NAME} src/optical_flow_node.cpp src/optical_flow_main.cpp)



# Link libraries
ament_target_dependencies(
  ${PROJECT_NAME}
  cv_bridge
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
)

# Install executable and additional files
install(
  DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
