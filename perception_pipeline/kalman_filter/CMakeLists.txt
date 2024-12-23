cmake_minimum_required(VERSION 3.8)
project(kalman_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(
  include
) 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc  calib3d optflow)

add_executable(${PROJECT_NAME}  src/kalman_filter.cpp src/kalman_filter_node.cpp)

ament_target_dependencies(
  ${PROJECT_NAME} 
  cv_bridge
  rclcpp 
  std_msgs
  sensor_msgs
  OpenCV  
  )
install(DIRECTORY include launch DESTINATION share/${PROJECT_NAME}/)

install(
TARGETS
${PROJECT_NAME} 
DESTINATION lib/${PROJECT_NAME})

ament_package()
