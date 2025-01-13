cmake_minimum_required(VERSION 3.8)
project(camerainfo_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs)
find_package(sensor_msgs)
find_package(rclcpp)
find_package(cv_bridge)

add_executable(${PROJECT_NAME}  src/camerainfo_publisher.cpp)

target_link_libraries(${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    ${cv_bridge_LIBRARIES}
)
ament_target_dependencies(
  ${PROJECT_NAME} 
  rclcpp 
  std_msgs
  sensor_msgs 
  cv_bridge
  )


  install(
    TARGETS
    ${PROJECT_NAME} 
    DESTINATION lib/${PROJECT_NAME})

ament_package()
