cmake_minimum_required(VERSION 3.5)
project(edgar_odom_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(edgar_odom_bridge_node src/edgar_odom_bridge_node.cpp)

ament_target_dependencies(edgar_odom_bridge_node rclcpp nav_msgs geometry_msgs 
  tf2 tf2_geometry_msgs)

install(TARGETS
  edgar_odom_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
