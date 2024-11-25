cmake_minimum_required(VERSION 3.8)
project(play_dataset)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch)
ament_package()