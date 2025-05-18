cmake_minimum_required(VERSION 3.8)
project(criticallity_pipeline_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ttc_calculator REQUIRED)
find_package(stereo_perception_msgs REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
