cmake_minimum_required(VERSION 3.8)
project(visual_odometry)

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
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)

# include directories
set(INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                 $<INSTALL_INTERFACE:include>
                 ${OpenCV_INCLUDE_DIRS}
                 ${cv_bridge_INCLUDE_DIRS}
                 ${rclcpp_INCLUDE_DIRS}                 
                 )

# src files
set(SRC_FILES src/visual_odometry.cpp)

# module libs
set (MODULE_LIBS  cv_bridge
                  rclcpp 
                  std_msgs
                  sensor_msgs
                  geometry_msgs
                  nav_msgs
                  OpenCV 
                  message_filters
                  tf2
                  tf2_ros
                  )

# create library
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} ${MODULE_LIBS})

# install
install( 
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY 
  nodes
  DESTINATION share/${PROJECT_NAME})


# add nodes
add_executable(visual_odometry_node nodes/visual_odometry_node.cpp)
ament_target_dependencies(visual_odometry_node ${MODULE_LIBS})
target_link_libraries(visual_odometry_node ${PROJECT_NAME})

install(TARGETS visual_odometry_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()