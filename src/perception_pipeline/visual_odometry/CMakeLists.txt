cmake_minimum_required(VERSION 3.8)
project(visual_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Detect ROS version
execute_process(
    COMMAND bash -c "echo -n $ROS_DISTRO"
    OUTPUT_VARIABLE ROS_VERSION
)
# Define preprocessor macros for ros version if they are not already defined
if(NOT DEFINED ROS_VERSION_MACRO)
    if(ROS_VERSION STREQUAL "jazzy")
        add_compile_definitions(ROS_VERSION_JAZZY)
        set(ROS_VERSION_MACRO "ROS_VERSION_JAZZY")
    elseif(ROS_VERSION STREQUAL "humble")
        add_compile_definitions(ROS_VERSION_HUMBLE)
        set(ROS_VERSION_MACRO "ROS_VERSION_HUMBLE")
    endif()
endif()


# Include directories
include_directories(
  include
)

# Specify custom OpenCV path (your custom build)
#set(OpenCV_DIR set(OpenCV_DIR /usr/local/lib/cmake/opencv)
#)  # Update with your OpenCV install path

find_package(OpenCV REQUIRED)

# Check if OpenCV was found and print the result
if(OpenCV_FOUND)
  message(STATUS "OpenCV found!")
  message(STATUS "OpenCV version: ${OpenCV_VERSION}")
  message(STATUS "OpenCV include directory: ${OpenCV_INCLUDE_DIRS}")
  message(STATUS "OpenCV library directory: ${OpenCV_LIBRARIES}")
  message(STATUS "OpenCV config path: ${OpenCV_CONFIG_PATH}")
else()
  message(FATAL_ERROR "OpenCV not found!")
endif()

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