cmake_minimum_required(VERSION 3.8)
project(stereo_perception_autoware_auto_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wno-pedantic)
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


# Print ROS version
message("[ObjectDetector] Found ROS version: ${ROS_VERSION}")

# Find required libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(stereo_perception_msgs REQUIRED)





# Include directories
# Define paths for header files
set(INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${autoware_auto_perception_msgs_INCLUDE_DIRS}
    ${stereo_perception_msgs_INCLUDE_DIRS}
)


if(ROS_VERSION STREQUAL "humble")
  set(MODULE_LIBS 
      rclcpp 
      std_msgs
      geometry_msgs
      nav_msgs
      stereo_perception_msgs
      autoware_auto_perception_msgs
  )
else()
  set(MODULE_LIBS
      rclcpp 
      std_msgs
      geometry_msgs
      nav_msgs
      stereo_perception_msgs
      autoware_auto_perception_msgs
      )
endif()

# Install launch files and nodes
install(DIRECTORY
  nodes
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Add object_detector_node (ROS2 Node)
add_executable(stereo_perception_autoware_auto_bridge_node nodes/stereo_perception_autoware_auto_bridge_node.cpp)
ament_target_dependencies(stereo_perception_autoware_auto_bridge_node ${MODULE_LIBS})
target_include_directories(stereo_perception_autoware_auto_bridge_node PUBLIC ${INCLUDE_DIRS})


if(OpenMP_CXX_FOUND)
    target_link_libraries(stereo_perception_autoware_auto_bridge_node OpenMP::OpenMP_CXX)
endif()

# Install the node
install(TARGETS stereo_perception_autoware_auto_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
