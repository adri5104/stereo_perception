cmake_minimum_required(VERSION 3.8)
project(stereo_computation)

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
message("[stereo_computation] Found ROS version: ${ROS_VERSION}")

# Find required libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)

# Print OpenCV version
message("[stereo_computation] Found OpenCV version: ${OpenCV_VERSION}")


# Include directories
# Define paths for header files
set(INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS} 
)


# Module dependencies (which libraries the package depends on)
set(MODULE_LIBS
    cv_bridge 
    rclcpp 
    std_msgs
    sensor_msgs
    OpenCV 
    message_filters
)



# Install nodes
install(DIRECTORY
  nodes
  DESTINATION share/${PROJECT_NAME}
)

# Add object_detector_node (ROS2 Node)
add_executable(stereo_computation_node nodes/stereo_computation_node.cpp)
ament_target_dependencies(stereo_computation_node ${MODULE_LIBS})
target_include_directories(stereo_computation_node PUBLIC ${INCLUDE_DIRS})

# Install the node
install(TARGETS stereo_computation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
