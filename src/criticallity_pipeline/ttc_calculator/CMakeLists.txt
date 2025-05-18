cmake_minimum_required(VERSION 3.8)
project(ttc_calculator)

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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED) 


find_package(stereo_perception_msgs REQUIRED)





# Include directories
# Define paths for header files
set(INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}   
    ${stereo_perception_msgs_INCLUDE_DIRS}
)

# C++ Source files
set(SRC_FILES
  src/ttc_calculator.cpp
)

if(ROS_VERSION STREQUAL "humble")
  set(MODULE_LIBS 
      rclcpp 
      std_msgs
      sensor_msgs
      geometry_msgs
      visualization_msgs
      nav_msgs
      Eigen3
      PCL
      stereo_perception_msgs
  )
else()
  set(MODULE_LIBS
      rclcpp 
      std_msgs
      sensor_msgs
      geometry_msgs
      visualization_msgs 
      nav_msgs
      Eigen3
      PCL
      stereo_perception_msgs
      )
endif()


# Create shared library (C++)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})                   # Library name and source files
target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIRS}) # Include directories
ament_target_dependencies(${PROJECT_NAME} ${MODULE_LIBS})          # Module dependencies

# Link OpenMP to the library
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)



# Install launch files and nodes
install(DIRECTORY
  nodes
  DESTINATION share/${PROJECT_NAME}
)

# Add object_detector_node (ROS2 Node)
add_executable(ttc_calculator_node nodes/ttc_calculator_node.cpp)
ament_target_dependencies(ttc_calculator_node ${MODULE_LIBS})
target_include_directories(ttc_calculator_node PUBLIC ${INCLUDE_DIRS})


if(OpenMP_CXX_FOUND)
    target_link_libraries(ttc_calculator_node OpenMP::OpenMP_CXX)
endif()

# Install the node
install(TARGETS ttc_calculator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
