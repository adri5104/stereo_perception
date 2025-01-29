cmake_minimum_required(VERSION 3.8)
project(object_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wno-pedantic)

endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED) 

# print opencv version
message("[ObjectDetector] Found OpenCV version: ${OpenCV_VERSION}")

# include directories
set(INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                 $<INSTALL_INTERFACE:include>
                 ${OpenCV_INCLUDE_DIRS}
                 ${cv_bridge_INCLUDE_DIRS}
                 ${rclcpp_INCLUDE_DIRS}                 
                 )

# src files
set(SRC_FILES src/object_detector.cpp
              src/world_entity.cpp)

# module libs
  set(MODULE_LIBS  cv_bridge
                  rclcpp 
                  std_msgs
                  sensor_msgs
                  visualization_msgs
                  OpenCV 
                  Eigen3
                  PCL

                 )

# create library
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} ${MODULE_LIBS})

# Link OpenMP to the library
target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)

# Install the header files
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

# Install the launch files and nodes
install(DIRECTORY
  nodes
  launch
  DESTINATION share/${PROJECT_NAME}
)

# add nodes
add_executable(object_detector_node nodes/object_detector_node.cpp)
ament_target_dependencies(object_detector_node ${MODULE_LIBS})

# Link OpenMP to the node
target_link_libraries(object_detector_node ${PROJECT_NAME} OpenMP::OpenMP_CXX)

# Install the node
install(TARGETS object_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
