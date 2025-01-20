cmake_minimum_required(VERSION 3.8)
project(kalman_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc  calib3d optflow)

# include directories
set(INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                 $<INSTALL_INTERFACE:include>
                 ${OpenCV_INCLUDE_DIRS}
                 ${cv_bridge_INCLUDE_DIRS}
                 ${rclcpp_INCLUDE_DIRS}                 
                 )

# src files
set(SRC_FILES src/kalman_core.cpp 
              src/worldpoint.cpp)

# module libs
set (MODULE_LIBS  cv_bridge
                  rclcpp 
                  message_filters
                  std_msgs
                  sensor_msgs
                  visualization_msgs
                  OpenCV 
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
  launch
  DESTINATION share/${PROJECT_NAME})


# add nodes
add_executable(kalman_filter_node nodes/kalman_filter_node.cpp)
ament_target_dependencies(kalman_filter_node ${MODULE_LIBS})
target_link_libraries(kalman_filter_node ${PROJECT_NAME})

install(TARGETS kalman_filter_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
