cmake_minimum_required(VERSION 3.8)
project(object_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wno-pedantic)
endif()

# Find required libraries
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
find_package(CUDA REQUIRED)  
find_package(message_filters REQUIRED)


# Print OpenCV version
message("[ObjectDetector] Found OpenCV version: ${OpenCV_VERSION}")

# CUDA Compilation Flags
# - C++ 17 standard
# - NVIDIA GPUs with Compute Capability 5.0+.
# - Generate line information for debugging
# - Allows constexpr functions in CUDA kernels
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_86,code=sm_86")

# Include directories
# Define paths for header files
set(INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}  
    ${CUDA_INCLUDE_DIRS}  
)

# C++ Source files
set(SRC_FILES
    src/object_detector.cpp
    src/world_entity.cpp
)

# CUDA .cu source files
set(CUDA_SRC_FILES
    src/object_detector_cuda.cu
)

# Module dependencies (which libraries the package depends on)
set(MODULE_LIBS
    cv_bridge 
    rclcpp 
    std_msgs
    sensor_msgs
    visualization_msgs
    OpenCV 
    Eigen3
    PCL
    message_filters
    
)

# Create object_detector shared library (C++)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})                   # Library name and source files
target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIRS}) # Include directories
ament_target_dependencies(${PROJECT_NAME} ${MODULE_LIBS})          # Module dependencies

# Add CUDA library if CUDA files exist
if(CUDA_FOUND AND CUDA_SRC_FILES)
    cuda_add_library(${PROJECT_NAME}_cuda SHARED ${CUDA_SRC_FILES})          # Cuda Library name and source files
    target_include_directories(${PROJECT_NAME}_cuda PUBLIC ${INCLUDE_DIRS})  # Include directories
    target_link_libraries(${PROJECT_NAME}_cuda ${CUDA_CUDART_LIBRARY})       # Link CUDA runtime library

    # Link CUDA library to object_detector
    target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_cuda)              # Link CUDA library to object_detector
endif()

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

# Install the CUDA library
install(TARGETS ${PROJECT_NAME}_cuda
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


# Install launch files and nodes
install(DIRECTORY
  nodes
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Add object_detector_node (ROS2 Node)
add_executable(object_detector_node nodes/object_detector_node.cpp)
ament_target_dependencies(object_detector_node ${MODULE_LIBS})
target_include_directories(object_detector_node PUBLIC ${INCLUDE_DIRS})

# Link CUDA and OpenMP manually only once
target_link_libraries(object_detector_node ${PROJECT_NAME} ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_cudart_LIBRARY})

if(OpenMP_CXX_FOUND)
    target_link_libraries(object_detector_node OpenMP::OpenMP_CXX)
endif()

# Install the node
install(TARGETS object_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
