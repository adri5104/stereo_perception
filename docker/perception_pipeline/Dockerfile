#==============================
#  Stage 1: Base with CUDA and dependencies
# ==============================
FROM nvidia/cuda:12.4.0-devel-ubuntu22.04 AS base

# Update packages
RUN apt update && apt upgrade -y

# Install essential tools
RUN apt install -y \
  build-essential \
  cmake \
  git \
  unzip \
  pkg-config \
  libgtk2.0-dev \
  libavcodec-dev \
  libavformat-dev \
  libswscale-dev \
  libtbb2 \
  libtbb-dev \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  python3-dev \
  python3-numpy \
  python3-pip \
  libxvidcore-dev \
  libx264-dev \
  libv4l-dev \
  v4l-utils \
  libopenblas-dev \
  liblapack-dev \
  libatlas-base-dev \
  gfortran \
  libhdf5-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libgflags-dev \
  libgoogle-glog-dev \
  libgphoto2-dev \
  libeigen3-dev \
  libglew-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev

# ==============================
#  Stage 2: Install CUDA toolkit and compile OpenCV with CUDA
# ==============================
FROM base AS opencv_builder

RUN apt install -y \
    libgphoto2-dev \
    libgtk-3-dev \
    libharfbuzz-dev \
    libtesseract-dev \
    libgdk-pixbuf2.0-dev



# Environment variables for CUDA
ENV PATH=/usr/local/cuda/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
ENV CUDA_HOME=/usr/local/cuda



# Detect CUDA version and adjust CUDA_ARCH_BIN
RUN export CUDA_VERSION=$(nvcc --version | grep "release" | awk '{print $6}' | cut -c2-) && \
  echo "CUDA Version Detected: $CUDA_VERSION" && \
  if [ "$CUDA_VERSION" = "12.4" ]; then \
    export CUDA_ARCH_BIN="8.0 8.6 9.0"; \
  elif [ "$CUDA_VERSION" = "12.2" ]; then \
    export CUDA_ARCH_BIN="7.5 8.0 8.6"; \
  else \
    export CUDA_ARCH_BIN="7.5 8.0"; \
  fi && \
  echo "Using CUDA_ARCH_BIN=$CUDA_ARCH_BIN"

# Clone OpenCV
WORKDIR /opt
RUN git clone --branch 4.10.0 --depth 1 https://github.com/opencv/opencv.git && \
    git clone --branch 4.10.0 --depth 1 https://github.com/opencv/opencv_contrib.git

# Create build directory
WORKDIR /opt/opencv
RUN rm -rf build && mkdir build
WORKDIR /opt/opencv/build


# Compile OpenCV with CUDA support (Optimized for Optical Flow + Feature Detection + Stereo Vision)
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
  -D WITH_CUDA=ON \
  -D WITH_CUDNN=ON \
  -D WITH_OPENCL=OFF \
  -D CUDA_FAST_MATH=ON \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D BUILD_EXAMPLES=OFF \
  -D BUILD_TESTS=OFF \
  -D BUILD_PERF_TESTS=OFF \
  -D BUILD_opencv_cudacodec=ON \
  -D BUILD_DOCS=OFF \
  -D BUILD_opencv_apps=OFF \
  -D BUILD_opencv_dnn=OFF \
  -D BUILD_opencv_flann=ON \
  -D BUILD_opencv_gapi=OFF \
  -D BUILD_opencv_java=OFF \
  -D BUILD_opencv_js=OFF \
  -D BUILD_opencv_ml=ON \
  -D BUILD_opencv_cudaoptflow=ON\  
  -D BUILD_opencv_objdetect=OFF \
  -D BUILD_opencv_photo=OFF \
  -D BUILD_opencv_python=OFF \
  -D BUILD_opencv_python_bindings_generator=OFF \
  -D BUILD_opencv_stitching=OFF \
  -D BUILD_opencv_superres=OFF \
  -D BUILD_opencv_ts=OFF \
  -D BUILD_opencv_videoio=OFF \
  -D BUILD_opencv_videostab=OFF \
  -D BUILD_opencv_cudalegacy=OFF \
  -D BUILD_opencv_cudev=ON \
  -D BUILD_opencv_features2d=ON \
  -D BUILD_opencv_cudafeatures2d=ON \ 
  -D BUILD_opencv_cudastereo=ON \
  -D BUILD_opencv_calib3d=ON \
  -D BUILD_opencv_tracking=OFF \   
  -D BUILD_opencv_bioinspired=OFF\ 
  -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
  ..
  
RUN make -j$(nproc) VERBOSE=1 && make install && ldconfig

# ==============================
# Stage 3: Final Image with ROS 2 Humble
# ==============================

FROM osrf/ros:humble-desktop AS final

# Copy compiled OpenCV from the previous stage
COPY --from=opencv_builder /usr/local /usr/local
COPY --from=opencv_builder /opt/opencv /opt/opencv
COPY --from=opencv_builder /opt/opencv_contrib /opt/opencv_contrib

# Set OpenCV as the default version
ENV OpenCV_DIR=/usr/local
ENV PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
ENV CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH

# Add non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME && \
  useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
  apt update && apt install -y sudo && \
  echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to normal user
USER $USERNAME

# Add user to video group for camera access
RUN sudo usermod --append --groups video $USERNAME

# Update packages and install dependencies
RUN sudo apt update && sudo apt upgrade -y


# Install cv_bridge dependencies
RUN sudo apt install -y ros-humble-cv-bridge

# Install PCL Dependencies
RUN sudo apt install ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-pcl-msgs

# Install foxglove bridge
RUN sudo apt install -y ros-${ROS_DISTRO}-foxglove-bridge

# Add ROS 2 to the environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Configure ROS 2 workspace
WORKDIR /perception_pipeline_ws
RUN mkdir -p src

# Switch to root
USER root

# Remove the old link and create the new one
RUN rm -f /usr/local/cuda && ln -s /usr/local/cuda-12.4 /usr/local/cuda

# Set ENV so nvcc is on PATH, 
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"
ENV CUDA_HOME="/usr/local/cuda"

RUN sudo mkdir -p /home/ubuntu/perception_pipeline_ws/src
RUN sudo mkdir  /home/ubuntu/perception_pipeline_ws/src/perception_pipeline
RUN sudo mkdir  /home/ubuntu/perception_pipeline_ws/src/launch_packages
RUN chown -R ubuntu:ubuntu /home/ubuntu/perception_pipeline_ws


# ==============================
# Stage 4: Add tracing tools
# ==============================

FROM final AS tracing

# Temporarily comment out any ROS 2 repository that may cause GPG signature errors
# (this avoids issues before we replace the key and update the repo)
RUN sed -i 's|^deb http://packages.ros.org|# deb http://packages.ros.org|' /etc/apt/sources.list.d/*.list || true && \
    sed -i 's|^deb .*ros2/ubuntu|# &|' /etc/apt/sources.list.d/*.list || true



# Clean up any old ROS key and repo files
RUN rm -f /etc/apt/sources.list.d/ros2.list && \
    rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Download and install the new ROS GPG key, and set up the ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o ros.key
RUN cp ros.key /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# Update package list again, now with the correct key and repo
RUN apt-get update

# Install tracing tools
RUN apt-get install -y \
    lttng-tools \
    liblttng-ust-dev \
    python3-lttng \
    python3-babeltrace \
    babeltrace \
    libmlpack-dev

# Setup tracing ROS2 tools
RUN mkdir -p /home/ubuntu/tracing_ws/src
WORKDIR /home/ubuntu/tracing_ws/
RUN vcs import src/ --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
WORKDIR /home/ubuntu/tracing_ws/src
RUN git clone https://github.com/ros-tracing/tracetools_analysis.git -b humble
WORKDIR /home/ubuntu/tracing_ws/
RUN rosdep update
RUN rosdep install --rosdistro humble --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
RUN colcon build --packages-up-to ros2trace ros2run tracetools_analysis performance_test --cmake-args -DPERFORMANCE_TEST_RCLCPP_ENABLED=ON
RUN echo "source /home/ubuntu/tracing_ws/install/setup.bash" >> /home/ubuntu/.bashrc

# Install expect 
RUN apt-get install -y expect

# ==============================
# Stage 5: Build Perception Pipeline
# ==============================
FROM tracing AS deploy

# Copy source files
COPY --chown=ubuntu:ubuntu src/perception_pipeline /home/ubuntu/perception_pipeline_ws/src/perception_pipeline
COPY --chown=ubuntu:ubuntu src/launch_packages /home/ubuntu/perception_pipeline_ws/src/launch_packages
COPY --chown=ubuntu:ubuntu src/utils /home/ubuntu/perception_pipeline_ws/src/utils

WORKDIR /home/ubuntu/perception_pipeline_ws/


# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to perception_pipeline_launch --parallel-workers $(nproc) --cmake-args -DCMAKE_CUDA_ARCHITECTURES=86"

# Source the workspace
RUN echo "source /home/ubuntu/perception_pipeline_ws/install/setup.bash" >> ~/.bashrc

# Set entrypoint
RUN apt-get update && apt-get install -y ros-humble-rmw-cyclonedds-cpp
RUN echo '#!/bin/bash' > /ros_entrypoint.sh
RUN echo 'set -e' >> /ros_entrypoint.sh
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /ros_entrypoint.sh 
RUN echo 'source /home/ubuntu/perception_pipeline_ws/install/local_setup.bash' >> /ros_entrypoint.sh
RUN echo 'exec "$@"' >> /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
USER ubuntu

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

