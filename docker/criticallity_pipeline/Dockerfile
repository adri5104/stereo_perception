# ==============================
# Stage 1: Final Image with ROS 2 Humble
# ==============================

FROM osrf/ros:humble-desktop AS final


RUN apt-get update

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

# Install foxglove bridge
RUN sudo apt install -y ros-${ROS_DISTRO}-foxglove-bridge

# Add ROS 2 to the environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Configure ROS 2 workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Switch to root
USER root


RUN sudo mkdir -p /home/ubuntu/ros2_ws/src
RUN sudo mkdir  /home/ubuntu/ros2_ws/src/criticallity_pipeline
RUN sudo mkdir  /home/ubuntu/ros2_ws/src/utils
RUN sudo mkdir  /home/ubuntu/ros2_ws/src/launch_packages
RUN chown -R ubuntu:ubuntu /home/ubuntu/ros2_ws



# ==============================
# Stage 2: Add tracing tools
# ==============================

FROM final AS tracing

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
RUN apt-get update
RUN rosdep install --rosdistro humble --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
RUN colcon build --packages-up-to ros2trace ros2run tracetools_analysis --cmake-args -DPERFORMANCE_TEST_RCLCPP_ENABLED=ON
RUN echo "source /home/ubuntu/tracing_ws/install/setup.bash" >> /home/ubuntu/.bashrc

# Install expect 
RUN apt-get install -y expect



# ==============================
# Stage 3: Build Criticallity Pipeline
# ==============================
FROM tracing AS deploy

RUN apt-get update && apt-get install -y ros-humble-rmw-cyclonedds-cpp
# Copy source files
COPY --chown=ubuntu:ubuntu src/criticallity_pipeline /home/ubuntu/ros2_ws/src/criticallity_pipeline
COPY --chown=ubuntu:ubuntu src/utils /home/ubuntu/ros2_ws/src/utils
COPY --chown=ubuntu:ubuntu src/launch_packages /home/ubuntu/ros2_ws/src/launch_packages
WORKDIR /home/ubuntu/ros2_ws/


# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build  --packages-up-to criticallity_pipeline_launch ttc_calculator --parallel-workers $(nproc) --cmake-args -DCMAKE_CUDA_ARCHITECTURES=86"

# Source the workspace
RUN echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> ~/.bashrc


# Create dir for the csv ttc logs
RUN mkdir -p /home/ubuntu/ttc_logs

# Set entrypoint
RUN echo '#!/bin/bash' > /ros_entrypoint.sh
RUN echo 'set -e' >> /ros_entrypoint.sh
RUN echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /ros_entrypoint.sh
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /ros_entrypoint.sh 
RUN echo 'exec "$@"' >> /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
USER ubuntu

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

