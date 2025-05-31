# ==============================
# Stage 1: Final Image with ROS 2 Humble
# ==============================

FROM osrf/ros:humble-desktop AS final


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

# Switch back to the ubuntu user
USER ubuntu


# ==============================
# Stage 2: Build Criticallity Pipeline
# ==============================
FROM final AS deploy

# Copy source files
COPY --chown=ubuntu:ubuntu src/criticallity_pipeline /home/ubuntu/ros2_ws/src/criticallity_pipeline
COPY --chown=ubuntu:ubuntu src/utils /home/ubuntu/ros2_ws/src/utils
COPY --chown=ubuntu:ubuntu src/launch_packages /home/ubuntu/ros2_ws/src/launch_packages
WORKDIR /home/ubuntu/ros2_ws/


# Build the criticallity pipeline
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to criticallity_pipeline_launch --parallel-workers $(nproc) --cmake-args -DCMAKE_CUDA_ARCHITECTURES=86"

# Build the wrapers
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to stereo_perception_autoware_bridge --parallel-workers $(nproc) --cmake-args -DCMAKE_CUDA_ARCHITECTURES=86"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to stereo_perception_autoware_auto_bridge --parallel-workers $(nproc) --cmake-args -DCMAKE_CUDA_ARCHITECTURES=86"

# Source the workspace
RUN echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set entrypoint
USER root
RUN apt-get update && apt-get install -y ros-humble-rmw-cyclonedds-cpp
RUN echo '#!/bin/bash' > /ros_entrypoint.sh
RUN echo 'set -e' >> /ros_entrypoint.sh
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /ros_entrypoint.sh 
RUN echo 'source /home/ubuntu/ros2_ws/install/local_setup.bash' >> /ros_entrypoint.sh
RUN echo 'exec "$@"' >> /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
USER ubuntu

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

