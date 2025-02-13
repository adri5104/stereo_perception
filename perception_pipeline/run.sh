#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source your workspace (if built)
if [ -f "/ros2_perception_pipeline/install/setup.bash" ]; then
    source /ros2_perception_pipeline/install/setup.bash
fi

# Change directory to workspace
cd /ros2_ws

# Build the workspace (Optional)
colcon build --symlink-install --parallel-workers $(nproc) --cmake-args -DCMAKE_CUDA_ARCHITECTURES=86  


# Source the workspace
source /ros2_ws/install/setup.bash

# Run the launch file
ros2 launch launch_pipeline launch_pipeline_driving_test1_docker.launch.py

# Start a bash shell (or run a default command)
exec "$@"