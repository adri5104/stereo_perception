#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source your workspace (if built)
if [ -f "/ros2_perception_pipeline/install/setup.bash" ]; then
    source /ros2_perception_pipeline/install/setup.bash
fi

# Change directory to workspace
cd /perception_pipeline_ws

# Build the workspace (Optional)
colcon build --symlink-install

# Start a bash shell (or run a default command)
exec "$@"
