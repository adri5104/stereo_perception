#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source your workspace 
source /home/ubuntu/perception_pipeline_ws/install/setup.bash


# Run the launch file
ros2 launch perception_pipeline_launch launch_pipeline_container.launch.py 

# Start a bash shell (or run a default command)
exec "$@"
