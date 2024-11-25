# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec python3 kitti2bag.py "$@"