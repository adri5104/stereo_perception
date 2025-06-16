# edgar_odom_bridge

ROS2 node that takes as input an odometry message and "deaccumulate" it, publishing the relative
transform of the camera between two frames. This node serves as an Wrapper to enable the perception pipeline to use alternative odometry sources. 