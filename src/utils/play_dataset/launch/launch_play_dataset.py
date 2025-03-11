import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play','-l',
                 "/home/adrian/ros2_ws/src/stereo_perception/Datasets/rkitti_rosbag_0014.db3",
                 '--remap', '/kitti/camera_color_left/image_raw:=/left/image_raw',
                 '/kitti/camera_color_left/camera_info:=/left/camera_info',
                 '/kitti/camera_color_right/image_raw:=/right/image_raw',
                 '/kitti/camera_color_right/camera_info:=/right/camera_info',
                 
                 ],
            output='screen',
        )
    ])