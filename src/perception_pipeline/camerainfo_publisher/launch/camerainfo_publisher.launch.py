#!/usr/bin/env python3

import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    """
    Launch file for the Camera info publisher with topic parameters.
    """
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package= 'camerainfo_publisher',
            executable= 'camerainfo_publisher',
            name= 'camerainfo_publisher',
            output= 'screen',
            parameters = [
              {"color_image_pub_topic": "/perception_pipeline/color_image_sync"},
              {"depth_image_pub_topic": "/perception_pipeline/depth_image_sync"},
              {"camera_info_pub_topic": "/perception_pipeline/camera_info_sync"},
              {"color_image_sub_topic": "/device_0/sensor_1/Color_0/image/data"},
              {"depth_image_sub_topic": "/device_0/sensor_0/Depth_0/image/data"},
              {"camera_info_sub_topic": "/device_0/sensor_0/Depth_0/info/camera_info"},
              {"publish_color_image": True},
              {"publish_depth_image": True}
            ]
        )
    ])  
