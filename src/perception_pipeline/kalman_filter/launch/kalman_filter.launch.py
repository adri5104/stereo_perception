#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    """
    Launch file for the KalmanFilterNode with topic parameters.
    """
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='kalman_filter',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            output='screen',
            parameters=[
                {'optical_flow_topic': '/optical_flow'},
                {'disparity_topic': '/device_0/sensor_0/Depth_0/image/data'},
                {'camera_info_topic': '/rgb/camera_info'},
                {'color_image_topic':'/device_0/sensor_1/Color_0/image/data'}, 
                {'debug_image_topic_':'debug_image'},
                {'output_6d_topic' : 'output_6d'},
            ]
        )
    ])  
