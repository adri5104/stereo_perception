#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    
    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='camerainfo_publisher',
            executable='camerainfo_publisher',
        ),
      
        launch_ros.actions.Node(
            package='optical_flow_computation',
            executable='optical_flow_computation',
            remappings=[
                ('left/image_raw', 'perception_pipeline/color_image_sync'),   
                ('/optical_flow', 'perception_pipeline/optical_flow'),
            ],
            parameters=[{
                'pyr_scale': 0.5  , 
                'levels': 2,
                'winsize': 11,
                'iterations': 20,
                'poly_n': 5,
                'poly_sigma': 1.0   ,
                'flags': 0,
            }],
        ),

        launch_ros.actions.Node(
            package='kalman_filter',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            output='screen',
            parameters=[
                {'optical_flow_topic': '/perception_pipeline/optical_flow'},
                {'depth_topic': 'perception_pipeline/depth_image_sync'},
                {'camera_info_topic': '/perception_pipeline/camera_info_sync'},
                {'color_image_topic':'/perception_pipeline/color_image_sync'}, 
                {'debug_image_topic_':'/perception_pipeline/debug_image'},
                {'output_6d_topic' : '/perception_pipeline/output_6d'},
            ]
        )
    ])  
