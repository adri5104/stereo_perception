#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    
    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package= 'camerainfo_publisher',
            executable= 'camerainfo_publisher',
            name= 'camerainfo_publisher',
            output= 'screen',
            parameters = [
              {"color_image_pub_topic": "/view/color_image_sync"},
              {"depth_image_pub_topic": "/view/depth_image_sync"},
              {"camera_info_pub_topic": "/perception_pipeline/camera_info_sync"},
              {"color_image_sub_topic": "/device_0/sensor_1/Color_0/image/data"},
              {"depth_image_sub_topic": "/device_0/sensor_0/Depth_0/image/data"},
              {"camera_info_sub_topic": "/device_0/sensor_0/Depth_0/info/camera_info"},
              {"publish_color_image": True},
              {"publish_depth_image": True},
            ]
             
        ),
        
        launch_ros.actions.Node(
            package= 'depth_image_proc',
            executable= 'point_cloud_xyzrgb_node',
            name= 'point_cloud_xyzrgb_node',
            output= 'screen',
            remappings=[
                ('rgb/camera_info ', '/perception_pipeline/camera_info_sync'),   
                ('rgb/image_rect_color', '/view/depth_image_sync'),
                ('depth_registered/image_rect', 'perception_pipeline/optical_flow'),
            ]
             
        ),
      
        launch_ros.actions.Node(
            package='optical_flow_computation',
            executable='optical_flow_computation',
            remappings=[
                ('left/image_raw', '/device_0/sensor_1/Color_0/image/data'),   
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
                {'depth_topic': "/device_0/sensor_0/Depth_0/image/data"},
                {'camera_info_topic': '/perception_pipeline/camera_info_sync'},
                {'color_image_topic':"/device_0/sensor_1/Color_0/image/data"}, 
                {'debug_image_topic_':'/debug/debug_image'},
                {'output_6d_topic' : '/perception_pipeline/output_6d'},
            ],
            ros_arguments= ["--log-level", "info"] 
        )
    ])  
