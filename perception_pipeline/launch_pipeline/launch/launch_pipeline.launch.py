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
              {"publish_color_image": False},
              {"publish_depth_image": False},
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
                'levels': 6,
                'winsize': 15 , #35
                'iterations': 10,
                'poly_n': 5,
                'poly_sigma': 1.5   ,
                'flags': 0,
            }],
        ),
        
        launch_ros.actions.Node(
            package='visual_odometry',
            executable='visual_odometry_node',
            parameters=[
                {'color_image_topic': '/device_0/sensor_1/Color_0/image/data'},
                {'depth_image_topic': '/device_0/sensor_0/Depth_0/image/data'},
                {'camera_info_topic': '/perception_pipeline/camera_info_sync'},
                {'odometry_topic': '/perception_pipeline/odometry'},
                {'odometry_debug_topic': '/debug/odometry_keypoints'},
                {'max_depth_odom' : 30.0},
                {'min_depth_odom' : 0.0},
                {'odometry_debug_image' : True},
              ],
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
                {'debug_markers_topic' : '/debug/image_6d_markers'},
                {'grid_size' : 20},
                {'use_ego_motion' : False},
                {'sigma2_x_system' : 10.0},
                {'sigma2_y_system' : 10.0},
                {'sigma2_z_system' : 1.0},
                {'sigma2_flow_y_measurement' : 1.0},
                {'sigma2_flow_x_measurement' : 1.0},
                {'sigma2_depth_system' : 1.0},
                {'sigma2_tx_measurement' : 10.0},
                {'sigma2_ty_measurement' : 10.0},
                {'sigma2_tz_measurement' : 10.0},
                {'sigma2_theta_measurement' : 10.0},
                {'sigma2_psi_system' : 10.0},
                {'fx' : 416.5728759765625},
                {'fy' : 416.5728759765625},
                {'cx' : 419.969818},
                {'cy' : 242.39743041992188},
                {'min_depth' : 0.5},
                {'max_depth' : 20.0},
            ],
            ros_arguments= ["--log-level", "info"] 
        )
    ])  
