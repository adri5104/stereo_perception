#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    
    return launch.LaunchDescription([
      
        launch.actions.ExecuteProcess(
          cmd=['ros2', 'bag', 'play', '-l', '/home/adrian/ros2_ws/src/stereo_perception/Datasets/rkitti_rosbag_0014.db3'],
          output='screen'
        ),

        launch_ros.actions.Node(
            package='optical_flow_computation', 
            executable='optical_flow_computation',
            remappings=[
                ('left/image_raw', '/left/image_raw'),   
                ('/optical_flow', 'perception_pipeline/optical_flow'),
            ],
            parameters=[{
                'pyr_scale': 0.5  , 
                'levels': 6,
                'winsize': 15 , #35
                'iterations': 15,
                'poly_n': 7,
                'poly_sigma': 1.2,
                'flags': 0,
              }],
            ),
            
            launch_ros.actions.Node(
              package='stereo_computation', 
              executable='stereo_computation_node',
              output='screen',
              parameters=[
                {"in_left_image_topic": "/kitti/camera_gray_left/image_raw"},
                {"in_right_image_topic": "/kitti/camera_gray_right/image_raw"},
                {"in_camera_info_topic": "/kitti/camera_gray_right/camera_info"},
                {"out_depth_image_topic": "perception_pipeline/depth_image"},
                {"out_disparity_image_topic": "perception_pipeline/disparity"},
                {"block_size": 19},
                {"num_disparities": 128},
                {"pre_filter_cap": 0},
                {"pre_filter_size": 0},
                {"pre_filter_type": 0},
                {"texture_threshold": 2}
                #{"speckle_window_size": 0},
                #{"speckle_range": 0 },
                ],
            ),
            


        #launch_ros.actions.Node(
        #    package='visual_odometry',
        #    executable='visual_odometry_node',
        #    parameters=[
        #        {'color_image_topic': '/left/image_raw'},
        #        {'depth_image_topic': 'perception_pipeline/depth_image'},
        #        {'camera_info_topic': '/left/camera_info'},
        #        {'odometry_debug_topic': '/debug/odometry_keypoints'},
        #        {'camera_frame_tf_topic': '/perception_pipeline/camera_frame_tf'},
        #        {'max_depth_odom' : 100000.0},
        #        {'min_depth_odom' : 0.0},
        #        {'odometry_debug_image' : True},
        #      ],
        #),       
        
        #launch_ros.actions.Node(
        #    package='kalman_filter',
        #    executable='kalman_filter_node',
        #    name='kalman_filter_node',
        #    output='screen',
        #    parameters=[
        #        {'optical_flow_topic': '/perception_pipeline/optical_flow'},
        #        {'depth_topic': "perception_pipeline/depth_image"},
        #        {'camera_info_topic': '/kitti/camera_gray_right/camera_info'},
        #        {'camera_frame_tf_topic': '/perception_pipeline/camera_frame_tf'},
        #        {'color_image_topic':"/left/image_raw"}, 
        #        {'debug_image_topic_':'/debug/debug_image'},
        #        {'output_6d_topic' : '/perception_pipeline/output_6d'},
        #        {'debug_markers_topic' : '/debug/image_6d_markers'},
        #        {'grid_size' : 15},
        #        {'use_ego_motion' : True},
        #        {'sigma2_x_system' : 4.0},
        #        {'sigma2_y_system' : 4.0},
        #        {'sigma2_z_system' : 1.0},
        #        {'sigma2_flow_y_measurement' : 2.0},
        #        {'sigma2_flow_x_measurement' : 2.0},
        #        {'sigma2_depth_system' : 1.0},
        #        {'sigma2_tx_measurement' : 10.0},
        #        {'sigma2_ty_measurement' : 10.0},
        #        {'sigma2_tz_measurement' : 10.0},
        #        {'sigma2_theta_measurement' : 10.0},
        #        {'sigma2_psi_system' : 10.0},
        #        {'min_depth' : 3.5},
        #        {'max_depth' : 18.0},
        #        {'min_height_' : -2.0},
        #        {'max_height' : 4.0},
        #    ],
        #    ros_arguments= ["--log-level", "info"] 
        #), 
        #
        #
        launch_ros.actions.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[
                {'send_buffer_limit:': 1000000000 },
            ],
        ),

          
        
    ])  
