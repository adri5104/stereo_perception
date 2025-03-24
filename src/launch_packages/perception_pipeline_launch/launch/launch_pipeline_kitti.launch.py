#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    use_sim_time = True
    input_rgb_topic = '/left/image_raw'
    input_gray_left_topic = '/kitti/camera_gray_left/image_raw'
    input_gray_right_topic = '/kitti/camera_gray_right/image_raw'
    input_depth_topic = '/perception_pipeline/depth_image'
    input_camera_info_topic = '/kitti/camera_gray_right/camera_info'
    
    
    return launch.LaunchDescription([
      
        launch.actions.ExecuteProcess(
          cmd=['ros2', 'bag', 'play', '-l', '/home/adrian/stereo_perception/Datasets/rkitti_rosbag_0014.db3'],
          output='screen'
        ),

        launch_ros.actions.Node(
            package='optical_flow_computation', 
            executable='optical_flow_computation',
            remappings=[
                ('left/image_raw', input_rgb_topic),   
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
                'use_sim_time': use_sim_time
              }],
            ),
            
            launch_ros.actions.Node(
              package='stereo_computation', 
              executable='stereo_computation_node',
              output='screen',
              parameters=[
                {"in_left_image_topic": input_gray_left_topic},
                {"in_right_image_topic": input_gray_right_topic},
                {"in_camera_info_topic": input_camera_info_topic},
                {"out_depth_image_topic": "perception_pipeline/depth_image"},
                {"out_disparity_image_topic": "perception_pipeline/disparity"},
                {"block_size": 15},
                {"num_disparities": 128},
                {"pre_filter_cap": 100},
                {"speckle_window_size": 100},
                {"speckle_range": 32},
                {'use_sim_time': use_sim_time}
                ],
            ),
            
          launch_ros.actions.Node(
            package='visual_odometry',
            executable='visual_odometry_node',
            parameters=[
                {'color_image_topic': input_gray_left_topic},
                {'depth_image_topic': input_depth_topic},
                {'camera_info_topic': input_camera_info_topic},
                {'odometry_debug_topic': '/debug/odometry_keypoints'},
                {'camera_frame_tf_topic': '/perception_pipeline/camera_frame_tf'},
                {'max_depth_odom' : 30.0},
                {'min_depth_odom' : 0.0},
                {'odometry_debug_image' : True},
                {'publish_odom' : True},
                {'odom_topic' : '/perception_pipeline/odom'},
                {'publish_path' : True},
                {'path_topic' : '/perception_pipeline/path'},
                {'use_sim_time': use_sim_time}
              ],
        ),
          
        launch_ros.actions.Node(
          package='kalman_filter',
          executable='kalman_filter_node',
          name='kalman_filter_node',
          output='screen',
          parameters=[
              {'optical_flow_topic': '/perception_pipeline/optical_flow'},
              {'depth_topic': input_depth_topic},
              {'camera_info_topic': input_camera_info_topic},
              {'camera_frame_tf_topic': '/perception_pipeline/camera_frame_tf'},
              {'color_image_topic':input_rgb_topic}, 
              {'debug_image_topic_':'/debug/debug_image'},
              {'output_6d_topic' : '/perception_pipeline/output_6d'},
              {'debug_markers_topic' : '/debug/image_6d_markers'},
              {'grid_size' : 31 },
              {'debug_image_grid' : False},
              {'use_ego_motion' : True},
              {'use_ego_var' : True},
              {'sigma2_x_system' : 0.1},
              {'sigma2_y_system' : 0.1},
              {'sigma2_z_system' : 0.1},
              {'sigma2_flow_y_measurement' : 5.0},
              {'sigma2_flow_x_measurement' : 5.0},
              {'sigma2_depth_measurement' : 2.0},
              {'sigma2_tx_measurement' : 0.000001},
              {'sigma2_ty_measurement' : 0.000001},
              {'sigma2_tz_measurement' : 0.000001},
              {'sigma2_rx_measurement' : 0.000001},
              {'sigma2_ry_measurement' : 0.000001},
              {'sigma2_rz_measurement' : 0.000001},
              {'min_depth' : 1.0},
              {'max_depth' : 120.0},
              {'min_height_' :-3.0},
              {'camera_ground_distance' : 2.0},
              {'use_sim_time': use_sim_time}
          ],
          ros_arguments= ["--log-level", "info"] 
      ),

      
     
    ])  
