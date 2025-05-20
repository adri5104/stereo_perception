#!/usr/bin/env python3

import launch 
import launch_ros.actions

def generate_launch_description():
    use_sim_time = True
    color_image_topic = '/multisense/left/image_rect'
    depth_image_topic = '/multisense/left/openni_depth'
    camera_info_topic = '/multisense/left/depth/camera_info'
    camera_frame_id = 'multisense/left_camera_optical_frame'
    
    return launch.LaunchDescription([
              
        launch_ros.actions.Node(
            package='optical_flow_computation', 
            executable='optical_flow_computation',
            parameters=[{
                'image_topic': color_image_topic,
                'optical_flow_topic': '/perception_pipeline/optical_flow',
                'pyr_scale': 0.5  , 
                'levels': 6,
                'winsize': 15 , #35
                'iterations': 10,
                'poly_n': 5,
                'poly_sigma': 1.5   ,
                'flags': 0,
                'use_sim_time': use_sim_time
            }],
        ),
        
        launch_ros.actions.Node(
            package='visual_odometry',
            executable='visual_odometry_node',
            parameters=[
                {'color_image_topic': color_image_topic},
                {'depth_image_topic': depth_image_topic},
                {'camera_info_topic': camera_info_topic},
                {'odometry_debug_topic': '/debug/odometry_keypoints'},
                {'camera_frame_tf_topic': '/perception_pipeline/camera_frame_tf'},
                {'max_depth_odom' : 20.0},
                {'min_depth_odom' : 0.1},
                {'odometry_debug_image' : True},
                {'frame_id': camera_frame_id},
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
                {'depth_topic': depth_image_topic},
                {'camera_info_topic': camera_info_topic},
                {'camera_frame_tf_topic': '/perception_pipeline/camera_frame_tf'},
                {'color_image_topic': color_image_topic}, 
                {'debug_image_topic_':'/debug/debug_image'},
                {'output_6d_topic' : '/perception_pipeline/output_6d'},
                {'debug_markers_topic' : '/debug/image_6d_markers'},
                {'frame_id': camera_frame_id},
                {'grid_size' : 17},
                {'debug_image_grid' : False},
                {'use_ego_motion' : False},
                {'use_ego_var' : False},
                {'sigma2_x_system' : 100.0},
                {'sigma2_y_system' : 100.0},
                {'sigma2_z_system' : 100.0},
                {'sigma2_flow_y_measurement' : 30.0},
                {'sigma2_flow_x_measurement' : 30.0},
                {'sigma2_depth_system' : 1.0  },
                {'sigma2_tx_measurement' : 10.0},
                {'sigma2_ty_measurement' : 10.0},
                {'sigma2_tz_measurement' : 10.0},
                {'sigma2_theta_measurement' : 10.0},
                {'sigma2_psi_system' : 10.0},
                {'min_depth' : 1.0},
                {'max_depth' : 15.0},
                {'min_height_' :-2.0},
                {'camera_ground_distance' : 2.0},
                {'use_sim_time': use_sim_time}
            ],
            ros_arguments= ["--log-level", "info"] 
        ),
                
        launch_ros.actions.Node(
          package='object_detector',
          executable='object_detector_node',
          name='object_detector_node',
          output='screen',
          parameters=[
            {'input_6d_topic': '/perception_pipeline/output_6d'},
            {'output_markers_topic' : '/perception_pipeline/output_markers'},
            {'output_clusters_topic' : '/perception_pipeline/output_clusters'},
            {'frame_id': camera_frame_id},
            {'eps': 0.3},
            {'minPts': 7},
            {'pos_weight': 1.0},
            {'vel_weight': 0.02},
            {'vel_threshold': 0.2},
            {'use_sim_time': use_sim_time}
            
          ],
          
        )
    ])  