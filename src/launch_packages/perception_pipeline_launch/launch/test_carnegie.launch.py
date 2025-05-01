#!/usr/bin/env python3

import launch 
import launch_ros.actions

def generate_launch_description():
    use_sim_time = True
    color_image_topic = '/S30/full_resolution/aux/image_rect_color'
    depth_image_topic = '/stereo/depth'
    camera_info_topic = '/S30/full_resolution/left/disparity/camera_info'
    
    return launch.LaunchDescription([
      
        # Static Transform Publisher (world -> initial_camera_frame)
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_camera',
            arguments=['0', '0', '0',  # Translation (set appropriately if needed)
                       '0.707', '0', '0', '0.707',  # Rotation (90° around X-axis)
                       'world', 'camera_optical_frame_initial'],  # Frame IDs
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        launch_ros.actions.Node(
            package= 'camerainfo_publisher',
            executable= 'camerainfo_publisher',
            name= 'camerainfo_publisher',
            output= 'screen',
            parameters = [
              {"color_image_pub_topic": "/rgb/image_rect_color"},
              {"depth_image_pub_topic": "/depth_registered/image_rect"},
              {"camera_info_pub_topic": "/perception_pipeline/camera_info_sync"},
              {"color_image_sub_topic": color_image_topic},
              {"depth_image_sub_topic": depth_image_topic},
              {"camera_info_sub_topic": camera_info_topic},
              {"publish_color_image": False},
              {"publish_depth_image": False},
              {'use_sim_time': use_sim_time}
            ]  
        ),
        
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
            output='screen',
            parameters=[
                {'color_image_topic': color_image_topic},
                {'depth_image_topic': depth_image_topic},
                {'camera_info_topic': camera_info_topic},
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

        
        
        
        
        # Best values so far
      
        launch_ros.actions.Node(
            package='kalman_filter',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            output='screen',
            parameters=[
                {'optical_flow_topic': '/perception_pipeline/optical_flow'},
                {'depth_topic': depth_image_topic},
                {'camera_info_topic': '/perception_pipeline/camera_info_sync'},
                {'camera_frame_tf_topic': '/perception_pipeline/camera_frame_tf'},
                {'color_image_topic': color_image_topic}, 
                {'debug_image_topic_':'/debug/debug_image'},
                {'output_6d_topic' : '/perception_pipeline/output_6d'},
                {'debug_markers_topic' : '/debug/image_6d_markers'},
                {'grid_size' : 12},
                {'debug_image_grid' : False},
                {'use_ego_motion' : True},
                {'use_ego_var' : True},
                {'sigma2_x_system' : 0.001},
                {'sigma2_y_system' : 0.001},
                {'sigma2_z_system' : 0.0001},
                {'sigma2_flow_y_measurement' : 1.0},
                {'sigma2_flow_x_measurement' : 1.0},
                {'sigma2_depth_measurement' : 0.1},
                {'sigma2_tx_measurement' : 0.1 },
                {'sigma2_ty_measurement' : 0.1 },
                {'sigma2_tz_measurement' : 0.1 },
                {'sigma2_rx_measurement' : 0.001 },
                {'sigma2_ry_measurement' : 0.001 },
                {'sigma2_rz_measurement' : 0.001 },
                {'min_depth' : 1.0},
                {'max_depth' : 40.0},
                {'min_height_' :-3.0},
                {'camera_ground_distance' : 1.5},
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
            {'eps': 1.0},
            {'minPts': 3},
            {'pos_weight': 0.7},
            {'vel_weight': 0.0001},
            {'vel_threshold': 0.7},
            {'use_sim_time': use_sim_time}
            
          ], 
        ),
        
        launch_ros.actions.Node(
            package='ttc_calculator',
            executable='ttc_calculator_node',
            name='ttc_calculator_node',
            output='screen',
            parameters=[
                {'input_clusters_topic': '/perception_pipeline/output_clusters'},
                {'input_twist_topic': '/perception_pipeline/ego_twist'},
                {'output_ttc_topic': '/criticallity_pipeline/ttc'},
                {'output_marker_topic': '/criticallity_pipeline/markers'},
                {'ego_width': 1.0},
                {'ego_length': 2.0},
                {'ego_height': 1.0},
                {'publish_ego_marker': True},
                {'use_sim_time': use_sim_time}
            ],
        ),
        
        # Publish some fake twist with ros2 topic pub
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/perception_pipeline/ego_twist', 'geometry_msgs/Twist', '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        )
        
    ])  
