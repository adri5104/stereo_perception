#!/usr/bin/env python3

import launch 
import launch_ros.actions

def generate_launch_description():
    use_sim_time = True
    
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
              {"color_image_sub_topic": "/device_0/sensor_1/Color_0/image/data"},
              {"depth_image_sub_topic": "/device_0/sensor_0/Depth_0/image/data"},
              {"camera_info_sub_topic": "/device_0/sensor_0/Depth_0/info/camera_info"},
              {"publish_color_image": False},
              {"publish_depth_image": False},
              {'use_sim_time': use_sim_time}
            ]  
        ),
        
        launch_ros.actions.Node(
            package='optical_flow_computation', 
            executable='optical_flow_computation',
            parameters=[{
                'image_topic': '/device_0/sensor_1/Color_0/image/data',
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
                {'color_image_topic': '/device_0/sensor_1/Color_0/image/data'},
                {'depth_image_topic': '/device_0/sensor_0/Depth_0/image/data'},
                {'camera_info_topic': '/perception_pipeline/camera_info_sync'},
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
                {'depth_topic': "/device_0/sensor_0/Depth_0/image/data"},
                {'camera_info_topic': '/perception_pipeline/camera_info_sync'},
                {'camera_frame_tf_topic': '/perception_pipeline/camera_frame_tf'},
                {'color_image_topic':"/device_0/sensor_1/Color_0/image/data"}, 
                {'debug_image_topic_':'/debug/debug_image'},
                {'output_6d_topic' : '/perception_pipeline/output_6d'},
                {'debug_markers_topic' : '/debug/image_6d_markers'},
                {'grid_size' : 8},
                {'debug_image_grid' : False},
                {'use_ego_motion' : True},
                {'use_ego_var' : True},
                {'sigma2_x_system' : 10.0},
                {'sigma2_y_system' : 10.0},
                {'sigma2_z_system' : 10.0},
                {'sigma2_flow_y_measurement' : 3.0},
                {'sigma2_flow_x_measurement' : 3.0},
                {'sigma2_depth_system' : 0.5},
                {'sigma2_tx_measurement' : 0.5},
                {'sigma2_ty_measurement' : 0.5},
                {'sigma2_tz_measurement' : 0.005},
                {'sigma2_theta_measurement' : 8.0},
                {'sigma2_phi_measurement' : 8.0},
                {'sigma2_psi_system' : 8.0},
                {'min_depth' : 2.0},
                {'max_depth' : 8.0},
                {'min_height_' :-1.0},
                {'camera_ground_distance' : 1.7},
                {'use_sim_time': use_sim_time}
            ],
            ros_arguments= ["--log-level", "info"] 
        ),
        #        
        launch_ros.actions.Node(
          package='object_detector',
          executable='object_detector_node',
          name='object_detector_node',
          output='screen',
          parameters=[
            {'input_6d_topic': '/perception_pipeline/output_6d'},
            {'output_markers_topic' : '/perception_pipeline/output_markers'},
            {'output_clusters_topic' : '/perception_pipeline/output_clusters'},
            {'eps': 0.03},
            {'minPts': 5},
            {'pos_weight': 0.00},
            {'vel_weight': 1.00},
            {'vel_threshold': 1.0},
            {'use_sim_time': use_sim_time}
            
          ],
          
        ),
    ])  
