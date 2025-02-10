#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    
    return launch.LaunchDescription([
      
        # Static Transform Publisher (world -> initial_camera_frame)
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_camera',
            arguments=['0', '0', '0',  # Translation (set appropriately if needed)
                       '0.707', '0', '0', '0.707',  # Rotation (90° around X-axis)
                       'world', 'camera_optical_frame_initial'],  # Frame IDs
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
              {"publish_color_image": True},
              {"publish_depth_image": True},
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
                {'odometry_debug_topic': '/debug/odometry_keypoints'},
                {'camera_frame_tf_topic': '/perception_pipeline/camera_frame_tf'},
                {'max_depth_odom' : 20.0},
                {'min_depth_odom' : 0.1},
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
                {'camera_frame_tf_topic': '/perception_pipeline/camera_frame_tf'},
                {'color_image_topic':"/device_0/sensor_1/Color_0/image/data"}, 
                {'debug_image_topic_':'/debug/debug_image'},
                {'output_6d_topic' : '/perception_pipeline/output_6d'},
                {'debug_markers_topic' : '/debug/image_6d_markers'},
                {'grid_size' : 20},
                {'use_ego_motion' : True},
                {'sigma2_x_system' : 4.0},
                {'sigma2_y_system' : 4.0},
                {'sigma2_z_system' : 1.0},
                {'sigma2_flow_y_measurement' : 2.0},
                {'sigma2_flow_x_measurement' : 2.0},
                {'sigma2_depth_system' : 1.0},
                {'sigma2_tx_measurement' : 10.0},
                {'sigma2_ty_measurement' : 10.0},
                {'sigma2_tz_measurement' : 10.0},
                {'sigma2_theta_measurement' : 10.0},
                {'sigma2_psi_system' : 10.0},
                {'min_depth' : 3.5},
                {'max_depth' : 18.0},
                {'min_height_' : -2.0},
                {'max_height' : 4.0},
            ],
            ros_arguments= ["--log-level", "info"] 
        ),
        
        launch_ros.actions.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[
                {'send_buffer_limit:': 1000000000 },
            ],
        ),
        
        launch_ros.actions.Node(
          package='object_detector',
          executable='object_detector_node',
          name='object_detector_node',
          output='screen',
          parameters=[
            {'input_6d_topic': '/perception_pipeline/output_6d'},
            {'output_markers_topic' : '/perception_pipeline/output_markers'},
            {'eps': 20.0},
            {'minPts': 10},
            {'pos_weight': 1.0},
            {'vel_weight': 0.0}
          ],
          
        )
    ])  
