#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    
    return launch.LaunchDescription([
      
      # Static transform: car -> camera
      launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_car_to_camera',
        arguments=[
          '0', '0', '1',  # Translation: x, y, z
          '0', '0', '0', '1',  # Rotation (quaternion): x, y, z, w
          'car',  # Parent frame
          'camera'  # Child frame
        ]
      ),

      # Static transform: camera -> camera_optical_frame
      launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_optical',
        arguments=[
          '0', '0', '0',  # Translation: x, y, z
          '-1.5708', '0', '-1.5708',  # Rotation (Euler angles in radians): roll, pitch, yaw
          'camera',  # Parent frame
          'camera_optical_frame'  # Child frame
        ]
      ),

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
          'pyr_scale': 0.5, 
          'levels': 6,
          'winsize': 25, 
          'iterations': 20,
          'poly_n': 7,
          'poly_sigma': 1.5,
          'flags': 0,
        }],
      ),
      
      launch_ros.actions.Node(
        package='visual_odometry',
        executable='visual_odometry_node',
        output='screen',
        parameters=[
          {'color_image_topic': '/device_0/sensor_1/Color_0/image/data'},
          {'depth_image_topic': '/device_0/sensor_0/Depth_0/image/data'},
          {'camera_info_topic': '/perception_pipeline/camera_info_sync'},
          {'odometry_topic': '/perception_pipeline/odometry'},
          {'odometry_debug_topic': '/debug/odometry_keypoints'},
          {'max_depth_odom' : 15.0},
          {'min_depth_odom' : 0.0},
          {'odometry_debug_image' : True},
          {'apply_expotential_smoothing': False},
          {'exponential_alpha': 0.1 },
          {'apply_statistical_filtering': False},             
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
          {'odometry_topic': '/perception_pipeline/odometry'},
          {'color_image_topic':"/device_0/sensor_1/Color_0/image/data"}, 
          {'debug_image_topic_':'/debug/debug_image'},
          {'output_6d_topic' : '/perception_pipeline/output_6d'},
          {'debug_markers_topic' : '/debug/image_6d_markers'},
          {'grid_size' : 5},
          {'use_ego_motion' : False},
          {'sigma2_x_system' : 0.1},
          {'sigma2_y_system' : 0.1},
          {'sigma2_z_system' : 0.01},
          {'sigma2_flow_y_measurement' : 1.0},
          {'sigma2_flow_x_measurement' : 3.0},
          {'sigma2_depth_system' : 10.0},
          {'sigma2_tx_measurement' : 5.0},
          {'sigma2_ty_measurement' : 5.0},
          {'sigma2_tz_measurement' : 5.0},
          {'sigma2_theta_measurement' : 5.0},
          {'sigma2_psi_system' : 1.0},
          {'min_depth' : 0.5},
          {'max_depth' : 10.0},
        ],
        ros_arguments= ["--log-level", "info"] 
      )
    ])  
