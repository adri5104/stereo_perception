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

    ]) 