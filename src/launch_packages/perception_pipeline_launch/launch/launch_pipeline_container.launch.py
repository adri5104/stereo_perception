import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():

    config_file_path = "/home/ubuntu/config/perception_pipeline_params.yaml"

    

    return launch.LaunchDescription([
        
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_camera',
            arguments=['0', '0', '0',
                       '0.707', '0', '0', '0.707',
                       'world', 'camera_optical_frame_initial'],
        ),

        launch_ros.actions.Node(
            package='camerainfo_publisher',
            executable='camerainfo_publisher',
            name='camerainfo_publisher',
            output='screen',
            parameters=[config_file_path],
        ),

        launch_ros.actions.Node(
            package='optical_flow_computation', 
            executable='optical_flow_computation',
            name='optical_flow_computation',  # so it matches the YAML top-level key
            output='screen',
            parameters=[config_file_path],
        ),
        
        launch_ros.actions.Node(
            package='visual_odometry',
            executable='visual_odometry_node',
            name='visual_odometry_node',  # matches YAML
            output='screen',
            parameters=[config_file_path],
        ),

        launch_ros.actions.Node(
            package='kalman_filter',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            output='screen',
            parameters=[config_file_path],
            ros_arguments=["--log-level", "info"] 
        ),

        launch_ros.actions.Node(
            package='object_detector',
            executable='object_detector_node',
            name='object_detector_node',
            output='screen',
            parameters=[config_file_path],
        ),
    ])
