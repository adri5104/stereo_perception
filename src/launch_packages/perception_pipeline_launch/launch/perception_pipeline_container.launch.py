import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():

    config_file_path = "/home/ubuntu/config/perception_pipeline_params.yaml"

    camera_info_pub = launch_ros.actions.Node(
        package='camerainfo_publisher',
        executable='camerainfo_publisher',
        name='camerainfo_publisher',
        output='screen',
        parameters=[config_file_path],
    )

    opt_flow = launch_ros.actions.Node(
        package='optical_flow_computation', 
        executable='optical_flow_computation',
        name='optical_flow_computation',  # so it matches the YAML top-level key
        output='screen',
        parameters=[config_file_path],
    )
    
    visual_odometry = launch_ros.actions.Node(
        package='visual_odometry',
        executable='visual_odometry_node',
        name='visual_odometry_node',  # matches YAML
        output='screen',
        parameters=[config_file_path],
    )

    kalman_filter = launch_ros.actions.Node(
        package='kalman_filter',
        executable='kalman_filter_node',
        name='kalman_filter_node',
        output='screen',
        parameters=[config_file_path],
        ros_arguments=["--log-level", "info"] 
    )

    object_detector = launch_ros.actions.Node(
        package='object_detector',
        executable='object_detector_node',
        name='object_detector_node',
        output='screen',
        parameters=[config_file_path],
        )
    
    edgar_odom_bridge = launch_ros.actions.Node(
        package='edgar_odom_bridge',
        executable='edgar_odom_bridge_node',
        name='edgar_odom_bridge_node',
        output='screen',
        parameters=[config_file_path],
    ) 
    
    ld = launch.LaunchDescription()
    ld.add_action(opt_flow)
    ld.add_action(camera_info_pub)
    ld.add_action(visual_odometry)
    ld.add_action(kalman_filter)
    ld.add_action(object_detector)
    #ld.add_action(edgar_odom_bridge)
    return ld
        
    
