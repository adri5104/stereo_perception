import os
import yaml
import launch
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = "/home/ubuntu/config/perception_pipeline_params.yaml"

    # Load YAML config
    with open(config_file_path, 'r') as f:
        full_config = yaml.safe_load(f)

    # Utility: extract node if 'launch' is True
    def conditional_node(name, package, executable):
        node_config = full_config.get(name, {})
        if node_config.get('launch', False):
            ros_params = node_config.get('ros__parameters', {})
            return Node(
                package=package,
                executable=executable,
                name=name,
                output='screen',
                parameters=[ros_params]
            )
        return None

    # Define nodes
    nodes = [
        conditional_node("camerainfo_publisher", "camerainfo_publisher", "camerainfo_publisher"),
        conditional_node("optical_flow_computation", "optical_flow_computation", "optical_flow_computation"),
        conditional_node("visual_odometry_node", "visual_odometry", "visual_odometry_node"),
        conditional_node("kalman_filter_node", "kalman_filter", "kalman_filter_node"),
        conditional_node("object_detector_node", "object_detector", "object_detector_node"),
        conditional_node("edgar_odom_bridge_node", "edgar_odom_bridge", "edgar_odom_bridge_node"),
        conditional_node("foxglove_bridge_node", "foxglove_bridge", "foxglove_bridge"),
    ]

    # Assemble LaunchDescription
    ld = launch.LaunchDescription()
    for node in nodes:
        if node is not None:
            ld.add_action(node)

    return ld
