import os
import yaml
import launch
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = "/home/ubuntu/config/criticallity_pipeline_params.yaml"

    # Load YAML config
    with open(config_file_path, 'r') as f:
        full_config = yaml.safe_load(f)
        
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
        conditional_node("ttc_calculator", "ttc_calculator_node", "ttc_calculator_node"),
        conditional_node("foxglove_bridge_node", "foxglove_bridge", "foxglove_bridge"),
    ]

    # Assemble LaunchDescription
    ld = launch.LaunchDescription()
    for node in nodes:
        if node is not None:
            ld.add_action(node)

    return ld