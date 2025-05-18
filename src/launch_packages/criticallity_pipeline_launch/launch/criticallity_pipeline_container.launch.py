import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():

    config_file_path = "/home/ubuntu/config/criticallity_pipeline_params.yaml"

    return launch.LaunchDescription([
        
        launch_ros.actions.Node(
            package='ttc_calculator',
            executable='ttc_calculator_node',
            name='ttc_calculator_node',
            output='screen',
            parameters=[config_file_path],
        ),
        
    ])