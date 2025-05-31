import launch 
import launch_ros.actions

import launch
import launch_ros.actions

def generate_launch_description():

    config_file_path = "/home/ubuntu/config/params.yaml"

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='stereo_perception_autoware_auto_bridge',
            executable='stereo_perception_autoware_auto_bridge_node',
            name='stereo_perception_autoware_auto_bridge_node',
            output='screen',
            parameters=[config_file_path]
        ),
    ])