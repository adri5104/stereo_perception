import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='stereo_perception_edgar_bridge',
            executable='stereo_perception_edgar_bridge_node',
            name='stereo_perception_bridge_node',
            output='screen',
            parameters=[
                {"edgar_motion_topic": "/edgar/edgar_motion"},
                {"edgar_trajectory_topic": "/edgar/edgar_trajectory"},
                {"twist_topic": "/stereo_perception/ego_twist"},
                {"path_topic": "/stereo_perception/ego_path"},
            ]
        ),
    ])
