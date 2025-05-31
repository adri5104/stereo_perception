import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='stereo_perception_autoware_auto_bridge',
            executable='stereo_perception_autoware_auto_bridge_node',
            name='stereo_perception_autoware_auto_bridge_node',
            output='screen',
            parameters=[
                {"predicted_objects_topic": "/autoware/predicted_objects"},
                {"clustered_object_array_topic": "/autoware/clustered_objects"},
            ]
        ),
    ])
    
    
    
