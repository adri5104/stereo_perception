#!/usr/bin/env python3

import launch 
import launch_ros.actions

def generate_launch_description():
    use_sim_time = False
    
    return launch.LaunchDescription([    
        launch_ros.actions.Node(
            package='ttc_calculator',
            executable='ttc_calculator_node',
            name='ttc_calculator_node',
            output='screen',
            parameters=[
                {'input_clusters_topic': '/perception_pipeline/output_clusters'},
                {'input_twist_topic': '/perception_pipeline/ego_twist'},
                {'input_path_topic': '/perception_pipeline/ego_path'},
                {'output_ttc_topic': '/criticallity_pipeline/ttc'},
                {'output_marker_topic': '/criticallity_pipeline/markers'},
                {'ego_width': 1.0},
                {'ego_length': 2.0},
                {'ego_height': 1.0},
                {'publish_ego_marker': True},
                {'use_path': False},
                {'use_sim_time': use_sim_time}
            ],
        )
    ])  