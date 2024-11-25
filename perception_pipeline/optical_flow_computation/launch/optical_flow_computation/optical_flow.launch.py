from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='optical_flow_computation',
            executable='optical_flow_computation',
            remappings=[
                ('left/image_raw', 'left/image_raw'),   
                ('/optical_flow', '/optical_flow'),
            ],
            parameters=[{
                'pyr_scale': 0.5  , 
                'levels': 10,
                'winsize': 11,
                'iterations': 20,
                'poly_n': 5,
                'poly_sigma': 1.0   ,
                'flags': 0,
            }],
        ),
       
        
    ])