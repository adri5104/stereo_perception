from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo disparity_node para calcular la disparidad
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            remappings=[
                ('left/image_rect', '/left/image_raw'),   # Si ya están rectificadas, usa los topics de imagen rectificada
                ('right/image_rect', '/right/image_raw'),
                ('left/camera_info', '/left/camera_info'),
                ('right/camera_info', '/right/camera_info'),
            ],
            parameters=[{
                'sgbm_mode': 1,  # Algoritmo SGBM
                'prefilter_size': 9,
                'prefilter_cap': 31,
                'correlation_window_size': 5,
                'min_disparity': 1,
                'disparity_range': 64,
                'uniqueness_ratio': 30.0,
                'texture_threshold': 15,
                'speckle_size': 100,
                'speckle_range': 4,
                'approximate_sync': True,
                'queue_size': 5,
            }],
        ),
       
        
    ])