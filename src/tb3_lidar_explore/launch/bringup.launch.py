from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    base_frame  = LaunchConfiguration('base_frame',  default='base_footprint')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('base_frame',   default_value='base_footprint'),

        Node(
            package='tb3_lidar_explore',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': base_frame,
                'min_frontier_size': 12,
                'attempt_timeout_sec': 45.0
            }]
        )
    ])
