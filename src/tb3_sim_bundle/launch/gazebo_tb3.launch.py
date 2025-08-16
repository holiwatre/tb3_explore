from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    world = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([FindPackageShare('tb3_sim_bundle'), 'worlds', 'maze_3x3.world'])

    gzserver = ExecuteProcess(
        cmd=['gzserver', world_path, '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # gzserver가 /spawn_entity 서비스 올린 뒤에 로봇 스폰 런치를 포함
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('tb3_sim_bundle'), '/launch/spawn_tb3.launch.py'])
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_path),
        gzserver,
        gzclient,
        TimerAction(period=2.0, actions=[spawn]),
    ])
