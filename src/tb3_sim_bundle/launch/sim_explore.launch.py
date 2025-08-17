#!/usr/bin/env python3
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def _find_nav2_bringup():
    # Try to locate nav2_bringup (navigation_launch.py or bringup_launch.py)
    if get_package_share_directory is not None:
        try:
            nav2_share = Path(get_package_share_directory('nav2_bringup'))
            cand1 = nav2_share / 'launch' / 'navigation_launch.py'
            cand2 = nav2_share / 'launch' / 'bringup_launch.py'
            if cand1.exists():
                return str(cand1)
            if cand2.exists():
                return str(cand2)
        except Exception:
            pass
    for p in (
        '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py',
        '/opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py',
    ):
        if Path(p).exists():
            return p
    raise FileNotFoundError("nav2_bringup launch file not found")


def generate_launch_description():
    # ---- Launch args
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    explore_exec = LaunchConfiguration('explore_exec', default='frontier_explorer')
    autostart = LaunchConfiguration('autostart', default='true')

    # Package root: .../src/tb3_sim_bundle
    pkg_dir = Path(__file__).resolve().parents[1]

    # Default parameter file locations (you said the file name is nav2.yaml)
    default_nav2_params = str(pkg_dir / 'nav2.yaml')
    default_slam_params = str(pkg_dir / 'slam_toolbox.yaml')
    gazebo_launch = str(pkg_dir / 'launch' / 'gazebo_tb3.launch.py')

    nav2_params_file = LaunchConfiguration('nav2_params_file', default=default_nav2_params)
    slam_params_file = LaunchConfiguration('slam_params_file', default=default_slam_params)

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('explore_exec', default_value='frontier_explorer'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('nav2_params_file', default_value=TextSubstitution(text=default_nav2_params)),
        DeclareLaunchArgument('slam_params_file', default_value=TextSubstitution(text=default_slam_params)),
    ]

    # ---- Gazebo + TB3 spawn
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))

    # ---- SLAM Toolbox
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    # ---- Nav2 bringup (reads nav2.yaml)
    nav2_launch_path = _find_nav2_bringup()
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,   # ← 여기서 nav2.yaml 사용
            'autostart': autostart,
        }.items(),
    )

    # ---- Frontier Explorer
    explorer = Node(
        package='tb3_lidar_explore',
        executable=explore_exec,
        name='tb3_frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # sequencing: Gazebo -> (2s) SLAM+Nav2 -> (4s) Explorer
    delayed_stack = TimerAction(period=2.0, actions=[slam, nav2])
    delayed_explorer = TimerAction(period=4.0, actions=[explorer])

    return LaunchDescription(
        declare_args
        + [
            LogInfo(msg=['[sim_explore] Using Nav2 params: ', nav2_params_file]),
            LogInfo(msg=['[sim_explore] Using SLAM params: ', slam_params_file]),
            gazebo,
            delayed_stack,
            delayed_explorer,
        ]
    )
