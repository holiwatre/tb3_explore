from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

def generate_launch_description():
    world_yaml = PathJoinSubstitution([FindPackageShare('tb3_sim_bundle'), 'maps', 'maze_3x3.yaml'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('tb3_sim_bundle'), '/launch/gazebo_tb3.launch.py'])
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py']),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('nav2_bringup'), '/launch/bringup_launch.py']),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'True',                 # SLAM 경로 사용 (map 인자도 같이 주어 안전)
            'map': world_yaml,
            'autostart': 'True'
        }.items()
    )

    actions = [gazebo, TimerAction(period=2.0, actions=[slam]), TimerAction(period=4.0, actions=[nav2])]

    # 탐색 패키지가 있으면만 포함 (없으면 건너뛰고 계속 실행)
    try:
        get_package_share_directory('tb3_lidar_explore')
        explore = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare('tb3_lidar_explore'), '/launch/bringup.launch.py'])
        )
        actions.append(TimerAction(period=5.0, actions=[explore]))
    except PackageNotFoundError:
        actions.append(LogInfo(msg='[sim_explore] tb3_lidar_explore 패키지 없음 → SLAM+Nav2만 실행'))

    return LaunchDescription(actions)
