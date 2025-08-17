#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# sim_explore.launch.py
# - Gazebo(TB3 스폰) → SLAM Toolbox → Nav2 Bringup → Frontier Explorer 순서로 띄우는 런치
# - Nav2 파라미터(nav2.yaml)와 SLAM 파라미터(slam_toolbox.yaml)를 외부에서 교체 가능
# - Nav2 bringup 런치 파일 경로를 자동 탐색(ament_index → /opt/ros/humble 고정 경로)
# - TimerAction으로 의존순서/기동 지연을 줘서 TF/토픽 준비가 되지 않아 생기는 오류 회피
# ---------------------------------------------------------------------------

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

try:
    # ament_index를 통해 패키지 share 디렉토리 찾기 (설치 환경에서 신뢰도 높음)
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    # 소스 환경/경량 환경 등에서 import 실패하는 경우가 있어 None 처리 후 수동 경로로 폴백
    get_package_share_directory = None


def _find_nav2_bringup():
    """
    nav2_bringup의 메인 런치 파일(navigation_launch.py 또는 bringup_launch.py) 경로를 찾는다.
    1) ament_index로 탐색 → 2) /opt/ros/humble/... 고정 경로 폴백 → 3) 실패 시 예외
    """
    # ament_index가 있다면 우선 사용(오버레이/워크스페이스 우선)
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
            # 여기선 조용히 폴백 시도(설치가 안 되어 있거나 경로가 다른 케이스)
            pass
    # 표준 설치 경로 폴백(Humble 기본)
    for p in (
        '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py',
        '/opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py',
    ):
        if Path(p).exists():
            return p
    # 둘 다 실패하면 명확한 에러로 알림
    raise FileNotFoundError("nav2_bringup launch file not found")


def generate_launch_description():
    # ---- Launch args 정의(커맨드라인에서 오버라이드 가능)
    # 예: ros2 launch tb3_sim_bundle sim_explore.launch.py use_sim_time:=true autostart:=false
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')   # 시뮬레이션 시간 사용 여부
    explore_exec = LaunchConfiguration('explore_exec', default='frontier_explorer')  # 익스플로러 실행파일명
    autostart = LaunchConfiguration('autostart', default='true')         # Nav2 Lifecycle 자동 bringup 여부

    # 패키지 루트: .../src/tb3_sim_bundle (이 파일 기준 부모의 부모)
    pkg_dir = Path(__file__).resolve().parents[1]

    # 기본 파라미터 파일 경로(필요 시 런치 인자로 교체)
    default_nav2_params = str(pkg_dir / 'nav2.yaml')             # ← Nav2 설정 파일
    default_slam_params = str(pkg_dir / 'slam_toolbox.yaml')     # ← SLAM Toolbox 설정 파일
    gazebo_launch = str(pkg_dir / 'launch' / 'gazebo_tb3.launch.py')  # TB3를 Gazebo에 스폰하는 내부 런치

    # 런치 인자(외부에서 파일 경로를 바꿔 끼울 수 있게)
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=default_nav2_params)
    slam_params_file = LaunchConfiguration('slam_params_file', default=default_slam_params)

    # 사용자가 ros2 launch 커맨드에서 바꿀 수 있도록 인자 선언
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('explore_exec', default_value='frontier_explorer'),
        DeclareLaunchArgument('autostart', default_value='true'),
        # TextSubstitution을 써야 기본값 경로 문자열이 그대로 표시됨
        DeclareLaunchArgument('nav2_params_file', default_value=TextSubstitution(text=default_nav2_params)),
        DeclareLaunchArgument('slam_params_file', default_value=TextSubstitution(text=default_slam_params)),
    ]

    # ---- Gazebo + TB3 스폰 (world/로봇 스폰을 담당하는 내부 런치 포함)
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))

    # ---- SLAM Toolbox 노드 기동
    # sync_slam_toolbox_node: 온라인 SLAM(동기형). params 파일과 use_sim_time을 함께 전달
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    # ---- Nav2 bringup 포함 (nav2_bringup의 메인 런치 파일을 찾아서 include)
    nav2_launch_path = _find_nav2_bringup()
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,   # Nav2 스택 전체에 시뮬 시간 주입
            'params_file': nav2_params_file, # ← 여기서 nav2.yaml 전달(핵심)
            'autostart': autostart,          # true면 lifecycle 자동으로 configure/activate 진행
        }.items(),
    )

    # ---- Frontier Explorer (프론티어 기반 자율 탐사 노드)
    explorer = Node(
        package='tb3_lidar_explore',
        executable=explore_exec,          # 기본 'frontier_explorer', 필요 시 다른 실행파일로 교체 가능
        name='tb3_frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],  # 탐사 노드도 시뮬 시간 사용
    )

    # ---- 기동 순서 제어
    # Gazebo가 먼저 뜨고 → 2초 뒤 SLAM+Nav2 → 다시 4초 뒤 Explorer
    # 느린 PC/첫 빌드 환경에선 2.0/4.0 값을 조금 키우면 TF/토픽 준비 지연 문제를 줄일 수 있음
    delayed_stack = TimerAction(period=2.0, actions=[slam, nav2])
    delayed_explorer = TimerAction(period=4.0, actions=[explorer])

    # ---- 런치 설명 반환
    # LogInfo는 현재 사용 중인 파라미터 파일 경로를 콘솔에 깔끔히 표시
    return LaunchDescription(
        declare_args
        + [
            LogInfo(msg=['[sim_explore] Using Nav2 params: ', nav2_params_file]),
            LogInfo(msg=['[sim_explore] Using SLAM params: ', slam_params_file]),
            gazebo,              # 1) Gazebo 먼저
            delayed_stack,       # 2) 2초 후 SLAM + Nav2
            delayed_explorer,    # 3) 4초 후 Explorer
        ]
    )
