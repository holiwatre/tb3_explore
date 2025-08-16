from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_share = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'turtlebot3_burger.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # 경로/네임스페이스 토큰 정리
    robot_desc = robot_desc.replace('package://turtlebot3_description', f'file://{pkg_share}')
    robot_desc = robot_desc.replace('${namespace}', '')

    # diff_drive + ray(LaserScan) 플러그인 주입
    plugin_xml = """
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/</namespace>
        <remapping>cmd_vel:=/cmd_vel</remapping>
        <remapping>odom:=/odom</remapping>
      </ros>
      <update_rate>50</update_rate>
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.160</wheel_separation>
      <wheel_diameter>0.066</wheel_diameter>
      <odom_frame>odom</odom_frame>
      <base_frame>base_footprint</base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>

  <gazebo reference="base_scan">
    <sensor type="ray" name="lds_laser">
      <pose>0 0 0 0 0 0</pose>
      <update_rate>10</update_rate>
      <always_on>true</always_on>
      <visualize>true</visualize>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.12</min>
          <max>3.5</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=/scan</remapping>
        </ros>
        <frame_name>base_scan</frame_name>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>
"""
    robot_desc = robot_desc.replace('</robot>', plugin_xml + '\n</robot>')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_tb3',
            output='screen',
            arguments=['-entity', 'tb3', '-topic', 'robot_description', '-x', '0.5', '-y', '0.5', '-z', '0.05'],
        ),
    ])
