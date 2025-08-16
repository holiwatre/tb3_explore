# tb3_explore (ROS 2 Humble)

## Build
colcon build --symlink-install --merge-install
source /opt/ros/humble/setup.bash
source install/setup.bash

## Run (SLAM + Nav2 + Frontier Explore)
ros2 launch tb3_sim_bundle sim_explore.launch.py

## Run (Gazebo + TurtleBot3 only)
ros2 launch tb3_sim_bundle gazebo_tb3.launch.py
