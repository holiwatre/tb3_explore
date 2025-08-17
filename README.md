# tb3_explore (ROS 2 Humble)
https://youtu.be/QOwtcc62ebA

## Requirements
sudo apt install ros-humble-gazebo-ros-pkgs

sudo apt install ros-humble-gazebo-ros

## Build
colcon build --symlink-install --merge-install

source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 pkg prefix tb3_sim_bundle ## ~/home/$USER/tb3_explore/install 뜨면 성공! 만약 이 경로가 안 뜰 시 밑의 export 2줄 실행해서 강제 경로 설정


export AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH:$HOME/tb3_explore/install"

export COLCON_PREFIX_PATH="$HOME/tb3_explore/install"



## Run (SLAM + Nav2 + Frontier Explore)
ros2 launch tb3_sim_bundle sim_explore.launch.py

## Run (Gazebo + TurtleBot3 only)
ros2 launch tb3_sim_bundle gazebo_tb3.launch.py
