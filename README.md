# tb3_explore (ROS 2 Humble)
v1.3.0 구동 영상
https://youtu.be/nfYJaGYwewA

최신 release v1.5.0

## Requirements

sudo apt install ros-humble-navigation2

sudo apt install ros-humble-nav2-bringup

sudo apt install ros-humble-gazebo-ros-pkgs

sudo apt install ros-humble-gazebo-ros

sudo apt install ros-humble-robot-localization

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


## localization (current location echo)
ros2 run tf2_ros tf2_echo map base_link

## 텍스트 그리드(`maze_grid.txt`)로 Gazebo world를 자동 생성하는 스크립트 `make_maze_world.py`.
### Usage
- maze_grid.txt 파일에서 '#' -> 구조물, '.' -> 빈공간
- 원하는 미로 형식대로 구성 후 (왼쪽 밑이 0,0 좌표 / xy좌표계라 생각)

- cd ~/tb3_explore
- sudo chown -R "$USER":"$USER" ~/tb3_explore ## 권한 부여
- python3 make_maze_world.py --grid maze_grid.txt --out ~/tb3_explore/src/tb3_sim_bundle/worlds/maze_3x3.world

  #추가로 격자 크기와 개수를 변경하고 싶다면 밑의 arguments를 뒤에 붙이면 됨

  --cell 0.4 # 격자 크기 40cm x 40cm
  
  --cells 12 # 격자 개수 12개 x 12개


- cd ~/tb3_explore
- colcon build --symlink-install --merge-install
- source install/setup.bash
