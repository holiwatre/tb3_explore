# Changelog
- v0.0.0

  첫번째 숫자: 대규모 업데이트

  두번째 숫자: 기능 추가

  세번재 숫자: 버그 수정

## [Unreleased]
### Added
- 

### Changed
- 

### Fixed
- 

## [v1.5.0] - 2025-08-23
### Added
- rviz에서 프런티어 군집 시각화

### Changed
-

### Usage
- rviz에서 Add - by topic - marker array 추가하면 프런티어 계산된 군집 시각화 가능

## [v1.4.0] - 2025-08-22
### Added
- localization 패키지 추가
- imu 노이즈 추가

### Changed
- 런치파일 실행 지연 변경 (기존 SLAM + Nav2 같이 실행 -> 수정 SLAM 켜지고 2초 뒤 Nav2 실행)

- 기타 사소한 수정
### Usage
- 시뮬 2개이상 컴퓨터에서 돌릴 때 ROS_DOMAIN_ID 반드시 다르게 할 것.


## [v1.3.0] - 2025-08-21
### Added

### Changed
- frontier_explorer.py 수정

  costmap 안에 골 좌표 안 던지게 수정 및 자잘한 수정
- spawn_tb3.launch.py 수정

  wheel base link 추가
- 
### Usage
- rviz에서 received goal plan 추가하면 실시간 경로계획 시각화 가능


## [v1.2.0] - 2025-08-18
### Added
- slam_toolbox.yaml # slam parameter 파일 추가


### Changed
- 맵 격자 크기 변경 (기존 격자 크기: 25cm x 25cm >> 변경 격자 크기: 40cm x 40cm)

  25cm는 너무 좁아서 터틀봇이 경로 지정하고 움직이기 힘들었음
- src/tb3_lidar_explore/tb3_lidar_explore/frontier_explorer.py

	src/tb3_sim_bundle/launch/sim_explore.launch.py

	src/tb3_sim_bundle/nav2.yaml

  주요 파일 수정 및 주석 추가

  좁은 곳에서도 움직일 수 있게 수정

  벽에 부딪히지 않게 최대한 조정 (아주 가끔 부딪힘)
- 해상도 변경 (기존 해상도: 5cm, 변경 해상도: 1cm)
### Usage
-

## [v1.1.0] - 2025-08-17
### Added
- 텍스트 그리드(`maze_grid.txt`)로 Gazebo world를 자동 생성하는 스크립트 `make_maze_world.py`.
### Changed
- `maze_3x3.world` 생성 로직 리팩터링 및 월드 리라이트.
### Usage
- maze_grid.txt 파일에서 '#' -> 구조물, '.' -> 빈공간
- 원하는 미로 형식대로 구성 후 (왼쪽 밑이 0,0 좌표 / xy좌표계라 생각)

- cd ~/tb3_explore
- sudo chown -R "$USER":"$USER" ~/tb3_explore ## 권한 부여
- python3 make_maze_world.py --grid maze_grid.txt --out ~/tb3_explore/src/tb3_sim_bundle/worlds/maze_3x3.world


- cd ~/tb3_explore
- colcon build --symlink-install --merge-install
- source install/setup.bash
- 

## [v1.0.0] - 2025-08-16
### Added
- 초기 릴리스.
