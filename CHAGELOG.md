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
