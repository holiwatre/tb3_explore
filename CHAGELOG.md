# Changelog

## [Unreleased]
### Added
- 

### Changed
- 

### Fixed
- 

## [v1.1.0] - 2025-08-17
### Added
- 텍스트 그리드(`maze_grid.txt`)로 Gazebo world를 자동 생성하는 스크립트 `make_maze_world.py`.
### Changed
- `maze_3x3.world` 생성 로직 리팩터링 및 월드 리라이트.
### Usage
- maze_grid.txt 파일에서 '#' -> 구조물, '.' -> 빈공간
- 원하는 미로 형식대로 구성 후
- python3 make_maze_world.py --grid maze_3x3.world --out ~/tb3_explore/src/tb3_sim_bundle/worlds/maze_3x3.world

- cd ~/tb3_explore
- colcon build --symlink-install --merge-install
- source install/setup.bash
- 

## [v1.0.0] - 2025-08-16
### Added
- 초기 릴리스.
