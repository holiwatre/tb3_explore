#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ---------------------------------------------------------------------------
# frontier_explorer.py
# - LIDAR 기반 프런티어(자유/미지 경계) 탐사 노드
# - Nav2의 NavigateToPose 액션을 사용해 프런티어 근처 자유셀로 이동합니다.
# - 맵(OccupancyGrid)을 구독하여 프런티어를 직접 찾아내고, 안전한 목표점을 계산합니다.
# - TF(map → base_footprint)를 사용해 로봇 현재 위치와 목표 방향(yaw)을 계산합니다.
# - 파라미터로 클러스터 최소 크기, 전송 주기, 프런티어로부터의 당겨오기 거리 등을 조정할 수 있습니다.
# ---------------------------------------------------------------------------

import math
import time
from collections import deque
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped


def yaw_to_quat(yaw: float) -> Quaternion:
    """
    Z축(yaw) 회전을 쿼터니언(Quaternion)으로 변환.
    - 여기서는 평면 이동만 고려하므로 roll=pitch=0, yaw만 사용.
    """
    q = Quaternion()
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = sy
    q.w = cy
    return q


class FrontierExplorer(Node):
    """
    LIDAR 기반 프런티어(자유↔미지 경계)를 탐색하여 Nav2로 이동 목표를 보내는 노드.
    - /map(OccupancyGrid) 구독
    - TF(map→base_footprint)로 로봇 포즈 조회
    - Nav2 액션(navigate_to_pose)으로 목표 전송
    - nav2_params.yaml에서 글로벌 플래너 allow_unknown=true가 권장됨
    """

    def __init__(self):
        super().__init__('tb3_frontier_explorer')

        # ---- 맵 구독 QoS 설정
        #  - RELIABLE: 모든 메시지를 신뢰성 있게 수신(지연이 발생해도 재전송)
        #  - KEEP_LAST(depth=1): 최신 1개만 유지(맵은 덮어쓰는 성격이므로 충분)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos
        )

        # ---- Nav2 NavigateToPose 액션 클라이언트
        #  - 서버 토픽 이름은 기본적으로 'navigate_to_pose'
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ---- TF 버퍼/리스너
        #  - map → base_footprint 변환을 조회하여 로봇의 현재 (x,y,yaw)를 얻음
        #  - cache_time은 TF 캐시 용량(여유롭게 10초)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- 내부 상태값
        self.occ: Optional[OccupancyGrid] = None     # 최신 점유 그리드
        self.last_send_time = 0.0                    # 마지막 목표 전송 시각
        self.goal_active = False                     # 현재 Nav2 목표 진행 중 여부
        self.last_goal: Optional[Tuple[float, float]] = None  # 마지막으로 보낸 목표 (디버그용)

        # ---- 파라미터 선언(런치/CLI로 오버라이드 가능)
        #  min_cluster_size : 너무 작은 프런티어 클러스터는 노이즈로 간주하여 무시
        #  send_period      : 목표 전송 최소 간격(초). 너무 잦은 갱신은 불안정/진동 유발
        #  pullback_cells   : 프런티어 경계에서 맵 내부 자유셀 쪽으로 당겨오는 한 칸 수
        #  unknown/free/occupied : 맵 데이터 값 매핑(-1, 0, 100이 일반적이지만 맵 생성기에 따라 다를 수 있음)
        self.declare_parameter('min_cluster_size', 8)
        self.declare_parameter('send_period', 2.0)
        self.declare_parameter('pullback_cells', 2)
        self.declare_parameter('unknown', -1)
        self.declare_parameter('free', 0)
        self.declare_parameter('occupied', 100)

        # ---- 주기 실행 타이머(0.5초마다 tick 실행)
        #  - 맵 수신/TF 준비 상태를 보고 프런티어를 찾아 새 목표를 보냄
        self.timer = self.create_timer(0.5, self.tick)

        self.get_logger().info('frontier_explorer started.')

    # ======================================================================
    # ROS 콜백/유틸
    # ======================================================================

    def map_callback(self, msg: OccupancyGrid):
        """최신 OccupancyGrid 메시지를 보관."""
        self.occ = msg

    def robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        TF(map → base_footprint)로부터 로봇 (x, y, yaw) 반환.
        - timeout 0.2s: TF 준비가 덜 되어 있을 수 있어 짧게 기다렸다가 실패 시 None
        - yaw 추정은 쿼터니언 → yaw 변환 공식 사용
        """
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), Duration(seconds=0.2)
            )
        except TransformException as e:
            self.get_logger().warn(f'robot_pose lookup failed: {str(e)}')
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        # 쿼터니언 → yaw 변환(표준 공식)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (tx, ty, yaw)

    def send_goal(self, wx: float, wy: float):
        """
        Nav2에 (map 프레임 기준) 목표 포즈 전송.
        - 목표 yaw는 현재 로봇 위치에서 목표점으로 바라보는 방향으로 설정(도착 안정성↑)
        - 액션 서버 준비 여부 확인 후 비동기 전송/결과 콜백 연결
        """
        # 액션 서버 준비 여부 확인(짧은 타임아웃)
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('navigate_to_pose action server not available yet.')
            return

        # 목표 메시지 구성
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'  # map 좌표계로 목표 지정
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        # 현재 위치가 있으면 목표까지의 yaw를 계산, 없으면 0으로
        pose = self.robot_pose()
        if pose is not None:
            rx, ry, _ = pose
            yaw = math.atan2(wy - ry, wx - rx)
        else:
            yaw = 0.0

        # 위치/자세 채우기
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quat(yaw)

        # 상태 갱신 및 전송
        self.get_logger().info(f'Sending goal to ({wx:.2f}, {wy:.2f}) yaw={math.degrees(yaw):.1f}°')
        self.goal_active = True
        self.last_goal = (wx, wy)
        self.last_send_time = time.time()

        send_future = self.nav_client.send_goal_async(goal)

        # --- 액션 응답 콜백: 수락 여부 확인
        def _goal_response(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected')
                self.goal_active = False
                return
            self.get_logger().info('Goal accepted')
            # 결과 응답을 비동기로 기다림
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_result_cb)

        # --- 결과 콜백: 상태 코드(4=SUCCEEDED 등) 로깅 및 내부 플래그 해제
        def _result_cb(fut):
            status = fut.result().status  # 4:SUCCEEDED, 5:ABORTED, 6:CANCELED
            self.get_logger().info(f'Goal finished with status: {status}')
            self.goal_active = False
            self.last_send_time = time.time()

        send_future.add_done_callback(_goal_response)

    # ======================================================================
    # 프런티어 탐색 로직(주기 실행)
    # ======================================================================

    def tick(self):
        """주기적으로 호출되어 프런티어를 찾고, 안전한 목표를 Nav2에 전송."""
        # 맵이 아직 없거나, 최근에 목표를 보냈거나, 현재 진행 중이면 대기
        if self.occ is None:
            return
        if self.goal_active or (time.time() - self.last_send_time) < float(self.get_parameter('send_period').value):
            return

        # ---- 맵 메타데이터/데이터 꺼내기
        occ = self.occ
        w = occ.info.width
        h = occ.info.height
        res = occ.info.resolution
        ox = occ.info.origin.position.x
        oy = occ.info.origin.position.y
        data = occ.data  # 1차원 배열: row-major, index = y*w + x

        # ---- 레이블 값(미지/자유/점유) 파라미터 적용
        unknown = int(self.get_parameter('unknown').value)
        free = int(self.get_parameter('free').value)
        occupied = int(self.get_parameter('occupied').value)

        # ---- 로봇 위치(index 좌표) 계산
        rpose = self.robot_pose()
        if rpose is None:
            return
        rx, ry, _ = rpose
        rx_i = int((rx - ox) / res)
        ry_i = int((ry - oy) / res)
        if not (0 <= rx_i < w and 0 <= ry_i < h):
            self.get_logger().warn('Robot is out of map bounds')
            return

        # ------------------------------------------------------------------
        # 1) 프런티어 셀 찾기
        #  - 정의: 현재 셀이 자유(free)이고 8이웃 중 적어도 하나가 미지(unknown)인 셀
        #  - 지도 테두리는 인덱스 범위 체크를 위해 1~(h-2), 1~(w-2)만 순회
        # ------------------------------------------------------------------
        frontier_pts: List[Tuple[int, int]] = []
        for y in range(1, h - 1):
            idx_row = y * w
            for x in range(1, w - 1):
                if data[idx_row + x] != free:
                    continue
                has_unknown = False
                for ny in (y - 1, y, y + 1):
                    for nx in (x - 1, x, x + 1):
                        if nx == x and ny == y:
                            continue
                        if data[ny * w + nx] == unknown:
                            has_unknown = True
                            break
                    if has_unknown:
                        break
                if has_unknown:
                    frontier_pts.append((x, y))

        if not frontier_pts:
            self.get_logger().info('No frontier found. Exploration likely completed.')
            return

        # ------------------------------------------------------------------
        # 2) 8-이웃 BFS 클러스터링
        #  - 인접한 프런티어 셀들을 하나의 클러스터로 묶음
        #  - 너무 작은 클러스터는 노이즈로 간주하고 버림(min_cluster_size)
        # ------------------------------------------------------------------
        pts_set = set(frontier_pts)
        visited = set()
        clusters: List[List[Tuple[int, int]]] = []
        for x, y in frontier_pts:
            if (x, y) in visited:
                continue
            q = deque([(x, y)])
            visited.add((x, y))
            cluster = []
            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for ny in (cy - 1, cy, cy + 1):
                    for nx in (cx - 1, cx, cx + 1):
                        if nx == cx and ny == cy:
                            continue
                        if 0 <= nx < w and 0 <= ny < h and (nx, ny) in pts_set and (nx, ny) not in visited:
                            visited.add((nx, ny))
                            q.append((nx, ny))
            clusters.append(cluster)

        # 작은 클러스터 제거
        min_cluster = int(self.get_parameter('min_cluster_size').value)
        clusters = [c for c in clusters if len(c) >= min_cluster]
        if not clusters:
            self.get_logger().info('Only tiny frontier clusters found; waiting for map growth.')
            return

        # ------------------------------------------------------------------
        # 3) 로봇과 가까운 클러스터 우선
        #  - 각 클러스터의 중심(평균)과 로봇 index 거리 제곱을 비교하여 정렬
        # ------------------------------------------------------------------
        def cell_dist2(c):
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            dx = cx - rx_i
            dy = cy - ry_i
            return dx * dx + dy * dy

        clusters.sort(key=cell_dist2)

        # ------------------------------------------------------------------
        # 4) 프런티어 가장자리에서 "안전한 자유셀" 선택 + 프런티어 반대 방향으로 약간 당겨오기
        #  - pullback: unknown 쪽이 아닌, 맵 내부 자유영역 쪽으로 몇 칸 이동해 목표 안정화
        #  - 또한 8이웃에 occupied(벽)가 붙어있는 셀은 회피(충돌 여유 확보)
        # ------------------------------------------------------------------
        target_world = None
        pullback = int(self.get_parameter('pullback_cells').value)

        for cluster in clusters:
            # 클러스터 중심(정수 index) 근방에서 자유셀을 작은 반경부터 탐색
            cx = int(round(sum(p[0] for p in cluster) / len(cluster)))
            cy = int(round(sum(p[1] for p in cluster) / len(cluster)))

            found = None
            # r: 확장 반경(마름모/사각 둘레 샘플링으로 간략 탐색)
            for r in range(0, 6):
                for dx in range(-r, r + 1):
                    for dy in (-r, r):
                        nx, ny = cx + dx, cy + dy
                        if 0 <= nx < w and 0 <= ny < h and data[ny * w + nx] == free:
                            found = (nx, ny)
                            break
                    if found:
                        break
                if found:
                    break

            if not found:
                # 이 클러스터 주변에 적절한 자유셀이 없으면 다음 클러스터 시도
                continue

            tx, ty = found

            # --- unknown 방향의 합벡터(ux,uy)를 구해 반대 방향(-ux,-uy)으로 pullback
            ux = uy = 0
            for ny in (ty - 1, ty, ty + 1):
                for nx in (tx - 1, tx, tx + 1):
                    if nx == tx and ny == ty:
                        continue
                    if 0 <= nx < w and 0 <= ny < h and data[ny * w + nx] == unknown:
                        ux += (nx - tx)
                        uy += (ny - ty)
            if ux != 0 or uy != 0:
                mag = math.hypot(ux, uy)
                if mag > 0.0:
                    # 단위벡터로 만들고, 자유셀을 따라 pullback_steps 만큼 이동
                    vx, vy = -ux / mag, -uy / mag
                    px, py = tx, ty
                    steps = pullback
                    while steps > 0:
                        nx = int(round(px + vx))
                        ny = int(round(py + vy))
                        if 0 <= nx < w and 0 <= ny < h and data[ny * w + nx] == free:
                            px, py = nx, ny
                            steps -= 1
                        else:
                            break
                    tx, ty = px, py

            # --- 벽(occupied) 바로 인접한 자유셀은 회피(8이웃 중 점유가 있으면 제외)
            close_to_wall = False
            for ny in (ty - 1, ty, ty + 1):
                for nx in (tx - 1, tx, tx + 1):
                    if nx == tx and ny == ty:
                        continue
                    if 0 <= nx < w and 0 <= ny < h and data[ny * w + nx] >= occupied:
                        close_to_wall = True
                        break
                if close_to_wall:
                    break
            if close_to_wall:
                # 이 후보는 버리고 다음 클러스터에서 다시 시도
                continue

            # --- index → world 좌표 변환(셀 중심으로 살짝 오프셋 +0.5)
            wx = ox + (tx + 0.5) * res
            wy = oy + (ty + 0.5) * res
            target_world = (wx, wy)
            break

        if target_world is None:
            self.get_logger().info('Could not find a safe free target near frontier.')
            return

        # 계산한 안전 목표로 이동
        self.send_goal(target_world[0], target_world[1])


def main(args=None):
    """ROS 노드 초기화 후 spin."""
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 런치에서 이미 shutdown 된 경우를 대비해 ok() 체크 후 종료
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
