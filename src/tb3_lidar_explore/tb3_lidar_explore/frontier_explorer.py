#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
    LIDAR 기반 프런티어(자유-미지 경계)로 자동 이동하여
    지도를 채우는 간단한 탐사 노드.
    - Nav2 NavigateToPose 액션 사용
    - 글로벌 플래너는 allow_unknown=true (nav2_params.yaml에서 설정)
    """

    def __init__(self):
        super().__init__('tb3_frontier_explorer')

        # 맵 구독
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos)

        # Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF (map -> base_footprint)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 내부 상태
        self.occ: Optional[OccupancyGrid] = None
        self.last_send_time = 0.0
        self.goal_active = False
        self.last_goal: Optional[Tuple[float, float]] = None

        # 파라미터
        self.declare_parameter('min_cluster_size', 8)   # 작은 클러스터 무시
        self.declare_parameter('send_period', 2.0)      # 목표 전송 최소 간격 (초)
        self.declare_parameter('pullback_cells', 2)     # 프런티어에서 안쪽 자유영역으로 당겨오기
        self.declare_parameter('unknown', -1)           # 미지 값
        self.declare_parameter('free', 0)               # 자유 값
        self.declare_parameter('occupied', 100)         # 점유 값(또는 50+)

        # 주기 tick
        self.timer = self.create_timer(0.5, self.tick)

        self.get_logger().info('frontier_explorer started.')

    # -------------------- ROS 콜백/유틸 --------------------

    def map_callback(self, msg: OccupancyGrid):
        self.occ = msg

    def robot_pose(self) -> Optional[Tuple[float, float, float]]:
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
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (tx, ty, yaw)

    def send_goal(self, wx: float, wy: float):
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('navigate_to_pose action server not available yet.')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        pose = self.robot_pose()
        if pose is not None:
            rx, ry, _ = pose
            yaw = math.atan2(wy - ry, wx - rx)
        else:
            yaw = 0.0

        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quat(yaw)

        self.get_logger().info(f'Sending goal to ({wx:.2f}, {wy:.2f}) yaw={math.degrees(yaw):.1f}°')
        self.goal_active = True
        self.last_goal = (wx, wy)
        self.last_send_time = time.time()

        send_future = self.nav_client.send_goal_async(goal)

        def _goal_response(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected')
                self.goal_active = False
                return
            self.get_logger().info('Goal accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_result_cb)

        def _result_cb(fut):
            status = fut.result().status  # 4:SUCCEEDED, 5:ABORTED, 6:CANCELED
            self.get_logger().info(f'Goal finished with status: {status}')
            self.goal_active = False
            self.last_send_time = time.time()

        send_future.add_done_callback(_goal_response)

    # -------------------- 프런티어 탐색 로직 --------------------

    def tick(self):
        if self.occ is None:
            return
        if self.goal_active or (time.time() - self.last_send_time) < float(self.get_parameter('send_period').value):
            return

        occ = self.occ
        w = occ.info.width
        h = occ.info.height
        res = occ.info.resolution
        ox = occ.info.origin.position.x
        oy = occ.info.origin.position.y
        data = occ.data

        unknown = int(self.get_parameter('unknown').value)
        free = int(self.get_parameter('free').value)
        occupied = int(self.get_parameter('occupied').value)

        rpose = self.robot_pose()
        if rpose is None:
            return
        rx, ry, _ = rpose
        rx_i = int((rx - ox) / res)
        ry_i = int((ry - oy) / res)
        if not (0 <= rx_i < w and 0 <= ry_i < h):
            self.get_logger().warn('Robot is out of map bounds')
            return

        # 1) 프런티어 셀 찾기 (자유 + 8이웃 중 하나 이상 미지)
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

        # 2) 8-이웃 BFS 클러스터링
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

        min_cluster = int(self.get_parameter('min_cluster_size').value)
        clusters = [c for c in clusters if len(c) >= min_cluster]
        if not clusters:
            self.get_logger().info('Only tiny frontier clusters found; waiting for map growth.')
            return

        # 3) 로봇에 가까운 클러스터 우선
        def cell_dist2(c):
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            dx = cx - rx_i
            dy = cy - ry_i
            return dx * dx + dy * dy
        clusters.sort(key=cell_dist2)

        # 4) 프런티어 가장자리에서 안전한 자유셀 선택 + 약간 당겨오기
        target_world = None
        pullback = int(self.get_parameter('pullback_cells').value)

        for cluster in clusters:
            cx = int(round(sum(p[0] for p in cluster) / len(cluster)))
            cy = int(round(sum(p[1] for p in cluster) / len(cluster)))

            found = None
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
                continue

            tx, ty = found

            # unknown 방향의 합 벡터 계산 → 반대 방향으로 pullback
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

            # 벽(occupied) 바로 옆은 피함
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
                continue

            wx = ox + (tx + 0.5) * res
            wy = oy + (ty + 0.5) * res
            target_world = (wx, wy)
            break

        if target_world is None:
            self.get_logger().info('Could not find a safe free target near frontier.')
            return

        self.send_goal(target_world[0], target_world[1])


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 런치에서 이미 shutdown 된 경우를 대비
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
