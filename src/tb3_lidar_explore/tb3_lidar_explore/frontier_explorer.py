#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ---------------------------------------------------------------------------
# frontier_explorer.py
# - LIDAR 기반 프런티어(자유/미지 경계) 탐사 노드
# - Nav2의 NavigateToPose 액션을 사용해 프런티어 근처 자유셀로 이동합니다.
# - 맵(OccupancyGrid)을 구독하여 프런티어를 직접 찾아내고, 안전한 목표점을 계산합니다.
# - TF(map → base_footprint)를 사용해 로봇 현재 위치와 목표 방향(yaw)을 계산합니다.
# - 파라미터로 클러스터 최소 크기, 전송 주기, 프런티어로부터의 당겨오기 거리 등을 조정할 수 있습니다.


# >> 수정사항 : min_cluster_size : 8 >> 15 / 장애물 인식 코드 (프런티어 4단게) 제거! : 복도는 지나갔지만 odom이 뒤틀릴때 map도 같이 틀리고 뒤틀린 장애물을 costmap으로 인식하여 경로가 만들어지는듯??
# 그리고 좁은 지역(복도)에서는 곡선 경로 생성이 어려울 수도 있고 cost area 내부로 프런티어 목표가 생성되지 않도록 하는 코드를 추가할 계획!
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
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, qos
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
        self.costmap : Optional[OccupancyGrid] = None
        self.last_send_time = 0.0                    # 마지막 목표 전송 시각
        self.goal_active = False                     # 현재 Nav2 목표 진행 중 여부
        self.last_goal: Optional[Tuple[float, float]] = None  # 마지막으로 보낸 목표 (디버그용)

        # ---- 파라미터 선언(런치/CLI로 오버라이드 가능)
        #  min_cluster_size : 너무 작은 프런티어 클러스터는 노이즈로 간주하여 무시
        #  send_period      : 목표 전송 최소 간격(초). 너무 잦은 갱신은 불안정/진동 유발
        #  pullback_cells   : 프런티어 경계에서 맵 내부 자유셀 쪽으로 당겨오는 한 칸 수
        #  unknown/free/occupied : 맵 데이터 값 매핑(-1, 0, 100이 일반적이지만 맵 생성기에 따라 다를 수 있음)
        self.declare_parameter('min_cluster_size', 15) # 8
        self.declare_parameter('send_period', 2.0)
        self.declare_parameter('pullback_cells', 2)
        self.declare_parameter('unknown', -1) # -1
        self.declare_parameter('free', 0)
        self.declare_parameter('occupied', 100) # 100
        self.declare_parameter('max_goal_dist_ratio', 0.7) # 맵 길이의 30%를 임계값으로 사용
        self.declare_parameter('step_goal_dist', 1.0)
        self.max_goal_dist_meters: Optional[float] = None # 계산된 실제 거리를 저장할 변수
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

        # 맵이 처음 수신되었을 때 max_goal_dist를 동적으로 계산
        if self.occ is not None and self.max_goal_dist_meters is None:
            map_w_m = self.occ.info.width * self.occ.info.resolution
            map_h_m = self.occ.info.height * self.occ.info.resolution

            # 가로, 세로 중 더 짧은 길이를 기준으로 삼음
            map_length = min(map_w_m, map_h_m)

            ratio = self.get_parameter('max_goal_dist_ratio').value
            self.max_goal_dist_meters = map_length * ratio

            self.get_logger().info(
                f"Map received. Adaptive max_goal_dist set to {self.max_goal_dist_meters:.2f}m "
                f"({map_length:.2f}m * {ratio*100:.0f}%)"
            )
        
    def costmap_callback(self,msg : OccupancyGrid):
        self.costmap = msg
        """costmap OccupancyGrid 메시지를 보관."""

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
        goal = NavigateToPose.Goal() # NavigateToPose: action server에 해당.. explorer가 client에 해당!
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
        #self.get_logger().info(f'Sending goal to ({616.616:.2f}, {616.616:.2f}) yaw={math.degrees(yaw):.1f}°') # source /install/setup.bash 하니깐 수정되는듯?
        self.goal_active = True
        self.last_goal = (wx, wy)
        self.last_send_time = time.time()

        send_future = self.nav_client.send_goal_async(goal) # nav2 에게 goal 좌표던져줌.. 

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
        if self.occ is None or self.costmap is None:
            return
        if self.goal_active or (time.time() - self.last_send_time) < float(self.get_parameter('send_period').value):
            return

        # ---- 맵 메타데이터/데이터 꺼내기
        occ = self.occ # 아마 /map 데이터 일듯...?
        w = occ.info.width
        h = occ.info.height                # 맵의 가로/세로 셀 개수 / 즉, 전체 크기 = width × resolution, height × resolution)
        res = occ.info.resolution          #한 셀의 실제 크기 (예: 0.05 m = 5cm)
        ox = occ.info.origin.position.x
        oy = occ.info.origin.position.y
        data = occ.data  # 1차원 배열: row-major, index = y*w + x
        
        # <<< MODIFIED: Costmap 데이터도 가져옴
        costmap_data = self.costmap.data # 1차원 배열 / width: row vector를 hegiht개수만큼 1차원으로 나열한 데이터..
        w_cost = self.costmap.info.width
        h_cost = self.costmap.info.height
        
        
        # 맵과 코스트맵의 크기/해상도가 다르면 로직이 복잡해지므로 같다고 가정. 다를 경우 경고.
        if w != w_cost or h != h_cost:
            self.get_logger().warn('Map and Costmap dimensions do not match! Target selection may fail.')
            return
        
        
        
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
                if data[idx_row + x] != free: # 이게 slam toolbox의 /map 데이터가 -1, 0, 100으로 이루어져 있어서 이렇게 쓰는거네 애초에...
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
                cx, cy = q.popleft() # 아하 👍 지금 핵심 오해가 딱 여기 있어요. deque의 popleft()는 큐 맨 앞의 원소를 꺼내고 제거합니다.
                cluster.append((cx, cy)) # 그래서 프런티어 추가시 기존의 프런티어 제거! >> 그러다가 마지막 프런티어의 8방향 탐색에도 추가 프런티어 못 찾으면 []되서 종료됨!
                for ny in (cy - 1, cy, cy + 1): # 8방향 탐색! / 그리고 이미 프런티어 군집은 visited에 추가되서 위의 for문 에서 고려되지 않음!
                    for nx in (cx - 1, cx, cx + 1): # 이런 방식으로 여러 개의 프런티어 군집이 만들어진다....
                        if nx == cx and ny == cy:
                            continue
                        if 0 <= nx < w and 0 <= ny < h and (nx, ny) in pts_set and (nx, ny) not in visited:
                            visited.add((nx, ny))
                            q.append((nx, ny)) # 이때 어차피 조건에서 초기의 프런티어 좌표는 걸리지네! visited때문에!
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
        def cell_dist2(c): # c : 각 cluster를 의미.. [(1,1)(1,2)(1,3)....] 으로 구성될 것.. / p가 각 프런티어 좌표를 의미...
            cx = sum(p[0] for p in c) / len(c)  # rx_i : map 좌표계 기준으로한 로봇의 상대위치...
            cy = sum(p[1] for p in c) / len(c)
            dx = cx - rx_i  
            dy = cy - ry_i
            return dx * dx + dy * dy # 프런티어 군집의 평균 중심점과 로봇사이의 거리...

        clusters.sort(key=cell_dist2) # 그러면 아마도 거리를 기준으로 클러스터를 정렬한 모양인듯?

        # ------------------------------------------------------------------
        # 4) 프런티어 가장자리에서 "안전한 자유셀" 선택 + 프런티어 반대 방향으로 약간 당겨오기
        #  - pullback: unknown 쪽이 아닌, 맵 내부 자유영역 쪽으로 몇 칸 이동해 목표 안정화
        #  - 또한 8이웃에 occupied(벽)가 붙어있는 셀은 회피(충돌 여유 확보)
        #  - 안전한 자유셀 선택 + 당겨오기 (Costmap 조건 추가)
        # ------------------------------------------------------------------
        target_world = None
        pullback = int(self.get_parameter('pullback_cells').value)

        for cluster in clusters:
            # 클러스터 중심(정수 index) 근방에서 자유셀을 작은 반경부터 탐색
            cx = int(round(sum(p[0] for p in cluster) / len(cluster)))
            cy = int(round(sum(p[1] for p in cluster) / len(cluster)))

            found = None
            # r: 확장 반경(마름모/사각 둘레 샘플링으로 간략 탐색)
            for r in range(0, 6): # 프런티어 군집들의 중심점이 free인지를 판단하는 단계..
                for dx in range(-r, r + 1):
                    for dy in (-r, r): # 이거 조심할게 range가 아닌 튜플 ()이라 내부 요소만큼 반복됨! 그래서 처음에 0 두 번 반복됨!
                        nx, ny = cx + dx, cy + dy
                        if 0 <= nx < w and 0 <= ny < h and data[ny * w + nx] == free and costmap_data[ny*w + nx] == 0 :
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
            for ny in (ty - 1, ty, ty + 1): ## 8방향 탐색/ 아까 찾은 free인 프런티어 중심점을 기준으로...
                for nx in (tx - 1, tx, tx + 1):
                    if nx == tx and ny == ty:
                        continue
                    if 0 <= nx < w and 0 <= ny < h and data[ny * w + nx] == unknown:
                        ux += (nx - tx)
                        uy += (ny - ty)
            if ux != 0 or uy != 0:
                mag = math.hypot(ux, uy) # 벡텈 크기!
                if mag > 0.0:
                    # 단위벡터로 만들고, 자유셀을 따라 pullback_steps 만큼 이동
                    vx, vy = -ux / mag, -uy / mag
                    px, py = float(tx), float(ty) # <<< MODIFIED: 부동소수점 연산을 위해 float으로 변환
                    steps = pullback
                    while steps > 0:
                        nx_f = px + vx
                        ny_f = py + vy
                        nx, ny = int(round(nx_f)), int(round(ny_f)) # <<< MODIFIED: 가장 가까운 정수 인덱스로 변환
                        # <<< MODIFIED: Costmap 비용이 0인지 확인하는 조건 추가
                        if 0 <= nx < w and 0 <= ny < h and data[ny * w + nx] == free and costmap_data[ny * w + nx] == 0:
                            px, py = nx_f, ny_f # px, py를 부동소수점(nx_f, ny_f)으로 업데이트하면, 벡터 방향을 따라 더 부드럽고 정확하게 이동한 위치를 누적할 수 있습니다
                            steps -= 1
                        else:
                            break
                    tx, ty = int(round(px)), int(round(py)) # <<< MODIFIED: 최종 위치를 정수 인덱스로 확정

            # --- 벽(occupied) 바로 인접한 자유셀은 회피(8이웃 중 점유가 있으면 제외)
            # close_to_wall = False
            # for ny in (ty - 1, ty, ty + 1):
            #     for nx in (tx - 1, tx, tx + 1): # 다시 위에서 계산한 tx,ty기준으로 8방향 탐색..
            #         if nx == tx and ny == ty:
            #             continue
            #         if 0 <= nx < w and 0 <= ny < h and data[ny * w + nx] >= occupied: # 하나라도 벽 존재하면.. 이건 버림...새로운 nx,ny로!
            #             close_to_wall = True
            #             break
            #     if close_to_wall:
            #         break
            # if close_to_wall:
            #     # 이 후보는 버리고 다음 클러스터에서 다시 시도 / 이게 클러스터 for문에 걸리는 거임! 벽 존재하면 버리고 다음 군집에서 시도...
            #     continue

            # --- index → world 좌표 변환(셀 중심으로 살짝 오프셋 +0.5)
            wx = ox + (tx + 0.5) * res
            wy = oy + (ty + 0.5) * res
            target_world = (wx, wy)  # 근데 이렇게 되어 버리면 가장 가까운 클러스터 하나만 고려하는 건가?? 
            break

        if target_world is None:
            self.get_logger().info('Could not find a safe free target near frontier.')
            return
        
        # <<< MODIFIED: 중간 목표점 생성 로직
        # 맵이 수신되지 않아 아직 거리가 계산되지 않았다면 로직을 건너뜀
        if self.max_goal_dist_meters is None:
            return

        final_wx, final_wy = target_world
        dist_to_goal = math.hypot(final_wx - rx, final_wy - ry)
        max_dist = self.max_goal_dist_meters # 계산된 동적 임계값 사용
        goal_wx, goal_wy = final_wx, final_wy
        
        # if dist_to_goal > max_dist:
        #     step_dist = self.get_parameter('step_goal_dist').value
        #     # 로봇 위치에서 최종 목표 방향으로 step_dist 만큼 떨어진 지점 계산
        #     ratio = step_dist / dist_to_goal
        #     goal_wx = rx + (final_wx - rx) * ratio
        #     goal_wy = ry + (final_wy - ry) * ratio
        #     self.get_logger().info(f"Goal is too far ({dist_to_goal:.2f}m). Sending intermediate goal.")
            
            
        
        
        # 계산한 안전 목표로 이동
        self.send_goal(goal_wx, goal_wy) # nav2 : action server에 goal 좌표던져줌!
        # 핵심은 프런티어 군집을 잘 찾아서 다음 목표지점을 던져주는 거니깐....


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
