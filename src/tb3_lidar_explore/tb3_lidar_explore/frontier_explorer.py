import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import tf2_ros

def yaw_to_quat(yaw):
    q = Quaternion()
    q.w = math.cos(yaw/2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw/2.0)
    return q

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('min_frontier_size', 12)
        self.declare_parameter('attempt_timeout_sec', 45.0)
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.min_frontier = int(self.get_parameter('min_frontier_size').value)
        self.attempt_timeout = float(self.get_parameter('attempt_timeout_sec').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.map = None
        self.goal_in_progress = False
        self.last_goal_time = self.get_clock().now()

        self.timer = self.create_timer(2.0, self.tick)
        self.get_logger().info('frontier_explorer started.')

    def on_map(self, msg: OccupancyGrid):
        self.map = msg

    def grid_at(self, data, w, x, y):
        return data[y * w + x]

    def find_frontiers(self, grid: OccupancyGrid):
        w = grid.info.width
        h = grid.info.height
        data = grid.data
        frontiers = []
        for y in range(1, h-1):
            for x in range(1, w-1):
                v = self.grid_at(data, w, x, y)
                if v != 0:  # free space만 기준
                    continue
                # 4-neigh 중 unknown(-1) 하나라도 있으면 frontier
                if (self.grid_at(data, w, x+1, y) == -1 or
                    self.grid_at(data, w, x-1, y) == -1 or
                    self.grid_at(data, w, x, y+1) == -1 or
                    self.grid_at(data, w, x, y-1) == -1):
                    frontiers.append((x, y))
        return frontiers

    def cluster_frontiers(self, pts):
        # BFS로 간단 클러스터
        pts_set = set(pts)
        clusters = []
        while pts_set:
            seed = pts_set.pop()
            stack = [seed]
            cluster = [seed]
            while stack:
                cx, cy = stack.pop()
                for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                    nx, ny = cx+dx, cy+dy
                    if (nx,ny) in pts_set:
                        pts_set.remove((nx,ny))
                        stack.append((nx,ny))
                        cluster.append((nx,ny))
            if len(cluster) >= self.min_frontier:
                clusters.append(cluster)
        return clusters

    def grid_to_world(self, grid: OccupancyGrid, x, y):
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        wx = ox + (x + 0.5) * res
        wy = oy + (y + 0.5) * res
        return wx, wy

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.base_frame, rclpy.time.Time(), Duration(seconds=0.2))
            t = tf.transform.translation
            # yaw from quaternion
            q = tf.transform.rotation
            siny_cosp = 2*(q.w*q.z + q.x*q.y)
            cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (t.x, t.y, yaw)
        except Exception as e:
            self.get_logger().warn(f'robot_pose lookup failed: {e}')
            return None

    def pick_goal(self, clusters, grid: OccupancyGrid):
        pose = self.robot_pose()
        if pose is None:
            return None
        rx, ry, _ = pose
        res = grid.info.resolution

        def is_free(ix, iy):
            w = grid.info.width
            h = grid.info.height
            if ix < 0 or iy < 0 or ix >= w or iy >= h:
                return False
            v = grid.data[iy * w + ix]
            return v == 0  # free only

        best = None
        best_d = 1e9
        for c in clusters:
            cx = int(sum(p[0] for p in c) / len(c))
            cy = int(sum(p[1] for p in c) / len(c))

            # 로봇 쪽으로 N셀(예: 3셀) backoff
            dx = rx - (grid.info.origin.position.x + (cx + 0.5) * res)
            dy = ry - (grid.info.origin.position.y + (cy + 0.5) * res)
            L = math.hypot(dx, dy) + 1e-9
            ux, uy = dx / L, dy / L

            bx, by = cx, cy
            for _ in range(3):  # 3 cells back
                bx = int(round(bx + ux))   # grid 한 칸 로봇 쪽
                by = int(round(by + uy))
                if is_free(bx, by):
                    wx, wy = self.grid_to_world(grid, bx, by)
                    d = math.hypot(wx - rx, wy - ry)
                    if d < best_d:
                        best_d = d
                        best = (wx, wy)
                    break
        return best

    def send_goal(self, wx, wy):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available yet.')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation = yaw_to_quat(0.0)

        self.goal_in_progress = True
        self.last_goal_time = self.get_clock().now()

        self.get_logger().info(f'Sending goal to ({wx:.2f}, {wy:.2f})')
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_cb)
        return True

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.goal_in_progress = False
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        self.get_logger().info(f'Goal finished with status: {status}')
        self.goal_in_progress = False

    def tick(self):
        # 목표 수행 중인데 오래 걸리면 취소(단순 타임아웃)
        if self.goal_in_progress:
            if (self.get_clock().now() - self.last_goal_time).nanoseconds * 1e-9 > self.attempt_timeout:
                self.get_logger().warn('Attempt timeout; will look for a new frontier next tick.')
                self.goal_in_progress = False
            return

        if self.map is None:
            return

        f = self.find_frontiers(self.map)
        if not f:
            self.get_logger().info('No more frontiers. Exploration may be complete.')
            return
        clusters = self.cluster_frontiers(f)
        if not clusters:
            self.get_logger().info('Frontiers present but too small; waiting for expansion.')
            return
        goal = self.pick_goal(clusters, self.map)
        if goal:
            self.send_goal(*goal)

def main():
    rclpy.init()
    n = FrontierExplorer()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
