import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

from robot_info_msg.msg import RobotInfo
from std_msgs.msg import String as StdString
from std_msgs.msg import Bool as StdBool
from std_msgs.msg import Float64 as StdFloat64


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_xy_list(tokens: List[str]) -> List[Tuple[float, float]]:
    out = []
    for t in tokens:
        xs, ys = t.split(',')
        out.append((float(xs), float(ys)))
    return out


def parse_offsets(s: str) -> List[Tuple[float, float]]:
    # "dx,dy;dx,dy;dx,dy"
    items = [it.strip() for it in s.split(';') if it.strip()]
    out = []
    for it in items:
        dx, dy = it.split(',')
        out.append((float(dx), float(dy)))
    return out


class TriangleFormationController(Node):
    def __init__(self):
        super().__init__('triangle_formation_controller')

        # ---- params ----
        self.declare_parameter('robot_names', ['wamv1', 'wamv2', 'wamv3'])
        self.declare_parameter('robot_goals', ['0,0', '0,0', '0,0'])
        self.declare_parameter('formation_offsets', '0,0;-6,4;-6,-4')

        self.declare_parameter('leader_speed', 2.0)
        self.declare_parameter('lookahead', 6.0)
        self.declare_parameter('goal_radius', 2.0)

        self.declare_parameter('k_v_to_thrust', 2.0)
        self.declare_parameter('k_w_to_diffthrust', 3.0)
        self.declare_parameter('thrust_limit', 12.0)

        self.declare_parameter('yaw_kp', 1.8)
        self.declare_parameter('pos_kp', 0.6)

        self.robot_names: List[str] = list(self.get_parameter('robot_names').value)
        goal_tokens: List[str] = list(self.get_parameter('robot_goals').value)
        self.robot_goals_xy: List[Tuple[float, float]] = parse_xy_list(goal_tokens)
        self.offsets: List[Tuple[float, float]] = parse_offsets(
            self.get_parameter('formation_offsets').value
        )

        assert len(self.robot_names) == 3, "This demo assumes exactly 3 robots."
        assert len(self.robot_goals_xy) == 3, "robot_goals must have 3 items."
        assert len(self.offsets) == 3, "formation_offsets must have 3 items."

        self.leader = self.robot_names[0]
        self.leader_speed = float(self.get_parameter('leader_speed').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.goal_radius = float(self.get_parameter('goal_radius').value)

        self.k_v_to_thrust = float(self.get_parameter('k_v_to_thrust').value)
        self.k_w_to_diff = float(self.get_parameter('k_w_to_diffthrust').value)
        self.thrust_lim = float(self.get_parameter('thrust_limit').value)

        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.pos_kp = float(self.get_parameter('pos_kp').value)

        # ---- state ----
        self.odom: Dict[str, Odometry] = {}
        self.start_inited = False
        self.t0 = None  # sim time
        self.leader_start_xy = None  # (x,y) world
        self.leader_goal_xy = self.robot_goals_xy[0]  # world

        # ---- pubs/subs ----
        self.pub_left: Dict[str, any] = {}
        self.pub_right: Dict[str, any] = {}
        self.pub_info: Dict[str, any] = {}

        for name in self.robot_names:
            self.create_subscription(
                Odometry,
                f'/{name}/sensors/position/ground_truth_odometry',
                lambda msg, n=name: self._on_odom(n, msg),
                10
            )
            self.pub_left[name] = self.create_publisher(Float64, f'/{name}/thrusters/left/thrust', 10)
            self.pub_right[name] = self.create_publisher(Float64, f'/{name}/thrusters/right/thrust', 10)
            self.pub_info[name] = self.create_publisher(RobotInfo, f'/{name}/robot_info', 10)

        # 20 Hz control loop
        self.create_timer(0.05, self._step)

        self.get_logger().info(
            f'FORMATION demo started. robots={self.robot_names}, goals={self.robot_goals_xy}, offsets={self.offsets}'
        )

    def _on_odom(self, name: str, msg: Odometry):
        self.odom[name] = msg

    def _get_pose2d(self, name: str) -> Tuple[float, float, float]:
        o = self.odom[name]
        x = o.pose.pose.position.x
        y = o.pose.pose.position.y
        q = o.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return x, y, yaw

    def _distance_to_goal(self, name: str) -> float:
        x, y, _ = self._get_pose2d(name)
        gx, gy = self.robot_goals_xy[self.robot_names.index(name)]
        return math.hypot(x - gx, y - gy)

    def _publish_thrust(self, name: str, left: float, right: float):
        left = max(-self.thrust_lim, min(self.thrust_lim, left))
        right = max(-self.thrust_lim, min(self.thrust_lim, right))
        ml = Float64()
        mr = Float64()
        ml.data = float(left)
        mr.data = float(right)
        self.pub_left[name].publish(ml)
        self.pub_right[name].publish(mr)

    def _publish_robot_info(self, name: str):
        # publish RobotInfo so experiment_manager can stop :contentReference[oaicite:8]{index=8}
        now = self.get_clock().now().to_msg()
        msg = RobotInfo()
        msg.header.stamp = now
        msg.robot_name = StdString(data=name)
        msg.pose = self.odom[name].pose.pose
        msg.velocity = self.odom[name].twist.twist

        if self.t0 is None:
            travel = 0.0
        else:
            dt = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
            travel = max(0.0, dt)

        msg.travel_time = StdFloat64(data=float(travel))
        reached = self._distance_to_goal(name) <= self.goal_radius
        msg.reach_goal = StdBool(data=bool(reached))
        self.pub_info[name].publish(msg)

    def _leader_los(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        # LOS straight line from leader_start -> leader_goal
        x0, y0 = self.leader_start_xy
        x1, y1 = self.leader_goal_xy
        dx = x1 - x0
        dy = y1 - y0
        L = math.hypot(dx, dy) + 1e-9
        ux = dx / L
        uy = dy / L

        # projection
        vx = x - x0
        vy = y - y0
        s = vx * ux + vy * uy
        s = max(0.0, min(L, s))
        px = x0 + s * ux
        py = y0 + s * uy

        # lookahead point
        s_la = min(L, s + self.lookahead)
        x_la = x0 + s_la * ux
        y_la = y0 + s_la * uy

        psi_ref = math.atan2(y_la - y, x_la - x)
        epsi = wrap_pi(psi_ref - yaw)

        omega = self.yaw_kp * epsi
        # speed reduce near goal
        dist_goal = math.hypot(x - x1, y - y1)
        v = min(self.leader_speed, 0.5 + 0.1 * dist_goal)
        return v, omega

    def _follower_track_point(self, name: str, target_xy: Tuple[float, float]) -> Tuple[float, float]:
        x, y, yaw = self._get_pose2d(name)
        tx, ty = target_xy
        psi_ref = math.atan2(ty - y, tx - x)
        epsi = wrap_pi(psi_ref - yaw)
        dist = math.hypot(tx - x, ty - y)

        omega = self.yaw_kp * epsi
        v = min(self.leader_speed, self.pos_kp * dist)
        return v, omega

    def _vw_to_thrust(self, v: float, omega: float) -> Tuple[float, float]:
        base = self.k_v_to_thrust * v
        diff = self.k_w_to_diff * omega
        left = base - diff
        right = base + diff
        return left, right

    def _step(self):
        # need all odom ready
        if any(n not in self.odom for n in self.robot_names):
            return

        # init start
        if not self.start_inited:
            self.t0 = self.get_clock().now()
            lx, ly, _ = self._get_pose2d(self.leader)
            self.leader_start_xy = (lx, ly)
            self.start_inited = True
            self.get_logger().info(f'Init leader start = {self.leader_start_xy}, leader goal = {self.leader_goal_xy}')

        # leader control
        lx, ly, lyaw = self._get_pose2d(self.leader)
        vL, wL = self._leader_los(lx, ly, lyaw)
        lth, rth = self._vw_to_thrust(vL, wL)
        self._publish_thrust(self.leader, lth, rth)

        # formation targets (world)
        cy = math.cos(lyaw)
        sy = math.sin(lyaw)

        for i, name in enumerate(self.robot_names[1:], start=1):
            ox, oy = self.offsets[i]
            # rotate offset by leader yaw: world = leader + R * offset
            tx = lx + cy * ox - sy * oy
            ty = ly + sy * ox + cy * oy

            vF, wF = self._follower_track_point(name, (tx, ty))
            lth, rth = self._vw_to_thrust(vF, wF)
            self._publish_thrust(name, lth, rth)

        # publish robot_info for experiment manager
        for name in self.robot_names:
            self._publish_robot_info(name)

        # if all reached, stop
        if all(self._distance_to_goal(n) <= self.goal_radius for n in self.robot_names):
            for n in self.robot_names:
                self._publish_thrust(n, 0.0, 0.0)

def main():
    rclpy.init()
    node = TriangleFormationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()