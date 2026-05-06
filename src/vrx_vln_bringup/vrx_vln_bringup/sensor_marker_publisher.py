#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from visualization_msgs.msg import Marker, MarkerArray


class SensorMarkerPublisher(Node):
    def __init__(self):
        super().__init__('sensor_marker_publisher')

        self.declare_parameter('robot_name', 'wamv')
        self.declare_parameter('base_frame', 'wamv/wamv/base_link')

        self.robot_name = self.get_parameter('robot_name').value
        self.base_frame = self.get_parameter('base_frame').value

        self.latest_imu = None
        self.latest_gps = None
        self.latest_odom = None

        self.create_subscription(Imu, 'sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(NavSatFix, 'sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(
            Odometry,
            'sensors/position/ground_truth_odometry',
            self.odom_callback,
            10,
        )

        self.marker_pub = self.create_publisher(MarkerArray, 'sensor_debug/markers', 10)
        self.create_timer(0.2, self.publish_markers)

    def imu_callback(self, msg):
        self.latest_imu = msg

    def gps_callback(self, msg):
        self.latest_gps = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_array.markers.append(
            self.make_text_marker(0, self.gps_text(), 2.2, (0.2, 0.9, 1.0))
        )
        marker_array.markers.append(
            self.make_text_marker(1, self.imu_text(), 2.9, (1.0, 0.8, 0.2))
        )
        marker_array.markers.append(
            self.make_text_marker(2, self.odom_text(), 3.6, (0.3, 1.0, 0.3))
        )
        self.marker_pub.publish(marker_array)

    def make_text_marker(self, marker_id, text, z_offset, color):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.base_frame
        marker.ns = 'sensor_debug'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = z_offset
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.38
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.text = text
        return marker

    def gps_text(self):
        if self.latest_gps is None:
            return 'GPS\nwaiting for /sensors/gps/gps/fix'

        return (
            'GPS\n'
            f'lat: {self.latest_gps.latitude:.6f}\n'
            f'lon: {self.latest_gps.longitude:.6f}\n'
            f'alt: {self.latest_gps.altitude:.2f} m'
        )

    def imu_text(self):
        if self.latest_imu is None:
            return 'IMU\nwaiting for /sensors/imu/imu/data'

        return (
            'IMU\n'
            f'ang vel: {self.latest_imu.angular_velocity.x:.2f}, '
            f'{self.latest_imu.angular_velocity.y:.2f}, '
            f'{self.latest_imu.angular_velocity.z:.2f}\n'
            f'lin acc: {self.latest_imu.linear_acceleration.x:.2f}, '
            f'{self.latest_imu.linear_acceleration.y:.2f}, '
            f'{self.latest_imu.linear_acceleration.z:.2f}'
        )

    def odom_text(self):
        if self.latest_odom is None:
            return 'ODOM\nwaiting for /sensors/position/ground_truth_odometry'

        pose = self.latest_odom.pose.pose
        twist = self.latest_odom.twist.twist
        yaw_deg = self.quaternion_to_yaw_degrees(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        speed = math.hypot(twist.linear.x, twist.linear.y)

        return (
            'ODOM\n'
            f'x: {pose.position.x:.2f}  y: {pose.position.y:.2f}\n'
            f'z: {pose.position.z:.2f}  yaw: {yaw_deg:.1f} deg\n'
            f'vx: {twist.linear.x:.2f}  vy: {twist.linear.y:.2f}  speed: {speed:.2f}'
        )

    @staticmethod
    def quaternion_to_yaw_degrees(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def main(args=None):
    rclpy.init(args=args)
    node = SensorMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
