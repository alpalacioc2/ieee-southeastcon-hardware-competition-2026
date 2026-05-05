#!/usr/bin/env python3

import math
from statistics import median

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


class LidarTwelveSensors(Node):
    def __init__(self):
        super().__init__('lidar_twelve_sensors')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('sensor_topic', '/sensors')
        self.declare_parameter('marker_topic', '/sensor_markers')
        self.declare_parameter('debug_scan_topic', '/scan_debug')
        self.declare_parameter('frame_id', 'laser')

        # Beam width
        self.declare_parameter('half_width_deg', 5.0)

        # Valid ranges
        self.declare_parameter('min_valid_range', 0.10)
        self.declare_parameter('max_valid_range', 4.00)

        # Arrow fallback length if invalid
        self.declare_parameter('default_marker_length', 0.30)

        # ---------------- Beam centers in degrees ----------------
        # Adjust these if RViz shows the wedges slightly off.

        # Left wall
        self.declare_parameter('left_front_deg', -110.0)
        self.declare_parameter('left_mid_deg', -90.0)
        self.declare_parameter('left_back_deg', -70.0)

        # Right wall
        self.declare_parameter('right_front_deg', 110.0)
        self.declare_parameter('right_mid_deg', 90.0)
        self.declare_parameter('right_back_deg', 70.0)

        # Front wall
        self.declare_parameter('front_left_deg', -160.0)
        self.declare_parameter('front_mid_deg', 180.0)
        self.declare_parameter('front_right_deg', 160.0)

        # Back wall
        self.declare_parameter('back_left_deg', -20.0)
        self.declare_parameter('back_mid_deg', 0.0)
        self.declare_parameter('back_right_deg', 20.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.sensor_topic = self.get_parameter('sensor_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.debug_scan_topic = self.get_parameter('debug_scan_topic').value

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.sensor_pub = self.create_publisher(Float32MultiArray, self.sensor_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.debug_scan_pub = self.create_publisher(LaserScan, self.debug_scan_topic, 10)

        self.get_logger().info('lidar_twelve_sensors node started.')
        self.get_logger().info(f'Subscribing to: {self.scan_topic}')
        self.get_logger().info(f'Publishing sensor data to: {self.sensor_topic}')
        self.get_logger().info(f'Publishing markers to: {self.marker_topic}')
        self.get_logger().info(f'Publishing debug scan to: {self.debug_scan_topic}')

    def angle_to_index(self, angle_rad: float, angle_min: float, angle_inc: float, n: int):
        i = int(round((angle_rad - angle_min) / angle_inc))
        return max(0, min(i, n - 1))

    def get_window_indices(self, scan: LaserScan, center_deg: float, half_width_deg: float):
        n = len(scan.ranges)
        center_rad = math.radians(center_deg)
        half_rad = math.radians(half_width_deg)

        i0 = self.angle_to_index(center_rad - half_rad, scan.angle_min, scan.angle_increment, n)
        i1 = self.angle_to_index(center_rad + half_rad, scan.angle_min, scan.angle_increment, n)

        if i1 < i0:
            i0, i1 = i1, i0

        return i0, i1

    def get_window_median(self, scan: LaserScan, i0: int, i1: int, min_valid: float, max_valid: float):
        vals = []
        for r in scan.ranges[i0:i1 + 1]:
            if math.isfinite(r) and (min_valid <= r <= max_valid):
                vals.append(r)

        if not vals:
            return None

        return float(median(vals))

    def make_arrow_marker(self, marker_id: int, frame_id: str, angle_deg: float,
                          length: float, color, ns: str):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        yaw = math.radians(angle_deg)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        marker.scale.x = max(length, 0.02)
        marker.scale.y = 0.02
        marker.scale.z = 0.04

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        marker.lifetime = Duration(sec=0, nanosec=250000000)
        return marker

    def build_debug_scan(self, scan: LaserScan, windows):
        debug_scan = LaserScan()
        debug_scan.header = scan.header
        debug_scan.angle_min = scan.angle_min
        debug_scan.angle_max = scan.angle_max
        debug_scan.angle_increment = scan.angle_increment
        debug_scan.time_increment = scan.time_increment
        debug_scan.scan_time = scan.scan_time
        debug_scan.range_min = scan.range_min
        debug_scan.range_max = scan.range_max

        n = len(scan.ranges)
        debug_ranges = [float('inf')] * n

        for (i0, i1) in windows:
            for i in range(i0, i1 + 1):
                debug_ranges[i] = scan.ranges[i]

        debug_scan.ranges = debug_ranges

        if len(scan.intensities) == n:
            debug_intensities = [0.0] * n
            for (i0, i1) in windows:
                for i in range(i0, i1 + 1):
                    debug_intensities[i] = scan.intensities[i]
            debug_scan.intensities = debug_intensities

        return debug_scan

    def scan_callback(self, scan: LaserScan):
        half_width_deg = float(self.get_parameter('half_width_deg').value)
        min_valid = float(self.get_parameter('min_valid_range').value)
        max_valid = float(self.get_parameter('max_valid_range').value)
        default_len = float(self.get_parameter('default_marker_length').value)
        frame_id = str(self.get_parameter('frame_id').value)

        beam_defs = {
            'left_front':  float(self.get_parameter('left_front_deg').value),
            'left_mid':    float(self.get_parameter('left_mid_deg').value),
            'left_back':   float(self.get_parameter('left_back_deg').value),

            'right_front': float(self.get_parameter('right_front_deg').value),
            'right_mid':   float(self.get_parameter('right_mid_deg').value),
            'right_back':  float(self.get_parameter('right_back_deg').value),

            'front_left':  float(self.get_parameter('front_left_deg').value),
            'front_mid':   float(self.get_parameter('front_mid_deg').value),
            'front_right': float(self.get_parameter('front_right_deg').value),

            'back_left':   float(self.get_parameter('back_left_deg').value),
            'back_mid':    float(self.get_parameter('back_mid_deg').value),
            'back_right':  float(self.get_parameter('back_right_deg').value),
        }

        sensor_order = [
            'left_front', 'left_mid', 'left_back',
            'right_front', 'right_mid', 'right_back',
            'front_left', 'front_mid', 'front_right',
            'back_left', 'back_mid', 'back_right',
        ]

        # Colors by group:
        # left = blue, right = red, front = green, back = yellow
        color_map = {
            'left_front':  (0.0, 0.4, 1.0),
            'left_mid':    (0.0, 0.4, 1.0),
            'left_back':   (0.0, 0.4, 1.0),

            'right_front': (1.0, 0.1, 0.1),
            'right_mid':   (1.0, 0.1, 0.1),
            'right_back':  (1.0, 0.1, 0.1),

            'front_left':  (0.1, 1.0, 0.1),
            'front_mid':   (0.1, 1.0, 0.1),
            'front_right': (0.1, 1.0, 0.1),

            'back_left':   (1.0, 1.0, 0.1),
            'back_mid':    (1.0, 1.0, 0.1),
            'back_right':  (1.0, 1.0, 0.1),
        }

        windows = {}
        values = {}

        for name in sensor_order:
            deg = beam_defs[name]
            i0, i1 = self.get_window_indices(scan, deg, half_width_deg)
            windows[name] = (i0, i1)
            values[name] = self.get_window_median(scan, i0, i1, min_valid, max_valid)

        # Publish /sensors
        # Order:
        # 0 left_front
        # 1 left_mid
        # 2 left_back
        # 3 right_front
        # 4 right_mid
        # 5 right_back
        # 6 front_left
        # 7 front_mid
        # 8 front_right
        # 9 back_left
        # 10 back_mid
        # 11 back_right
        sensor_msg = Float32MultiArray()
        sensor_msg.data = [
            float(values[name]) if values[name] is not None else -1.0
            for name in sensor_order
        ]
        self.sensor_pub.publish(sensor_msg)

        # Publish debug scan
        debug_scan = self.build_debug_scan(scan, list(windows.values()))
        self.debug_scan_pub.publish(debug_scan)

        # Publish markers
        markers = MarkerArray()
        for idx, name in enumerate(sensor_order, start=1):
            deg = beam_defs[name]
            val = values[name] if values[name] is not None else default_len
            markers.markers.append(
                self.make_arrow_marker(
                    idx,
                    frame_id,
                    deg,
                    val,
                    color_map[name],
                    'virtual_beams'
                )
            )
        self.marker_pub.publish(markers)

        self.get_logger().info(
            ' | '.join([
                f'LF={sensor_msg.data[0]:.3f}',
                f'LM={sensor_msg.data[1]:.3f}',
                f'LB={sensor_msg.data[2]:.3f}',
                f'RF={sensor_msg.data[3]:.3f}',
                f'RM={sensor_msg.data[4]:.3f}',
                f'RB={sensor_msg.data[5]:.3f}',
                f'FL={sensor_msg.data[6]:.3f}',
                f'FM={sensor_msg.data[7]:.3f}',
                f'FR={sensor_msg.data[8]:.3f}',
                f'BL={sensor_msg.data[9]:.3f}',
                f'BM={sensor_msg.data[10]:.3f}',
                f'BR={sensor_msg.data[11]:.3f}',
            ]),
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarTwelveSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
