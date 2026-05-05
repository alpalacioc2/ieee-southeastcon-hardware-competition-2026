#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from launch_drone_pkg import drone_control


class LaunchDroneNode(Node):
    def __init__(self):
        super().__init__('launch_drone_node')

        self.declare_parameter('command_topic', '/drone_command')
        self.declare_parameter('flight_time', 2)

        self.command_topic = self.get_parameter('command_topic').value
        self.flight_time = float(self.get_parameter('flight_time').value)

        self.busy = False

        self.subscription = self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            10
        )

        self.get_logger().info('launch_drone_node started.')
        self.get_logger().info(f'Listening on topic: {self.command_topic}')
        self.get_logger().info(f'Configured flight_time: {self.flight_time:.1f} s')

    def command_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'Received ROS command: "{cmd}"')

        if cmd != 'LAUNCH_DRONE':
            self.get_logger().warn(f'Ignoring unknown command: "{cmd}"')
            return

        if self.busy:
            self.get_logger().warn('Drone launch already in progress. Ignoring duplicate command.')
            return

        self.busy = True
        worker = threading.Thread(target=self._run_launch_sequence, daemon=True)
        worker.start()

    def _run_launch_sequence(self):
        try:
            self.get_logger().info('Starting drone takeoff/land sequence...')
            drone_control.takeoff_and_land(flight_time=self.flight_time)
            self.get_logger().info('Drone sequence completed successfully.')
        except Exception as e:
            self.get_logger().error(f'Drone sequence failed: {e}')
        finally:
            self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = LaunchDroneNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
