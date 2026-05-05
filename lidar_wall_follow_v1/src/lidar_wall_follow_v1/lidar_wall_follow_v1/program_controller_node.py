#!/usr/bin/env python3
"""
program_controller_node.py
==========================

Publishes commands to /wall_cmd for wall_follower_node and monitors /sensors
to advance through a programmed sequence.

Supported step types:
  1) Timed wall follow
  2) Timed stop
  3) Sensor-based wall follow until a chosen sensor reaches a target value
  4) Timed raw wheel command
  5) Timed task command
  6) Capture most recent detected color into a 4-element array
  7) Display the 4-element color array on the Arduino LCD
  8) Publish the 4-element color array to /ir_colors for the IR transmitter node
  9) Wait until "ARDUINO READY" is received from the Arduino, with timeout
  10) Publish a launch command to the drone node

Commands sent to wall_follower_node:
    "left_forward <distance_m>"
    "left_reverse <distance_m>"
    "stop"
    "raw <left_vel> <right_vel>"
    "task <task_name>"
    "display <c1>,<c2>,<c3>,<c4>"

Valid task names:
    Flag
    Crank
    Push

You may specify the sensor in a step by either:
    'sensor_name': 'back_mid'
or
    'sensor_index': 10

Color capture:
    The node subscribes to /image_colors and stores the most recent valid color.
    Valid colors:
        RED
        PURPLE
        GREEN
        BLUE

    To save the latest detected color into the color array:
        {
            'type': 'capture_color',
            'array_index': 0,
        }

    The array is:
        self.detected_colors = ["", "", "", ""]

Display colors:
    To display the currently stored array on the Arduino LCD:
        {
            'type': 'display_colors',
            'duration': 1.0,
        }

    Or display an explicitly provided array:
        {
            'type': 'display_colors',
            'colors': ['', 'RED', '', 'BLUE'],
            'duration': 1.0,
        }

Publish IR colors:
    To publish the currently stored array to /ir_colors:
        {
            'type': 'publish_ir_colors',
            'duration': 1.0,
        }

    Or publish an explicitly provided array:
        {
            'type': 'publish_ir_colors',
            'colors': ['', 'RED', '', 'BLUE'],
            'duration': 1.0,
        }

Wait for Arduino ready:
    To wait until the Arduino sends "ARDUINO READY":
        {
            'type': 'wait_for_arduino_ready',
            'timeout': 30.0,
        }

Launch drone:
    To command the drone node to take off and land:
        {
            'type': 'launch_drone',
            'duration': 3.0,
        }
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray


class ProgramControllerNode(Node):
    def __init__(self):
        super().__init__('program_controller_node')

        # ---------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------
        self.declare_parameter('cmd_topic', '/wall_cmd')
        self.declare_parameter('sensor_topic', '/sensors')
        self.declare_parameter('color_topic', '/image_colors')
        self.declare_parameter('ir_topic', '/ir_colors')
        self.declare_parameter('ready_topic', '/arduino_ready')
        self.declare_parameter('drone_topic', '/drone_command')
        self.declare_parameter('auto_start', True)

        cmd_topic    = self.get_parameter('cmd_topic').value
        sensor_topic = self.get_parameter('sensor_topic').value
        color_topic  = self.get_parameter('color_topic').value
        ir_topic     = self.get_parameter('ir_topic').value
        ready_topic  = self.get_parameter('ready_topic').value
        drone_topic  = self.get_parameter('drone_topic').value
        auto_start   = bool(self.get_parameter('auto_start').value)

        # ---------------------------------------------------------------
        # Sensor name <-> index mapping from lidar_twelve_sensors.py
        # ---------------------------------------------------------------
        self.SENSOR_INDEX = {
            'left_front':  0,
            'left_mid':    1,
            'left_back':   2,
            'right_front': 3,
            'right_mid':   4,
            'right_back':  5,
            'front_left':  6,
            'front_mid':   7,
            'front_right': 8,
            'back_left':   9,
            'back_mid':    10,
            'back_right':  11,
        }

        self.SENSOR_NAME = {v: k for k, v in self.SENSOR_INDEX.items()}

        self.VALID_TASKS = {
            'flag': 'Flag',
            'crank': 'Crank',
            'push': 'Push',
        }

        self.VALID_COLORS = {
            'RED',
            'PURPLE',
            'GREEN',
            'BLUE',
        }

        # ---------------------------------------------------------------
        # Pub / Sub
        # ---------------------------------------------------------------
        self.cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self.ir_pub = self.create_publisher(String, ir_topic, 10)
        self.drone_pub = self.create_publisher(String, drone_topic, 10)

        self.sensor_sub = self.create_subscription(
            Float32MultiArray, sensor_topic, self.sensor_callback, 10
        )

        self.color_sub = self.create_subscription(
            String, color_topic, self.color_callback, 10
        )

        self.ready_sub = self.create_subscription(
            String, ready_topic, self.ready_callback, 10
        )

        # ---------------------------------------------------------------
        # State
        # ---------------------------------------------------------------
        self.sequence = []
        self.step_index = 0
        self.timer = None
        self.started = False
        self.active_sensor_step = False
        self.active_wait_ready_step = False
        self.last_sensor_data = []
        self.step_complete = False
        self.arduino_ready_received = False

        self.latest_detected_color = None
        self.detected_colors = ["", "", "", ""]

        self.ir_repeat_timer = None
        self.ir_repeat_remaining = 0
        self.ir_repeat_text = ''
        self.ir_repeat_gap = 0.1

        # ---------------------------------------------------------------
        # Robot Sequence
        # ---------------------------------------------------------------
        self.sequence = [
            {
                'type': 'wait_for_arduino_ready',
                'timeout': 600.0,
            },
            {
                'type': 'display_colors',
                'colors': ['', '', '', ''],
                'duration': 2.0,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'forward',
                'distance': 0.17,
                'sensor_name': 'back_mid',
                'target_value': 0.840,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'reverse',
                'distance': 0.17,
                'sensor_name': 'back_mid',
                'target_value': 0.800,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 1.0,
            },

            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'forward',
                'distance': 0.17,
                'sensor_name': 'back_mid',
                'target_value': 0.840,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'reverse',
                'distance': 0.17,
                'sensor_name': 'back_mid',
                'target_value': 0.800,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 1.0,
            },

            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'forward',
                'distance': 0.17,
                'sensor_name': 'back_mid',
                'target_value': 0.840,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'reverse',
                'distance': 0.17,
                'sensor_name': 'back_mid',
                'target_value': 0.800,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 1.0,
            },
            {
                'type': 'capture_color',
                'array_index': 0,
            },
            {
                'type': 'display_colors',
                'duration': 2.0,
            },
            {
                'type': 'task_cmd',
                'task_name': 'Flag',
                'duration': 5.0,
            },
            {
                'type': 'stop',
                'duration': 1.0,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'reverse',
                'distance': 0.17,
                'sensor_name': 'back_mid',
                'target_value': 0.606,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 1.0,
            },
            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': -3.0,
                'duration': 1.35,
            },
            {
                'type': 'stop',
                'duration': 1.0,
            },
            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': 3.0,
                'duration': 2.3,
            },
            {
                'type': 'stop',
                'duration': 1.0,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'forward',
                'distance': 0.59,
                'sensor_name': 'back_mid',
                'target_value': 1.104,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },

            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'task_cmd',
                'task_name': 'Push',
                'duration': 15.0,
            },
            {
                'type': 'stop',
                'duration': 2.0,
            },
            {
                'type': 'capture_color',
                'array_index': 2,
            },
            {
                'type': 'display_colors',
                'duration': 2.0,
            },

            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'reverse',
                'distance': 0.59,
                'sensor_name': 'back_mid',
                'target_value': 0.983,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': -3.0,
                'right_vel': 3.0,
                'duration': 1.35,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },

            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': 3.0,
                'duration': 3.5,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': -3.0,
                'duration': 1.35,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },

            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'forward',
                'distance': 0.17,
                'sensor_name': 'back_mid',
                'target_value': 1.940,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },

            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'reverse',
                'distance': 0.17,
                'sensor_name': 'back_mid',
                'target_value': 1.806,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 1.00,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },

            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': -3.0,
                'duration': 0.550,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': 3.0,
                'duration': 4.15,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': -3.0,
                'duration': 0.675,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'forward',
                'distance': 0.25,
                'sensor_name': 'front_mid',
                'target_value': 0.211,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'reverse',
                'distance': 0.25,
                'sensor_name': 'front_mid',
                'target_value': 0.817,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'task_cmd',
                'task_name': 'Crank',
                'duration': 2.0,
            },
            {
                'type': 'stop',
                'duration': 1.0,
            },
            {
                'type': 'task_cmd',
                'task_name': 'Crank',
                'duration': 2.0,
            },
            {
                'type': 'stop',
                'duration': 1.0,
            },
            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': 3.0,
                'duration': 0.5,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': -3.0,
                'duration': 3.15,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'capture_color',
                'array_index': 1,
            },
            {
                'type': 'stop',
                'duration': 1.5,
            },
            {
                'type': 'display_colors',
                'duration': 2.0,
            },
            {
                'type': 'stop',
                'duration': 1.5,
            },
            {
                'type': 'publish_ir_colors',
                'duration': 10,
                'repeat_count': 10,
                'repeat_gap': 1.00,
            },
            {
                'type': 'launch_drone',
                'duration': 3.0,
            },
            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': -3.0,
                'duration': 1.35,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': -3.0,
                'right_vel': -3.0,
                'duration': 3.75,
            },
            {
                'type': 'stop',
                'duration': 1.0,
            },
            {
                'type': 'raw_cmd',
                'left_vel': -3.0,
                'right_vel': 3.0,
                'duration': 0.25,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'reverse',
                'distance': 0.20,
                'sensor_name': 'front_mid',
                'target_value': 1.500,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 20.00,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': -3.0,
                'right_vel': 3.0,
                'duration': 1.35,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': -3.0,
                'right_vel': -3.0,
                'duration': 3.75,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': 3.0,
                'right_vel': -3.0,
                'duration': 1.35,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'reverse',
                'distance': 0.59,
                'sensor_name': 'back_mid',
                'target_value': 0.42,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 0.0,
            },

            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': -3.0,
                'right_vel': -3.0,
                'duration': 1.35,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'raw_cmd',
                'left_vel': -3.0,
                'right_vel': 3.0,
                'duration': 1.35,
            },
            {
                'type': 'stop',
                'duration': 0.5,
            },
            {
                'type': 'wall_follow_until_sensor',
                'wall': 'left',
                'direction': 'reverse',
                'distance': 0.17,
                'sensor_name': 'back_mid',
                'target_value': 0.17,
                'condition': 'near',
                'tolerance': 0.015,
                'timeout': 20.00,
            },
        ]

        self.get_logger().info('program_controller_node ready.')
        self.get_logger().info(f'Publishing commands to: {cmd_topic}')
        self.get_logger().info(f'Publishing IR colors to: {ir_topic}')
        self.get_logger().info(f'Publishing drone commands to: {drone_topic}')
        self.get_logger().info(f'Subscribed to sensors on: {sensor_topic}')
        self.get_logger().info(f'Subscribed to detected colors on: {color_topic}')
        self.get_logger().info(f'Subscribed to Arduino ready messages on: {ready_topic}')
        self.get_logger().info(
            'Sensor map: '
            '0=left_front, 1=left_mid, 2=left_back, '
            '3=right_front, 4=right_mid, 5=right_back, '
            '6=front_left, 7=front_mid, 8=front_right, '
            '9=back_left, 10=back_mid, 11=back_right'
        )

        if auto_start:
            self.timer = self.create_timer(0.5, self._start_once)

    def _start_once(self):
        if self.started:
            return

        self.started = True

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        self.get_logger().info('Starting programmed sequence.')
        self._run_current_step()

    def sensor_callback(self, msg: Float32MultiArray):
        self.last_sensor_data = list(msg.data)

        if not self.active_sensor_step:
            return

        if self.step_index >= len(self.sequence):
            return

        step = self.sequence[self.step_index]
        if step.get('type', '').lower() != 'wall_follow_until_sensor':
            return

        sensor_index = self._resolve_sensor_index(step)
        if sensor_index is None:
            return

        target_value = float(step['target_value'])
        condition = step.get('condition', 'near').lower()
        tolerance = float(step.get('tolerance', 0.01))

        if sensor_index < 0 or sensor_index >= len(self.last_sensor_data):
            self.get_logger().warn(
                f'Sensor index {sensor_index} out of range for /sensors length '
                f'{len(self.last_sensor_data)}'
            )
            return

        sensor_value = self.last_sensor_data[sensor_index]

        if sensor_value < 0.0:
            return

        if self._sensor_condition_met(sensor_value, target_value, condition, tolerance):
            sensor_name = self.SENSOR_NAME.get(sensor_index, f'index_{sensor_index}')
            self.get_logger().info(
                f'Sensor condition met at step {self.step_index}: '
                f'{sensor_name}[{sensor_index}]={sensor_value:.3f}, '
                f'target={target_value:.3f}, condition={condition}, tol={tolerance:.3f}'
            )
            self._finish_active_sensor_step()

    def color_callback(self, msg: String):
        color = msg.data.strip().upper()

        if color not in self.VALID_COLORS:
            self.get_logger().warn(
                f'Ignoring invalid detected color "{msg.data}". '
                f'Valid colors: RED, PURPLE, GREEN, BLUE'
            )
            return

        self.latest_detected_color = color
        self.get_logger().info(
            f'Latest detected color updated: {self.latest_detected_color}',
            throttle_duration_sec=0.5
        )

    def ready_callback(self, msg: String):
        text = msg.data.strip()

        if text != 'ARDUINO READY':
            return

        self.arduino_ready_received = True

        if not self.active_wait_ready_step:
            self.get_logger().info('Received "ARDUINO READY" while not waiting. Latching ready state.')
            return

        if self.step_index >= len(self.sequence):
            return

        step = self.sequence[self.step_index]
        if step.get('type', '').lower() != 'wait_for_arduino_ready':
            return

        self.get_logger().info(
            f'Received "ARDUINO READY" at step {self.step_index}. Advancing sequence.'
        )
        self._finish_wait_ready_step()

    def _resolve_sensor_index(self, step):
        if 'sensor_name' in step:
            sensor_name = str(step['sensor_name']).strip().lower()
            if sensor_name not in self.SENSOR_INDEX:
                self.get_logger().error(
                    f'Unknown sensor_name "{sensor_name}". '
                    f'Valid names: {list(self.SENSOR_INDEX.keys())}'
                )
                return None
            return self.SENSOR_INDEX[sensor_name]

        if 'sensor_index' in step:
            try:
                sensor_index = int(step['sensor_index'])
            except (TypeError, ValueError):
                self.get_logger().error(f'Invalid sensor_index: {step["sensor_index"]}')
                return None

            if sensor_index not in self.SENSOR_NAME:
                self.get_logger().error(
                    f'Invalid sensor_index {sensor_index}. Valid indices are 0 through 11.'
                )
                return None
            return sensor_index

        self.get_logger().error(
            'Sensor-based step is missing sensor_name or sensor_index.'
        )
        return None

    def _resolve_task_name(self, task_name_raw):
        task_key = str(task_name_raw).strip().lower()
        if task_key not in self.VALID_TASKS:
            self.get_logger().error(
                f'Invalid task "{task_name_raw}". Valid tasks: Flag, Crank, Push'
            )
            return None
        return self.VALID_TASKS[task_key]

    def _sensor_condition_met(self, value: float, target: float, condition: str, tol: float) -> bool:
        if condition == 'near':
            return abs(value - target) <= tol
        elif condition == 'gte':
            return value >= target
        elif condition == 'lte':
            return value <= target
        else:
            self.get_logger().warn(f'Unknown condition "{condition}", using "near"')
            return abs(value - target) <= tol

    def _run_current_step(self):
        self.active_sensor_step = False
        self.active_wait_ready_step = False
        self.step_complete = False

        if self.step_index >= len(self.sequence):
            self.get_logger().info('Sequence complete. Sending final stop.')
            self._publish_cmd('stop')
            self.get_logger().info(f'Final detected_colors array: {self.detected_colors}')
            return

        step = self.sequence[self.step_index]
        step_type = step.get('type', '').lower()

        if step_type == 'wait_for_arduino_ready':
            timeout = float(step.get('timeout', 30.0))

            if self.arduino_ready_received:
                self.get_logger().info(
                    f'Step {self.step_index}: Arduino was already ready. Advancing immediately.'
                )
                self._schedule_timer(0.1, self._advance_sequence)
                return

            self.active_wait_ready_step = True
            self.get_logger().info(
                f'Step {self.step_index}: waiting for "ARDUINO READY" with timeout {timeout:.2f} s'
            )

            if timeout > 0.0:
                self._schedule_timer(timeout, self._arduino_ready_timeout)

        elif step_type == 'wall_follow':
            wall = step.get('wall', 'left').lower()
            direction = step.get('direction', 'forward').lower()
            distance = float(step.get('distance', 0.17))
            duration = float(step.get('duration', 1.0))

            cmd = self._build_wall_command(wall, direction, distance)
            if cmd is None:
                self.get_logger().error(f'Invalid wall_follow step at index {self.step_index}')
                self._publish_cmd('stop')
                return

            self.get_logger().info(
                f'Step {self.step_index}: {cmd} for {duration:.2f} s'
            )
            self._publish_cmd(cmd)
            self._schedule_timer(duration, self._advance_sequence)

        elif step_type == 'wall_follow_until_sensor':
            wall = step.get('wall', 'left').lower()
            direction = step.get('direction', 'forward').lower()
            distance = float(step.get('distance', 0.17))
            target_value = float(step.get('target_value', 0.0))
            condition = step.get('condition', 'near').lower()
            tolerance = float(step.get('tolerance', 0.01))
            timeout = float(step.get('timeout', 0.0))

            sensor_index = self._resolve_sensor_index(step)
            if sensor_index is None:
                self._publish_cmd('stop')
                return

            sensor_name = self.SENSOR_NAME.get(sensor_index, f'index_{sensor_index}')

            cmd = self._build_wall_command(wall, direction, distance)
            if cmd is None:
                self.get_logger().error(f'Invalid wall_follow_until_sensor step at index {self.step_index}')
                self._publish_cmd('stop')
                return

            self.get_logger().info(
                f'Step {self.step_index}: {cmd} until {sensor_name}[{sensor_index}] '
                f'{condition} {target_value:.3f} (tol={tolerance:.3f})'
            )
            self._publish_cmd(cmd)

            self.active_sensor_step = True

            if timeout > 0.0:
                self.get_logger().info(f'  Timeout enabled: {timeout:.2f} s')
                self._schedule_timer(timeout, self._sensor_step_timeout)

        elif step_type == 'raw_cmd':
            left_vel = float(step.get('left_vel', 0.0))
            right_vel = float(step.get('right_vel', 0.0))
            duration = float(step.get('duration', 1.0))

            cmd = f'raw {left_vel:.3f} {right_vel:.3f}'

            self.get_logger().info(
                f'Step {self.step_index}: {cmd} for {duration:.2f} s'
            )
            self._publish_cmd(cmd)
            self._schedule_timer(duration, self._advance_sequence)

        elif step_type == 'task_cmd':
            task_name = self._resolve_task_name(step.get('task_name', ''))
            duration = float(step.get('duration', 1.0))

            if task_name is None:
                self._publish_cmd('stop')
                return

            cmd = f'task {task_name}'

            self.get_logger().info(
                f'Step {self.step_index}: {cmd} for {duration:.2f} s'
            )
            self._publish_cmd(cmd)
            self._schedule_timer(duration, self._advance_sequence)

        elif step_type == 'capture_color':
            array_index = int(step.get('array_index', 0))
            self._capture_latest_color(array_index)
            self._schedule_timer(0.1, self._advance_sequence)

        elif step_type == 'display_colors':
            duration = float(step.get('duration', 1.0))

            if 'colors' in step:
                colors = step['colors']
            else:
                colors = self.detected_colors

            cmd = self._build_display_command(colors)
            if cmd is None:
                self._publish_cmd('stop')
                return

            self.get_logger().info(
                f'Step {self.step_index}: {cmd} for {duration:.2f} s'
            )
            self._publish_cmd(cmd)
            self._schedule_timer(duration, self._advance_sequence)

        elif step_type == 'publish_ir_colors':
            duration = float(step.get('duration', 1.0))
            repeat_count = int(step.get('repeat_count', 5))
            repeat_gap = float(step.get('repeat_gap', 0.10))

            if 'colors' in step:
                colors = step['colors']
            else:
                colors = self.detected_colors

            ir_text = self._build_ir_color_string(colors)
            if ir_text is None:
                self._publish_cmd('stop')
                return

            if repeat_count < 1:
                self.get_logger().warn(
                    f'publish_ir_colors repeat_count={repeat_count} is invalid, using 1'
                )
                repeat_count = 1

            if repeat_gap <= 0.0:
                self.get_logger().warn(
                    f'publish_ir_colors repeat_gap={repeat_gap} is invalid, using 0.10'
                )
                repeat_gap = 0.10

            self.get_logger().info(
                f'Step {self.step_index}: publish IR colors "{ir_text}" '
                f'{repeat_count} time(s), gap={repeat_gap:.2f} s, '
                f'step duration={duration:.2f} s'
            )

            self._start_ir_publish_burst(ir_text, repeat_count, repeat_gap)
            self._schedule_timer(duration, self._advance_sequence)

        elif step_type == 'launch_drone':
            duration = float(step.get('duration', 3.0))
            self.get_logger().info(
                f'Step {self.step_index}: publishing LAUNCH_DRONE for {duration:.2f} s'
            )
            self._publish_drone_command('LAUNCH_DRONE')
            self._schedule_timer(duration, self._advance_sequence)

        elif step_type == 'stop':
            duration = float(step.get('duration', 1.0))
            self.get_logger().info(
                f'Step {self.step_index}: stop for {duration:.2f} s'
            )
            self._publish_cmd('stop')
            self._schedule_timer(duration, self._advance_sequence)

        else:
            self.get_logger().error(
                f'Unknown step type at index {self.step_index}: {step_type}'
            )
            self._publish_cmd('stop')

    def _capture_latest_color(self, array_index: int):
        if array_index < 0 or array_index >= len(self.detected_colors):
            self.get_logger().error(
                f'capture_color index {array_index} out of range. Valid indices: 0..3'
            )
            return

        if self.latest_detected_color is None:
            self.get_logger().warn(
                f'No valid color has been received yet. '
                f'detected_colors unchanged: {self.detected_colors}'
            )
            return

        self.detected_colors[array_index] = self.latest_detected_color
        self.get_logger().info(
            f'Captured color "{self.latest_detected_color}" into '
            f'detected_colors[{array_index}]'
        )
        self.get_logger().info(f'detected_colors = {self.detected_colors}')

    def _build_wall_command(self, wall: str, direction: str, distance: float):
        if wall != 'left':
            self.get_logger().warn(
                f'Wall "{wall}" is not supported yet. Current wall_follower_node '
                f'only supports left wall.'
            )
            return None

        if direction == 'forward':
            return f'left_forward {distance:.3f}'
        elif direction == 'reverse':
            return f'left_reverse {distance:.3f}'
        else:
            self.get_logger().warn(
                f'Direction "{direction}" is invalid. Use "forward" or "reverse".'
            )
            return None

    def _build_display_command(self, colors):
        if len(colors) != 4:
            self.get_logger().error(
                f'display_colors requires exactly 4 entries, got {len(colors)}'
            )
            return None

        cleaned = []
        for c in colors:
            color = str(c).strip().upper()
            if color == '':
                cleaned.append('')
            elif color in self.VALID_COLORS:
                cleaned.append(color)
            else:
                self.get_logger().error(
                    f'Invalid color "{c}" for display command. '
                    f'Valid colors are RED, PURPLE, GREEN, BLUE, or empty string.'
                )
                return None

        return f'display {cleaned[0]},{cleaned[1]},{cleaned[2]},{cleaned[3]}'

    def _build_ir_color_string(self, colors):
        if len(colors) != 4:
            self.get_logger().error(
                f'publish_ir_colors requires exactly 4 entries, got {len(colors)}'
            )
            return None

        cleaned = []
        for c in colors:
            color = str(c).strip().upper()
            if color == '':
                cleaned.append('')
            elif color in self.VALID_COLORS:
                cleaned.append(color)
            else:
                self.get_logger().error(
                    f'Invalid color "{c}" for IR publish command. '
                    f'Valid colors are RED, PURPLE, GREEN, BLUE, or empty string.'
                )
                return None

        return f'{cleaned[0]},{cleaned[1]},{cleaned[2]},{cleaned[3]}'

    def _publish_cmd(self, cmd_text: str):
        msg = String()
        msg.data = cmd_text
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Published: "{cmd_text}"')

    def _publish_ir_colors(self, color_text: str):
        msg = String()
        msg.data = color_text
        self.ir_pub.publish(msg)
        self.get_logger().info(f'Published to /ir_colors: "{color_text}"')

    def _publish_drone_command(self, cmd_text: str):
        msg = String()
        msg.data = cmd_text
        self.drone_pub.publish(msg)
        self.get_logger().info(f'Published to /drone_command: "{cmd_text}"')

    def _start_ir_publish_burst(self, color_text: str, repeat_count: int, repeat_gap: float):
        if self.ir_repeat_timer is not None:
            self.ir_repeat_timer.cancel()
            self.ir_repeat_timer = None

        self.ir_repeat_text = color_text
        self.ir_repeat_remaining = repeat_count
        self.ir_repeat_gap = repeat_gap

        self._ir_publish_burst_callback()

    def _ir_publish_burst_callback(self):
        if self.ir_repeat_remaining <= 0:
            if self.ir_repeat_timer is not None:
                self.ir_repeat_timer.cancel()
                self.ir_repeat_timer = None
            return

        self._publish_ir_colors(self.ir_repeat_text)
        self.ir_repeat_remaining -= 1

        if self.ir_repeat_remaining > 0:
            if self.ir_repeat_timer is not None:
                self.ir_repeat_timer.cancel()
                self.ir_repeat_timer = None

            self.ir_repeat_timer = self.create_timer(
                self.ir_repeat_gap,
                self._ir_publish_burst_callback
            )
        else:
            if self.ir_repeat_timer is not None:
                self.ir_repeat_timer.cancel()
                self.ir_repeat_timer = None

    def _schedule_timer(self, duration_sec: float, callback):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        self.timer = self.create_timer(duration_sec, callback)

    def _finish_active_sensor_step(self):
        if self.step_complete:
            return

        self.step_complete = True
        self.active_sensor_step = False

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        self._publish_cmd('stop')
        self._schedule_timer(0.2, self._advance_sequence)

    def _finish_wait_ready_step(self):
        if self.step_complete:
            return

        self.step_complete = True
        self.active_wait_ready_step = False

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        self._schedule_timer(0.2, self._advance_sequence)

    def _sensor_step_timeout(self):
        if self.step_complete:
            return

        self.step_complete = True
        self.active_sensor_step = False

        self.get_logger().warn(
            f'Step {self.step_index} timed out before sensor condition was met. Stopping.'
        )
        self._publish_cmd('stop')
        self._schedule_timer(0.2, self._advance_sequence)

    def _arduino_ready_timeout(self):
        if self.step_complete:
            return

        self.step_complete = True
        self.active_wait_ready_step = False

        self.get_logger().warn(
            f'Step {self.step_index} timed out waiting for "ARDUINO READY". Advancing sequence.'
        )
        self._schedule_timer(0.2, self._advance_sequence)

    def _advance_sequence(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        self.step_index += 1
        self._run_current_step()

    def destroy_node(self):
        self.get_logger().info('Shutting down controller. Sending stop.')
        self._publish_cmd('stop')
        self.get_logger().info(f'detected_colors at shutdown: {self.detected_colors}')

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        if self.ir_repeat_timer is not None:
            self.ir_repeat_timer.cancel()
            self.ir_repeat_timer = None

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ProgramControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
