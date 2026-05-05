#!/usr/bin/env python3
"""
wall_follower_node.py
=====================
Subscribes to /sensors (Float32MultiArray from lidar_twelve_sensors.py),
uses the three left-side readings to follow the left wall at a target distance,
and sends differential-drive wheel velocity commands to an Arduino over serial.

Also subscribes to /wall_cmd (std_msgs/String) to receive commands from
program_controller_node. Valid command strings:
    "left_forward <distance_m>"    e.g. "left_forward 0.17"
    "left_reverse <distance_m>"    e.g. "left_reverse 0.17"
    "raw <left_vel> <right_vel>"   e.g. "raw 3 -3"
    "task <task_name>"             e.g. "task Flag"
    "display <c1>,<c2>,<c3>,<c4>"  e.g. "display RED,,BLUE,PURPLE"
    "stop"

Supported task names:
    Flag
    Crank
    Push

Sensor layout (from lidar_twelve_sensors.py):
  Index 0  left_front  angle = -110 deg  (20 deg forward of perpendicular)
  Index 1  left_mid    angle =  -90 deg  (perpendicular to wall)
  Index 2  left_back   angle =  -70 deg  (20 deg rearward of perpendicular)

A reading of -1.0 means the sensor returned no valid data.

Serial command formats sent to Arduino:
  Wheel command:   "C <left_vel> <right_vel>\r\n"
  Task command :   "T <task_name>\n"
  Display command: "D <c1>,<c2>,<c3>,<c4>\n"

Robot geometry:
  Wheel radius  r  = 3 in  = 0.0762 m
  Wheel base    L  = 7 in  = 0.1778 m
"""

import math
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


# ---------------------------------------------------------------------------
# Robot physical constants
# ---------------------------------------------------------------------------
WHEEL_RADIUS_M   = 3.0 * 0.0254
WHEEL_BASE_M     = 7.0 * 0.0254

SENSOR_ANGLE_DEG = 20.0
SENSOR_ANGLE_RAD = math.radians(SENSOR_ANGLE_DEG)

# Command states
STATE_STOP         = 'stop'
STATE_LEFT_FORWARD = 'left_forward'
STATE_LEFT_REVERSE = 'left_reverse'

# Supported task names
VALID_TASKS = {
    'flag': 'Flag',
    'crank': 'Crank',
    'push': 'Push',
}


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # ---- ROS parameters ------------------------------------------------
        self.declare_parameter('sensor_topic',    '/sensors')
        self.declare_parameter('cmd_topic',       '/wall_cmd')
        self.declare_parameter('ready_topic',     '/arduino_ready')
        self.declare_parameter('serial_port',     '/dev/ttyACM0')
        self.declare_parameter('serial_baud',     115200)

        self.declare_parameter('target_distance', 0.170)
        self.declare_parameter('base_speed',      3.00)

        self.declare_parameter('kp',              12.00)
        self.declare_parameter('ki',              0.02)
        self.declare_parameter('kd',              0.35)

        self.declare_parameter('max_correction',  0.50)
        self.declare_parameter('max_wheel_speed', 3.60)
        self.declare_parameter('min_wheel_speed', 0.20)

        # ---- Read parameters -----------------------------------------------
        sensor_topic  = self.get_parameter('sensor_topic').value
        cmd_topic     = self.get_parameter('cmd_topic').value
        ready_topic   = self.get_parameter('ready_topic').value
        serial_port   = self.get_parameter('serial_port').value
        serial_baud   = int(self.get_parameter('serial_baud').value)

        self.target_dist    = float(self.get_parameter('target_distance').value)
        self.base_speed     = float(self.get_parameter('base_speed').value)
        self.kp             = float(self.get_parameter('kp').value)
        self.ki             = float(self.get_parameter('ki').value)
        self.kd             = float(self.get_parameter('kd').value)
        self.max_correction = float(self.get_parameter('max_correction').value)
        self.max_wheel_spd  = float(self.get_parameter('max_wheel_speed').value)
        self.min_wheel_spd  = float(self.get_parameter('min_wheel_speed').value)

        # ---- Runtime state -------------------------------------------------
        self.direction   = 0
        self.prev_error  = 0.0
        self.integral    = 0.0
        self.prev_time   = None
        self.serial_rx_buffer = ''
        self.arduino_ready_latched = False

        # ---- Serial --------------------------------------------------------
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=serial_baud,
                timeout=0.05,
                write_timeout=1.0,
                dsrdtr=False,
                rtscts=False,
            )
            self.ser.dtr = False
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(
                f'Opened serial port {serial_port} @ {serial_baud} baud'
            )
        except serial.SerialException as exc:
            self.get_logger().error(f'Cannot open serial port: {exc}')
            self.ser = None

        # ---- Publishers ----------------------------------------------------
        self.ready_pub = self.create_publisher(String, ready_topic, 10)

        # ---- Subscribers ---------------------------------------------------
        self.sensor_sub = self.create_subscription(
            Float32MultiArray, sensor_topic, self.sensor_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            String, cmd_topic, self.cmd_callback, 10
        )

        # ---- Serial polling timer ------------------------------------------
        self.serial_poll_timer = self.create_timer(0.02, self._poll_serial)

        self.get_logger().info('wall_follower_node ready.')
        self.get_logger().info(f'  Sensor topic : {sensor_topic}')
        self.get_logger().info(f'  Command topic: {cmd_topic}')
        self.get_logger().info(f'  Ready topic  : {ready_topic}')
        self.get_logger().info(f'  Default target distance: {self.target_dist:.3f} m')

    # -----------------------------------------------------------------------
    # Command callback  (/wall_cmd)
    # -----------------------------------------------------------------------
    def cmd_callback(self, msg: String):
        parts = msg.data.strip().split()
        if not parts:
            return

        cmd = parts[0].lower()

        if cmd == STATE_STOP:
            self.get_logger().info('CMD: stop')
            self.direction = 0
            self._reset_pid()
            self._send_command(0.0, 0.0)

        elif cmd == STATE_LEFT_FORWARD:
            if len(parts) >= 2:
                try:
                    self.target_dist = float(parts[1])
                except ValueError:
                    self.get_logger().warn(f'Bad distance value: {parts[1]}')
            self.get_logger().info(
                f'CMD: left_forward  target={self.target_dist:.3f} m'
            )
            self._reset_pid()
            self.direction = 1

        elif cmd == STATE_LEFT_REVERSE:
            if len(parts) >= 2:
                try:
                    self.target_dist = float(parts[1])
                except ValueError:
                    self.get_logger().warn(f'Bad distance value: {parts[1]}')
            self.get_logger().info(
                f'CMD: left_reverse  target={self.target_dist:.3f} m'
            )
            self._reset_pid()
            self.direction = -1

        elif cmd == 'raw':
            if len(parts) < 3:
                self.get_logger().warn(f'Bad raw command: {msg.data}')
                return

            try:
                left_vel = float(parts[1])
                right_vel = float(parts[2])
            except ValueError:
                self.get_logger().warn(f'Bad raw wheel velocities: {msg.data}')
                return

            self.get_logger().info(
                f'CMD: raw  left={left_vel:.3f} right={right_vel:.3f}'
            )

            self.direction = 0
            self._reset_pid()
            self._send_command(left_vel, right_vel)

        elif cmd in ('task', 't'):
            if len(parts) < 2:
                self.get_logger().warn(f'Bad task command: {msg.data}')
                return

            task_key = parts[1].strip().lower()
            if task_key not in VALID_TASKS:
                self.get_logger().warn(
                    f'Invalid task "{parts[1]}". Valid tasks: Flag, Crank, Push'
                )
                return

            task_name = VALID_TASKS[task_key]
            self.get_logger().info(f'CMD: task {task_name}')

            self.direction = 0
            self._reset_pid()
            self._send_task_command(task_name)

        elif cmd == 'display':
            payload = msg.data[len(parts[0]):].strip()
            if payload == '':
                self.get_logger().warn(f'Bad display command: {msg.data}')
                return

            self.get_logger().info(f'CMD: display {payload}')

            self.direction = 0
            self._reset_pid()
            self._send_display_command(payload)

        else:
            self.get_logger().warn(f'Unknown command: {msg.data}')

    # -----------------------------------------------------------------------
    # Sensor callback  (/sensors)
    # -----------------------------------------------------------------------
    def sensor_callback(self, msg: Float32MultiArray):
        if self.direction == 0:
            return

        data = msg.data
        if len(data) < 3:
            self.get_logger().warn('Fewer than 3 sensor values — skipping.')
            return

        lf = data[0]
        lm = data[1]
        lb = data[2]

        perp_dist = self._estimate_perp_distance(lf, lm, lb)
        if perp_dist is None:
            self.get_logger().warn('All left sensors invalid — stopping.')
            self._send_command(0.0, 0.0)
            return

        heading_error = self._estimate_heading_error(lf, lb)

        distance_error = (perp_dist - self.target_dist) * self.direction

        HEADING_WEIGHT = 0.5
        error = distance_error + HEADING_WEIGHT * heading_error

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = (now - self.prev_time) if (self.prev_time and now > self.prev_time) else 0.05
        self.prev_time = now

        self.integral += error * dt
        max_int = self.max_correction / max(self.ki, 1e-9)
        self.integral = max(-max_int, min(max_int, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        correction = (
                self.kp * error
                + self.ki * self.integral
                + self.kd * derivative
        )
        correction = max(-self.max_correction, min(self.max_correction, correction))

        signed_base = self.base_speed * self.direction

        left_vel = signed_base - correction
        right_vel = signed_base + correction

        left_vel = self._clamp_wheel(left_vel)
        right_vel = self._clamp_wheel(right_vel)

        self._send_command(left_vel, right_vel)

        self.get_logger().info(
            f'dir={self.direction:+d}  perp={perp_dist:.3f}m  '
            f'hdg={math.degrees(heading_error):.1f}°  '
            f'err={error:.3f}  corr={correction:.3f}  '
            f'L={left_vel:.3f}  R={right_vel:.3f}',
            throttle_duration_sec=0.25
        )

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------
    def _clamp_wheel(self, vel: float) -> float:
        if vel == 0.0:
            return 0.0
        if vel > 0.0:
            return max(self.min_wheel_spd, min(self.max_wheel_spd, vel))
        return min(-self.min_wheel_spd, max(-self.max_wheel_spd, vel))

    def _estimate_perp_distance(self, lf, lm, lb):
        estimates = []
        if lm > 0.0:
            estimates.append(lm)
        if lf > 0.0:
            estimates.append(lf * math.cos(SENSOR_ANGLE_RAD))
        if lb > 0.0:
            estimates.append(lb * math.cos(SENSOR_ANGLE_RAD))
        return (sum(estimates) / len(estimates)) if estimates else None

    def _estimate_heading_error(self, lf, lb) -> float:
        if lf <= 0.0 or lb <= 0.0:
            return 0.0

        lf_perp  = lf * math.cos(SENSOR_ANGLE_RAD)
        lb_perp  = lb * math.cos(SENSOR_ANGLE_RAD)
        lf_axial = lf * math.sin(SENSOR_ANGLE_RAD)
        lb_axial = lb * math.sin(SENSOR_ANGLE_RAD)
        sep      = lf_axial + lb_axial

        if sep < 0.01:
            return 0.0

        return math.atan2(lf_perp - lb_perp, sep)

    def _reset_pid(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def _send_command(self, left_vel: float, right_vel: float):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not open — command not sent.')
            return

        cmd = f'C {left_vel:.4f} {right_vel:.4f}\r\n'
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            self.get_logger().debug(f'Serial TX: {cmd.strip()}')
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}')

    def _send_task_command(self, task_name: str):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not open — task not sent.')
            return

        cmd = f'T {task_name}\n'
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            self.get_logger().info(f'Serial TX task: {cmd.strip()}')
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial task write failed: {exc}')

    def _send_display_command(self, payload: str):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not open — display command not sent.')
            return

        cmd = f'D {payload}\n'
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            self.get_logger().info(f'Serial TX display: {cmd.strip()}')
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial display write failed: {exc}')

    # -----------------------------------------------------------------------
    # Poll serial continuously so ARDUINO READY can be seen even when idle
    # -----------------------------------------------------------------------
    def _poll_serial(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            waiting = self.ser.in_waiting
            if waiting <= 0:
                return

            data = self.ser.read(waiting)
            if not data:
                return

            text = data.decode('utf-8', errors='replace')
            self.serial_rx_buffer += text

            while '\n' in self.serial_rx_buffer:
                line, self.serial_rx_buffer = self.serial_rx_buffer.split('\n', 1)
                line = line.strip()
                if line == '':
                    continue

                self.get_logger().info(f'Arduino RX: {line}')

                if line == 'ARDUINO READY':
                    self._publish_arduino_ready()

        except serial.SerialException as exc:
            self.get_logger().error(f'Serial read failed: {exc}')

    def _publish_arduino_ready(self):
        msg = String()
        msg.data = 'ARDUINO READY'
        self.ready_pub.publish(msg)

        if not self.arduino_ready_latched:
            self.get_logger().info('Published /arduino_ready: "ARDUINO READY"')
            self.arduino_ready_latched = True
        else:
            self.get_logger().info(
                'Published /arduino_ready: "ARDUINO READY"',
                throttle_duration_sec=1.0
            )

    def destroy_node(self):
        self.get_logger().info('Shutting down — sending stop.')
        self._send_command(0.0, 0.0)

        if self.serial_poll_timer is not None:
            self.serial_poll_timer.cancel()
            self.serial_poll_timer = None

        if self.ser and self.ser.is_open:
            self.ser.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
