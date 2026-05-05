#!/usr/bin/env python3

"""
send_ir_node.py
---------------
ROS2 node – bridges /ir_colors topic to BLE GATT writes on XIAO ESP32C3.

Subscription : /ir_colors  (std_msgs/String)
Format       : "RED,GREEN,,BLUE"

Payload encoding:
byte = (antenna_nibble << 4) | color_nibble

IMPORTANT:
- Blank antenna entries are skipped entirely.
- 0x00 is never sent.
- Payload length can be 1, 2, 3, or 4 bytes
  (2, 4, 6, or 8 hex digits).
"""

import asyncio
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bleak import BleakClient, BleakScanner, BleakError


# ───────────────── BLE configuration ─────────────────

DEVICE_NAME = "XIAO-IR-BRIDGE"

SERVICE_UUID  = "12345678-1234-1234-1234-1234567890ab"
CHAR_UUID_CMD = "abcdefab-1234-5678-1234-56789abcdef0"

# static MAC
STATIC_MAC = "58:8C:81:A1:86:C2"


# ───────────────── Encoding tables ─────────────────

ANTENNA_NIBBLES = [0x0, 0x3, 0x5, 0x6]

COLOR_NIBBLES = {
    "RED":    0x9,
    "GREEN":  0xA,
    "BLUE":   0xC,
    "PURPLE": 0xF,
}


# ───────────────── BLE parameters ─────────────────

BLE_SCAN_TIMEOUT = 10.0
BLE_CONNECT_TIMEOUT = 15.0
BLE_RECONNECT_DELAY = 2.0

WRITE_WITH_RESPONSE = True


# ───────────────── Node implementation ─────────────────

class SendIRNode(Node):

    def __init__(self):
        super().__init__("send_ir_node")

        self.declare_parameter("ir_topic", "/ir_colors")
        ir_topic = self.get_parameter("ir_topic").value

        self.sub = self.create_subscription(
            String,
            ir_topic,
            self.ir_callback,
            10
        )

        self.get_logger().info("send_ir_node started")
        self.get_logger().info(f"topic : {ir_topic}")

        self._loop = None
        self._queue = None
        self._client = None
        self._connected = False
        self._shutdown = threading.Event()

        self._ble_thread = threading.Thread(
            target=self._ble_thread_main,
            daemon=True
        )
        self._ble_thread.start()

    # ───────────────── ROS callback ─────────────────

    def ir_callback(self, msg: String):

        text = msg.data.strip()
        self.get_logger().info(f"Received: {text}")

        try:
            colors = self.parse_colors(text)
            payload = self.colors_to_payload(colors)

            if len(payload) == 0:
                self.get_logger().warning("No valid antenna colors to send; skipping transmission")
                return

            self.get_logger().info(f"Payload: {payload.hex().upper()}")

            self.enqueue(payload)

        except Exception as e:
            self.get_logger().error(f"Parse error: {e}")

    # ───────────────── Encoding ─────────────────

    def parse_colors(self, text):
        """
        Accept exactly 4 comma-separated positions so antenna indices remain fixed.

        Examples:
          "RED,GREEN,,BLUE" -> ["RED", "GREEN", "", "BLUE"]
          ",,GREEN,"        -> ["", "", "GREEN", ""]
        """
        parts = [p.strip().upper() for p in text.split(",")]

        if len(parts) != 4:
            raise ValueError("Expected exactly 4 comma-separated antenna entries")

        return parts

    def colors_to_payload(self, colors):
        """
        Build a variable-length payload.
        Blank entries are skipped entirely.
        """
        payload = []

        for i, color in enumerate(colors):

            # Blank antenna entry -> skip, do NOT send 0x00 (results in improper transmission)
            if color == "":
                continue

            if color not in COLOR_NIBBLES:
                raise ValueError(f"Invalid color: {color}")

            byte = (ANTENNA_NIBBLES[i] << 4) | COLOR_NIBBLES[color]
            payload.append(byte)

        return bytes(payload)

    # ───────────────── Queue ─────────────────

    def enqueue(self, payload):

        if self._loop is None or self._queue is None:
            self.get_logger().warning("BLE loop not ready yet; dropping payload")
            return

        async def put():
            await self._queue.put(payload)

        asyncio.run_coroutine_threadsafe(put(), self._loop)

    # ───────────────── BLE thread ─────────────────

    def _ble_thread_main(self):

        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        self._queue = asyncio.Queue()

        self._loop.run_until_complete(self._ble_main())

    async def _ble_main(self):

        while not self._shutdown.is_set():

            if not self.is_connected():
                try:
                    await self.connect_ble()
                except Exception as e:
                    self.get_logger().error(f"Connect failed: {e}")
                    await asyncio.sleep(BLE_RECONNECT_DELAY)
                    continue

            try:
                payload = await asyncio.wait_for(self._queue.get(), timeout=0.5)
            except asyncio.TimeoutError:
                continue

            try:
                await self.write_payload(payload)
            except Exception as e:
                self.get_logger().error(f"Write failed: {e}")
                await self.disconnect_ble()

    # ───────────────── BLE helpers ─────────────────

    def is_connected(self):

        return (
            self._client is not None
            and self._client.is_connected
            and self._connected
        )

    async def connect_ble(self):

        # Try static MAC first
        if STATIC_MAC:

            self.get_logger().info(f"Trying MAC {STATIC_MAC}")

            try:
                client = BleakClient(STATIC_MAC, timeout=BLE_CONNECT_TIMEOUT)

                await client.connect()

                if client.is_connected:
                    self._client = client
                    self._connected = True
                    self.get_logger().info("Connected via MAC")
                    return

            except Exception:
                self.get_logger().warning("MAC connect failed")

        # Scan by name
        self.get_logger().info(f"Scanning for {DEVICE_NAME}")

        device = await BleakScanner.find_device_by_filter(
            lambda d, adv: d.name == DEVICE_NAME,
            timeout=BLE_SCAN_TIMEOUT
        )

        if device is None:
            raise BleakError("Device not found")

        self.get_logger().info(f"Found device {device.address}")

        self._client = BleakClient(device)

        await self._client.connect()

        if not self._client.is_connected:
            raise BleakError("Connection failed")

        self._connected = True
        self.get_logger().info("Connected to BLE device")

    async def write_payload(self, payload):

        if not self.is_connected():
            raise BleakError("Not connected")

        self.get_logger().info(f"Sending: {payload.hex().upper()}")

        await self._client.write_gatt_char(
            CHAR_UUID_CMD,
            payload,
            response=WRITE_WITH_RESPONSE
        )

        self.get_logger().info("Write complete")

    async def disconnect_ble(self):

        self._connected = False

        if self._client is not None:
            try:
                await self._client.disconnect()
            except Exception:
                pass

        self._client = None


# ───────────────── Entrypoint ─────────────────

def main(args=None):

    rclpy.init(args=args)

    node = SendIRNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node._shutdown.set()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()