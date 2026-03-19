import json
from typing import Optional

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
from serial import SerialException


class ImuReaderNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_reader")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("poll_period", 0.01)

        self._port = str(self.get_parameter("port").value)
        self._baud_rate = int(self.get_parameter("baud_rate").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._poll_period = float(self.get_parameter("poll_period").value)

        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()
        self._port_open_logged = False

        self._imu_publisher = self.create_publisher(Imu, "imu/data_raw", 10)
        self._accel_publisher = self.create_publisher(
            Vector3Stamped,
            "imu/accel",
            10,
        )
        self._gyro_publisher = self.create_publisher(
            Vector3Stamped,
            "imu/gyro",
            10,
        )

        self.create_timer(self._poll_period, self._poll_serial)
        self.get_logger().info(
            f"Reading IMU JSON from {self._port} at {self._baud_rate} baud"
        )

    def _open_serial_if_needed(self) -> bool:
        if self._serial is not None and self._serial.is_open:
            return True

        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud_rate,
                timeout=0.0,
            )
            self._buffer.clear()
            if not self._port_open_logged:
                self.get_logger().info(f"Connected to serial port {self._port}")
                self._port_open_logged = True
            return True
        except SerialException as exc:
            self._serial = None
            self._port_open_logged = False
            self.get_logger().warn(
                f"Waiting for serial port {self._port}: {exc}",
                throttle_duration_sec=5.0,
            )
            return False

    def _poll_serial(self) -> None:
        if not self._open_serial_if_needed():
            return

        assert self._serial is not None

        try:
            available_bytes = self._serial.in_waiting
            if available_bytes <= 0:
                return

            chunk = self._serial.read(available_bytes)
        except SerialException as exc:
            self.get_logger().error(f"Serial read failed: {exc}")
            self._close_serial()
            return

        if not chunk:
            return

        self._buffer.extend(chunk)

        while b"\n" in self._buffer:
            line, _, remainder = self._buffer.partition(b"\n")
            self._buffer = bytearray(remainder)

            if not line:
                continue

            self._handle_packet(line.decode("utf-8", errors="replace").strip())

    def _handle_packet(self, packet: str) -> None:
        try:
            payload = json.loads(packet)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"Ignoring malformed JSON packet: {packet}",
                throttle_duration_sec=2.0,
            )
            return

        required_fields = (
            "accel_x",
            "accel_y",
            "accel_z",
            "gyro_x",
            "gyro_y",
            "gyro_z",
        )
        if not all(field in payload for field in required_fields):
            self.get_logger().warn(
                f"Ignoring incomplete IMU packet: {packet}",
                throttle_duration_sec=2.0,
            )
            return

        try:
            accel_x = float(payload["accel_x"])
            accel_y = float(payload["accel_y"])
            accel_z = float(payload["accel_z"])
            gyro_x = float(payload["gyro_x"])
            gyro_y = float(payload["gyro_y"])
            gyro_z = float(payload["gyro_z"])
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"Ignoring non-numeric IMU packet: {packet}",
                throttle_duration_sec=2.0,
            )
            return

        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self._frame_id
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.orientation_covariance[0] = -1.0
        self._imu_publisher.publish(imu_msg)

        accel_msg = Vector3Stamped()
        accel_msg.header.stamp = stamp
        accel_msg.header.frame_id = self._frame_id
        accel_msg.vector.x = accel_x
        accel_msg.vector.y = accel_y
        accel_msg.vector.z = accel_z
        self._accel_publisher.publish(accel_msg)

        gyro_msg = Vector3Stamped()
        gyro_msg.header.stamp = stamp
        gyro_msg.header.frame_id = self._frame_id
        gyro_msg.vector.x = gyro_x
        gyro_msg.vector.y = gyro_y
        gyro_msg.vector.z = gyro_z
        self._gyro_publisher.publish(gyro_msg)

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except SerialException:
                pass
        self._serial = None
        self._port_open_logged = False

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
