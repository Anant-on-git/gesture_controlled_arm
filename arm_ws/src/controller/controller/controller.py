from collections import deque
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller")

        self.declare_parameter("moving_average_window", 5)
        self.declare_parameter("gravity_alpha", 0.02)
        self.declare_parameter("motion_engage_threshold", 1.2)
        self.declare_parameter("motion_release_threshold", 0.6)
        self.declare_parameter("max_motion_accel", 4.0)
        self.declare_parameter("motion_ramp_gain", 1.5)

        window_size = max(1, int(self.get_parameter("moving_average_window").value))
        self.accel_window = deque(maxlen=window_size)

        # Lightweight motion control pipeline.
        # 1. Smooth raw acceleration with a short moving average.
        # 2. Estimate gravity with a slow exponential average:
        #      g_k = (1 - alpha) * g_(k-1) + alpha * a_avg
        # 3. Remove gravity to get linear acceleration:
        #      a_lin = a_avg - g_k
        # 4. Convert motion into a normalized velocity-style command.
        self.gravity_alpha = float(self.get_parameter("gravity_alpha").value)
        self.motion_engage_threshold = float(
            self.get_parameter("motion_engage_threshold").value
        )
        self.motion_release_threshold = float(
            self.get_parameter("motion_release_threshold").value
        )
        self.max_motion_accel = float(self.get_parameter("max_motion_accel").value)
        self.motion_ramp_gain = float(self.get_parameter("motion_ramp_gain").value)

        if self.motion_release_threshold > self.motion_engage_threshold:
            self.get_logger().warn(
                "motion_release_threshold was larger than motion_engage_threshold; "
                "clamping release threshold to match engage threshold."
            )
            self.motion_release_threshold = self.motion_engage_threshold

        self.current_motion_direction: Dict[str, int] = {"x": 0, "y": 0, "z": 0}
        self.axis_active_time: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.current_velocity_command: Dict[str, float] = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        }
        self.gravity_estimate: Optional[Tuple[float, float, float]] = None
        self.last_sample_time: Optional[float] = None

        self.create_subscription(
            Vector3Stamped,
            "/imu/accel",
            self.accel_callback,
            10,
        )

        self.get_logger().info(
            "Controller ready: avg_window=%d gravity_alpha=%.3f motion_engage=%.2f "
            "motion_release=%.2f max_motion_accel=%.2f motion_ramp_gain=%.2f"
            % (
                self.accel_window.maxlen,
                self.gravity_alpha,
                self.motion_engage_threshold,
                self.motion_release_threshold,
                self.max_motion_accel,
                self.motion_ramp_gain,
            )
        )

    def update_gravity_estimate(
        self, avg_accel: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        if self.gravity_estimate is None:
            self.gravity_estimate = avg_accel
            return self.gravity_estimate

        gx, gy, gz = self.gravity_estimate
        ax, ay, az = avg_accel
        alpha = self.gravity_alpha
        self.gravity_estimate = (
            (1.0 - alpha) * gx + alpha * ax,
            (1.0 - alpha) * gy + alpha * ay,
            (1.0 - alpha) * gz + alpha * az,
        )
        return self.gravity_estimate

    def compute_sample_dt(self, msg: Vector3Stamped) -> float:
        stamp = msg.header.stamp
        stamp_seconds = float(stamp.sec) + (float(stamp.nanosec) * 1e-9)
        if stamp_seconds <= 0.0:
            stamp_seconds = self.get_clock().now().nanoseconds * 1e-9

        if self.last_sample_time is None:
            self.last_sample_time = stamp_seconds
            return 0.0

        dt = max(0.0, min(0.1, stamp_seconds - self.last_sample_time))
        self.last_sample_time = stamp_seconds
        return dt

    def axis_command(self, axis_name: str, linear_accel: float, dt: float) -> float:
        active_direction = self.current_motion_direction[axis_name]
        magnitude = abs(linear_accel)

        if magnitude >= self.motion_engage_threshold:
            direction = 1 if linear_accel > 0.0 else -1
        elif magnitude <= self.motion_release_threshold:
            direction = 0
        else:
            direction = active_direction

        self.current_motion_direction[axis_name] = direction

        if direction == 0:
            self.axis_active_time[axis_name] = 0.0
            self.current_velocity_command[axis_name] = 0.0
            return 0.0

        self.axis_active_time[axis_name] += dt
        normalized = min(1.0, magnitude / self.max_motion_accel)
        ramp = min(1.0, self.axis_active_time[axis_name] * self.motion_ramp_gain)
        command = direction * normalized * ramp
        self.current_velocity_command[axis_name] = command
        return command

    def accel_callback(self, msg: Vector3Stamped) -> None:
        dt = self.compute_sample_dt(msg)
        self.accel_window.append((msg.vector.x, msg.vector.y, msg.vector.z))

        sample_count = len(self.accel_window)
        avg_x = sum(sample[0] for sample in self.accel_window) / sample_count
        avg_y = sum(sample[1] for sample in self.accel_window) / sample_count
        avg_z = sum(sample[2] for sample in self.accel_window) / sample_count

        avg_accel = (avg_x, avg_y, avg_z)
        gravity_x, gravity_y, gravity_z = self.update_gravity_estimate(avg_accel)
        lin_x = avg_x - gravity_x
        lin_y = avg_y - gravity_y
        lin_z = avg_z - gravity_z

        cmd_x = self.axis_command("x", lin_x, dt)
        cmd_y = self.axis_command("y", lin_y, dt)
        cmd_z = self.axis_command("z", lin_z, dt)

        self.get_logger().info(
            "avg=(%.3f, %.3f, %.3f) lin=(%.3f, %.3f, %.3f) cmd=(%.2f, %.2f, %.2f) "
            "dir=(%d, %d, %d) held=(%.2f, %.2f, %.2f)s"
            % (
                avg_x,
                avg_y,
                avg_z,
                lin_x,
                lin_y,
                lin_z,
                cmd_x,
                cmd_y,
                cmd_z,
                self.current_motion_direction["x"],
                self.current_motion_direction["y"],
                self.current_motion_direction["z"],
                self.axis_active_time["x"],
                self.axis_active_time["y"],
                self.axis_active_time["z"],
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
