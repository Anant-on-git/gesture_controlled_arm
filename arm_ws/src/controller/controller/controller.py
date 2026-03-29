from collections import deque

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller")

        self.declare_parameter("moving_average_window", 5)
        window_size = max(1, self.get_parameter("moving_average_window").value)
        self.accel_window = deque(maxlen=window_size)

        self.create_subscription(
            Vector3Stamped,
            "/imu/accel",
            self.accel_callback,
            10,
        )

        self.get_logger().info(
            "Using accel moving average window: %d" % self.accel_window.maxlen
        )

    def accel_callback(self, msg: Vector3Stamped) -> None:
        self.accel_window.append((msg.vector.x, msg.vector.y, msg.vector.z))

        sample_count = len(self.accel_window)
        avg_x = sum(sample[0] for sample in self.accel_window) / sample_count
        avg_y = sum(sample[1] for sample in self.accel_window) / sample_count
        avg_z = sum(sample[2] for sample in self.accel_window) / sample_count

        self.get_logger().info(
            "Accel moving avg (%d): x=%.3f y=%.3f z=%.3f"
            % (sample_count, avg_x, avg_y, avg_z)
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
