import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller")
        self.create_subscription(
            Vector3Stamped,
            "/imu/accel",
            self.accel_callback,
            10,
        )

    def accel_callback(self, msg: Vector3Stamped) -> None:
        self.get_logger().info(
            "Accel: x=%.3f y=%.3f z=%.3f"
            % (msg.vector.x, msg.vector.y, msg.vector.z)
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
