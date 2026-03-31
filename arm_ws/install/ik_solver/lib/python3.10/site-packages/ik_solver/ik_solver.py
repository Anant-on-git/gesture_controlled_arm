import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


class IkSolverNode(Node):
    def __init__(self) -> None:
        super().__init__("ik_solver")

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("displacement_topic", "/controller/displacement")
        self.declare_parameter("joint_state_publish_rate", 10.0)
        self.declare_parameter("base_height", 0.05)
        self.declare_parameter("link1_length", 0.5)
        self.declare_parameter("link2_length", 0.4)
        self.declare_parameter("publish_initial_state", True)
        self.declare_parameter("initial_joint_positions", [0.0, 0.0, 0.0])

        self.joint_names: List[str] = ["joint_yaw", "joint_pitch", "joint_elbow"]
        self.joint_limits: Dict[str, Tuple[float, float]] = {
            "joint_yaw": (-3.14, 3.14),
            "joint_pitch": (-1.57, 1.57),
            "joint_elbow": (-1.57, 1.57),
        }

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.displacement_topic = str(self.get_parameter("displacement_topic").value)
        self.joint_state_publish_rate = float(
            self.get_parameter("joint_state_publish_rate").value
        )
        self.base_height = float(self.get_parameter("base_height").value)
        self.link1_length = float(self.get_parameter("link1_length").value)
        self.link2_length = float(self.get_parameter("link2_length").value)

        initial_positions = list(
            self.get_parameter("initial_joint_positions").get_parameter_value().double_array_value
        )
        if len(initial_positions) != len(self.joint_names):
            initial_positions = [0.0, 0.0, 0.0]

        self.current_joint_positions: Dict[str, float] = dict(
            zip(self.joint_names, initial_positions)
        )
        self.has_external_joint_state = False

        self.joint_state_publisher = self.create_publisher(
            JointState,
            self.joint_state_topic,
            10,
        )
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )
        self.create_subscription(
            Vector3Stamped,
            self.displacement_topic,
            self.displacement_callback,
            10,
        )

        if self.joint_state_publish_rate > 0.0:
            self.create_timer(
                1.0 / self.joint_state_publish_rate,
                self.publish_current_joint_state,
            )

        if bool(self.get_parameter("publish_initial_state").value):
            self.publish_current_joint_state()

        self.get_logger().info(
            "IK solver ready: joint_state_topic=%s displacement_topic=%s "
            "joint_state_publish_rate=%.2fHz base_height=%.3f link1=%.3f link2=%.3f"
            % (
                self.joint_state_topic,
                self.displacement_topic,
                self.joint_state_publish_rate,
                self.base_height,
                self.link1_length,
                self.link2_length,
            )
        )

    def joint_state_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        updated = False
        for joint_name, position in zip(msg.name, msg.position):
            if joint_name in self.current_joint_positions:
                self.current_joint_positions[joint_name] = position
                updated = True

        if updated:
            self.has_external_joint_state = True

    def forward_kinematics(
        self, yaw: float, shoulder: float, elbow: float
    ) -> Tuple[float, float, float]:
        reach = (
            self.link1_length * math.cos(shoulder)
            + self.link2_length * math.cos(shoulder + elbow)
        )
        x = reach * math.cos(yaw)
        y = reach * math.sin(yaw)
        z = (
            self.base_height
            + self.link1_length * math.sin(shoulder)
            + self.link2_length * math.sin(shoulder + elbow)
        )
        return x, y, z

    def inverse_kinematics(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        current_angles: Tuple[float, float, float],
    ) -> Optional[Tuple[float, float, float]]:
        yaw = math.atan2(target_y, target_x)
        planar_reach = math.hypot(target_x, target_y)
        vertical_offset = target_z - self.base_height

        cos_elbow = (
            planar_reach * planar_reach
            + vertical_offset * vertical_offset
            - self.link1_length * self.link1_length
            - self.link2_length * self.link2_length
        ) / (2.0 * self.link1_length * self.link2_length)

        if cos_elbow < -1.0 - 1e-6 or cos_elbow > 1.0 + 1e-6:
            return None

        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        sin_elbow_mag = math.sqrt(max(0.0, 1.0 - (cos_elbow * cos_elbow)))

        elbow_candidates = [
            math.atan2(sin_elbow_mag, cos_elbow),
            math.atan2(-sin_elbow_mag, cos_elbow),
        ]

        valid_solutions: List[Tuple[float, float, float]] = []
        for elbow in elbow_candidates:
            shoulder = math.atan2(vertical_offset, planar_reach) - math.atan2(
                self.link2_length * math.sin(elbow),
                self.link1_length + self.link2_length * math.cos(elbow),
            )
            candidate = (yaw, shoulder, elbow)
            if self.solution_within_limits(candidate):
                valid_solutions.append(candidate)

        if not valid_solutions:
            return None

        return min(valid_solutions, key=lambda angles: self.solution_distance(angles, current_angles))

    def solution_within_limits(self, angles: Tuple[float, float, float]) -> bool:
        for joint_name, angle in zip(self.joint_names, angles):
            lower, upper = self.joint_limits[joint_name]
            if angle < lower or angle > upper:
                return False
        return True

    def solution_distance(
        self,
        target_angles: Tuple[float, float, float],
        current_angles: Tuple[float, float, float],
    ) -> float:
        return sum(
            abs(target_angle - current_angle)
            for target_angle, current_angle in zip(target_angles, current_angles)
        )

    def publish_joint_state(self, positions: List[float]) -> None:
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = list(self.joint_names)
        joint_state_msg.position = list(positions)
        self.joint_state_publisher.publish(joint_state_msg)

    def publish_current_joint_state(self) -> None:
        self.publish_joint_state(
            [self.current_joint_positions[name] for name in self.joint_names]
        )

    def displacement_callback(self, msg: Vector3Stamped) -> None:
        current_angles = tuple(
            self.current_joint_positions[name] for name in self.joint_names
        )
        current_x, current_y, current_z = self.forward_kinematics(*current_angles)

        target_x = current_x + msg.vector.x
        target_y = current_y + msg.vector.y
        target_z = current_z + msg.vector.z

        solution = self.inverse_kinematics(
            target_x,
            target_y,
            target_z,
            current_angles,
        )
        if solution is None:
            self.get_logger().warn(
                "IK target unreachable or violates joint limits: "
                "current=(%.3f, %.3f, %.3f) displacement=(%.3f, %.3f, %.3f) "
                "target=(%.3f, %.3f, %.3f)"
                % (
                    current_x,
                    current_y,
                    current_z,
                    msg.vector.x,
                    msg.vector.y,
                    msg.vector.z,
                    target_x,
                    target_y,
                    target_z,
                ),
                throttle_duration_sec=1.0,
            )
            return

        for joint_name, angle in zip(self.joint_names, solution):
            self.current_joint_positions[joint_name] = angle

        self.publish_current_joint_state()

        self.get_logger().info(
            "Applied displacement=(%.3f, %.3f, %.3f) current_xyz=(%.3f, %.3f, %.3f) "
            "target_xyz=(%.3f, %.3f, %.3f) joints=(%.3f, %.3f, %.3f)"
            % (
                msg.vector.x,
                msg.vector.y,
                msg.vector.z,
                current_x,
                current_y,
                current_z,
                target_x,
                target_y,
                target_z,
                solution[0],
                solution[1],
                solution[2],
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IkSolverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
