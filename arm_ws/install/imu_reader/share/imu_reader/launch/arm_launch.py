from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open('/home/anant/gesture_controlled_arm/arm_ws/model/arm.urdf').read()
            }]
        )
    ])
