from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = Path("/home/anant/gesture_controlled_arm/arm_ws/model/arm.urdf")

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': urdf_path.read_text()
            }]
        ),
        Node(
            package='imu_reader',
            executable='imu_reader',
            output='screen',
        ),
        Node(
            package='controller',
            executable='controller',
            output='screen',
        ),
        Node(
            package='ik_solver',
            executable='ik_solver',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen',
        )
    ])
