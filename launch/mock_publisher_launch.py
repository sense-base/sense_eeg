from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="eeg_publisher",
                namespace="eeg_publisher1",
                executable="mock_publisher",
                name="mock_publisher",
            )
        ]
    )
