from launch import LaunchDescription  # type: ignore
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="eeg_publisher",
                namespace="eeg_publisher1",
                executable="mock_publisher",
                name="mock_publisher",
                parameters=[PathJoinSubstitution([
                FindPackageShare('eeg_publisher'), 'config', 'mock_publisher.yaml'])
                ],
            )
        ]
    )
