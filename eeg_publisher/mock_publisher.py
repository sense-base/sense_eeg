import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Timer
import numpy as np
from eeg_msgs.msg import EEGBlock
from std_msgs.msg import Header
from eeg_bridge.bridge import EEGBridge
from numpy.random import default_rng

from typing import Optional


class MockEEGPublisher(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("mock_eeg_publisher")
        self.bridge = EEGBridge()

        # logger parameters
        self.throttle_duration: float = 1.0
        self.once: bool = False

        self.n_seed: int = 0
        self.queue_size: int = 10
        self.num_channels: int = 8
        self.num_samples: int = 32
        self.sampling_rate: float = 256.0

        self.rng = default_rng(self.n_seed)

        self.timer_period_in_secs: float = 1 / (self.sampling_rate / self.num_samples)

        self.publisher: Publisher = self.create_publisher(
            EEGBlock, "/eeg/raw", self.queue_size
        )

        self.timer: Timer = self.create_timer(
            self.timer_period_in_secs, self.publish_data
        )

        self.get_logger().info(
            f"Mock EEG Publisher running at {self.sampling_rate} Hz "
            f"with {self.num_samples} samples and "
            f"timer period of {self.timer_period_in_secs} seconds"
        )

    def publish_data(self) -> None:
        eeg_array = self.rng.standard_normal(
            (self.num_channels, self.num_samples)
        ).astype(np.float32)

        # Create a Header with the current ROS time
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "eeg"

        # Use the bridge to convert to EEGBlock
        msg = self.bridge.numpy_to_eegblock(
            eeg_array,
            sampling_rate=self.sampling_rate,
            header=header,
        )

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published EEGBlock: {eeg_array.shape} → {len(msg.data)} values",
            once=self.once,
            throttle_duration_sec=self.throttle_duration,
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MockEEGPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
