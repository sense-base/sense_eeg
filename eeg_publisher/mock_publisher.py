import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Timer
import numpy as np
from std_msgs.msg import Header
from eeg_msgs.msg import EEGBlock
from numpy.random import default_rng

from typing import Optional


class MockEEGPublisher(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("mock_eeg_publisher")
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
        msg = EEGBlock()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.num_channels = self.num_channels
        msg.num_samples = self.num_samples
        msg.sampling_rate = self.sampling_rate
        msg.data = (
            self.rng.standard_normal(msg.num_channels * msg.num_samples)
            .astype(np.float32)
            .tolist()
        )
        self.publisher.publish(msg)
        self.get_logger().info(f"Published EEGBlock: {len(msg.data)} samples")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MockEEGPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
