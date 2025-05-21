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
        self.declare_parameter('num_channels', 8)
        self.declare_parameter('num_samples', 32)
        self.declare_parameter('sampling_rate', 256.0)

        self.rng = default_rng(self.n_seed)

        sample_rate = self.get_parameter('sampling_rate')
        n_samples = self.get_parameter('num_samples')

        timer_period_in_secs: float = 1 / (sample_rate / n_samples)

        self.publisher: Publisher = self.create_publisher(
            EEGBlock, "/eeg/raw", self.queue_size
        )
        
        # this might be problematic if we change sample_rate or n_samples on the 
        # fly, as the timer period won't be recalculated
        self.timer: Timer = self.create_timer(
            timer_period_in_secs, self.publish_data
        )

        self.get_logger().info(
            f"Mock EEG Publisher running at {sample_rate} Hz "
            f"with {n_samples} samples and "
            f"timer period of {timer_period_in_secs} seconds"
        )

    def publish_data(self) -> None:
        n_channels = self.get_parameter('num_channels').get_parameter_value().integer_value
        sample_rate = self.get_parameter('sampling_rate').get_parameter_value().float_value
        n_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        msg = EEGBlock()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.num_channels = n_channels
        msg.num_samples = n_samples
        msg.sampling_rate = sample_rate
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
