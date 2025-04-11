#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Header
from eeg_msgs.msg import EEGBlock


class MockEEGPublisher(Node):
    def __init__(self):
        super().__init__("mock_eeg_publisher")
        self.publisher = self.create_publisher(EEGBlock, "/eeg/raw", 10)
        self.timer = self.create_timer(
            0.125, self.publish_data
        )  # 8Hz (32 samples at 256Hz)
        self.get_logger().info("Mock EEG Publisher running...")

    def publish_data(self):
        msg = EEGBlock()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.num_channels = 8
        msg.num_samples = 32
        msg.sampling_rate = 256.0
        msg.data = (
            np.random.randn(msg.num_channels * msg.num_samples)
            .astype(np.float32)
            .tolist()
        )
        self.publisher.publish(msg)
        self.get_logger().info(f"Published EEGBlock: {len(msg.data)} samples")


def main(args=None):
    rclpy.init(args=args)
    node = MockEEGPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
