#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Header
from eeg_msgs.msg import EEGBlock


class MockEEGPublisher(Node):
    def __init__(self):
        super().__init__('mock_eeg_publisher')
        self.n_seed = 0
        np.random.seed(self.n_seed)
        self.queue_size = 10
        self.num_channels = 8
        self.num_samples = 32
        self.sampling_rate = 256.0
        self.timer_period_in_secs = 1 / (self.sampling_rate/self.num_samples)
        self.publisher = self.create_publisher(EEGBlock, '/eeg/raw', self.queue_size)
        self.timer = self.create_timer(self.timer_period_in_secs, self.publish_data)
        self.get_logger().info(
            f'Mock EEG Publisher running at {self.sampling_rate} Hz '
            f'with {self.num_samples} samples and '
            f'timer period of {self.timer_period_in_secs} seconds'
        )


    def publish_data(self):
        msg = EEGBlock()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.num_channels = self.num_channels
        msg.num_samples = self.num_samples
        msg.sampling_rate = self.sampling_rate
        msg.data = np.random.randn(msg.num_channels * msg.num_samples).astype(np.float32).tolist()
        self.publisher.publish(msg)
        self.get_logger().info(f'Published EEGBlock: {len(msg.data)} samples')


def main(args=None):
    rclpy.init(args=args)
    node = MockEEGPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
