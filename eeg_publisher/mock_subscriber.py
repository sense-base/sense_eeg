import rclpy
from rclpy.node import Node
from eeg_msgs.msg import EEGBlock
from eeg_bridge.bridge import EEGBridge


class MockEEGSubscriber(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("mock_eeg_subscriber")

        self.subscription = self.create_subscription(
            EEGBlock, "/eeg/raw", self.listener_callback, 10
        )
        self.bridge = EEGBridge()
        self.get_logger().info("Mock EEG Subscriber started, listening on /eeg/raw")

    def listener_callback(self, msg: EEGBlock) -> None:
        try:
            eeg_array = self.bridge.eegblock_to_numpy(msg)
            self.get_logger().info(f"Received EEGBlock: shape {eeg_array.shape}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse EEGBlock: {e}")


def main() -> None:
    rclpy.init()
    node = MockEEGSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
