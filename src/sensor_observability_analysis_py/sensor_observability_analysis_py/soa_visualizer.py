import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64


class SOAVisualizerNode(Node):

    def __init__(self):

        super().__init__(
            "soa_visualizer_node"
        )

        self.create_subscription(
            Float64,
            "/soa/index",
            self.callback,
            10
        )

    def callback(self, msg):

        self.get_logger().info(
            f"SOA Index: {msg.data:.4f}"
        )


def main():

    rclpy.init()

    node = SOAVisualizerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":

    main()