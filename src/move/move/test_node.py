#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class GaussianFilter(Node):

    def __init__(self):
        super().__init__("gaussian_filter")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/camera3/image_raw",
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            "/camera3/image_gaussian",
            10
        )

        self.get_logger().info("Gaussian Filter Started")

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        h, w = image.shape[:2]

        cx = w / 2
        cy = h / 2

        sigma = min(h, w) / 4

        # Coordinate grids
        x = np.arange(w)
        y = np.arange(h)

        X, Y = np.meshgrid(x, y)

        gaussian = np.exp(
            -((X - cx) ** 2 + (Y - cy) ** 2) /
            (2 * sigma ** 2)
        )

        gaussian = gaussian.astype(np.float32)

        # Apply mask to each channel
        output = image.astype(np.float32)

        output[:, :, 0] *= gaussian
        output[:, :, 1] *= gaussian
        output[:, :, 2] *= gaussian

        output = np.clip(output, 0, 255).astype(np.uint8)
        J = np.mean(output)

        self.get_logger().info(
            f"Objective = {100 * J / 255:.5f} %"
        )

        out_msg = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")

        out_msg.header = msg.header

        self.publisher.publish(out_msg)
        cv2.imshow("Gaussian Filtered Image", output)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = GaussianFilter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()