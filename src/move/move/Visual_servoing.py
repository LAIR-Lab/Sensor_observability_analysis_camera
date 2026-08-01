from rclpy.node import Node
from std_msgs.msg import Float64
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class VisualServoingNode(Node):
    def __init__(self):

        super().__init__(
            "visual_servoing_node"
        )

        self.bridge = CvBridge()

        self.sub1 = self.create_subscription(
            Image,
            "/camera1/image_raw",
            lambda msg: self.image_callback(msg, cam_id=1),
            10
        )

        self.sub2 = self.create_subscription(
            Image,
            "/camera2/image_raw",
            lambda msg: self.image_callback(msg, cam_id=2),
            10
        )

        self.sub3 = self.create_subscription(
            Image,
            "/camera3/image_raw",
            lambda msg: self.image_callback(msg, cam_id=3),
            10
        )

    def image_callback(self, msg, cam_id):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]

    