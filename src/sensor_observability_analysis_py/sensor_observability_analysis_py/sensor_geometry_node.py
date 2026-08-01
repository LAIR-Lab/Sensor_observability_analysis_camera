import json
import numpy as np

import rclpy

from rclpy.node import Node

from std_msgs.msg import String

from tf2_ros import Buffer
from tf2_ros import TransformListener

from scipy.spatial.transform import Rotation


class SensorGeometryNode(Node):

    def __init__(self):

        super().__init__(
            "sensor_geometry_node"
        )

        self.publisher = (
            self.create_publisher(
                String,
                "/soa/sensor_geometry",
                10
            )
        )

        self.tf_buffer = Buffer()

        self.tf_listener = (
            TransformListener(
                self.tf_buffer,
                self
            )
        )

        self.create_timer(
            0.1,
            self.timer_callback
        )

    def timer_callback(self):

        try:

            tf = (
                self.tf_buffer.lookup_transform(
                    "under_arm",
                    "upper_arm",
                    rclpy.time.Time()
                )
            )

        except Exception:
            return

        r = [

            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z

        ]

        q = tf.transform.rotation

        R = Rotation.from_quat([

            q.x,
            q.y,
            q.z,
            q.w

        ]).as_matrix()

        msg = String()

        msg.data = json.dumps({

            "position": r,

            "rotation": R.tolist()

        })

        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = SensorGeometryNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":

    main()