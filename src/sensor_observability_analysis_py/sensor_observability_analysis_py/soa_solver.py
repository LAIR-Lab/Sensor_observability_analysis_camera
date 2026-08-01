import json
import numpy as np

import rclpy

from rclpy.node import Node

from std_msgs.msg import (
    String,
    Float64
)

from sensor_observability_analysis_py.soa_math import (
    build_observability_matrix,
    observability_index,
    observability_index_p_norm,
    observability_index_max
)


class SOASolverNode(Node):

    def __init__(self):

        super().__init__(
            "soa_solver_node"
        )

        self.publisher = (
            self.create_publisher(
                Float64,
                "/soa/index",
                10
            )
        )

        self.create_subscription(
            String,
            "/soa/sensor_geometry",
            self.callback,
            10
        )

    def callback(self, msg):

        data = json.loads(
            msg.data
        )

        r = np.array(
            data["position"]
        )

        R = np.array(
            data["rotation"]
        )

        S = build_observability_matrix(
            r,
            R
        )

        index = (
            observability_index(S)
        )

        index_p_norm = (
            observability_index_p_norm(S)
        )

        index_max = (
            observability_index_max(S)
        )

        out = Float64()

        out.data = index

        self.publisher.publish(
            out
        )


def main():

    rclpy.init()

    node = SOASolverNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":

    main()