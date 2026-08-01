#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_ORDER = [
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5"
]


STEP = np.deg2rad(2.0)
MOVE_TIME = 2.0


class SOAGradientController(Node):

    def __init__(self):

        super().__init__("soa_gradient_controller")

        ##########################################
        # State
        ##########################################

        self.q = None
        self.Jsoa = None
        self.soa = None

        self.goal_active = False

        ##########################################
        # Subscribers
        ##########################################

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.create_subscription(
            Float64,
            "/soa/value",
            self.soa_callback,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            "/soa/jacobian",
            self.jacobian_callback,
            10
        )

        ##########################################
        # Action Client
        ##########################################

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.get_logger().info("Waiting for trajectory server...")

        self.client.wait_for_server()

        self.get_logger().info("Trajectory server connected.")

        self.timer = self.create_timer(
            3.0,
            self.control_loop
        )

    def joint_callback(self, msg):

        joint_map = dict(zip(msg.name, msg.position))

        self.q = np.array([
            joint_map[name]
            for name in JOINT_ORDER
        ])

    def soa_callback(self, msg):

        self.soa = msg.data

    def jacobian_callback(self, msg):

        self.Jsoa = np.array(msg.data)

    def control_loop(self):

        if self.goal_active:
            return

        if self.q is None:
            return

        if self.Jsoa is None:
            return

        if self.soa is None:
            return

        # Choose best joint

        best_joint = np.argmax(np.abs(self.Jsoa))
        direction = np.sign(self.Jsoa[best_joint])

        if direction == 0:

            self.get_logger().info(
                "Gradient is zero."
            )

            return
        
        # Target


        q_target = self.q.copy()

        q_target[best_joint] += direction * STEP

        ##########################################

        self.get_logger().info(
            "\n"
            "==============================\n"
            f"Current SOA : {self.soa:.4f}\n"
            f"Moving Joint : {best_joint}\n"
            f"Gradient : {self.Jsoa[best_joint]:.6f}\n"
            f"Direction : {direction}\n"
            "=============================="
        )

        self.send_goal(q_target)

    ###########################################################

    def send_goal(self, q_target):

        self.goal_active = True

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = JOINT_ORDER

        point = JointTrajectoryPoint()

        point.positions = q_target.tolist()

        point.time_from_start.sec = int(MOVE_TIME)

        goal.trajectory.points.append(point)

        future = self.client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_response_callback
        )

    ###########################################################

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.goal_active = False

            self.get_logger().error(
                "Goal rejected."
            )

            return

        self.get_logger().info(
            "Goal accepted."
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.goal_finished_callback
        )

    ###########################################################

    def goal_finished_callback(self, future):

        self.goal_active = False

        self.get_logger().info(
            "Motion complete."
        )


###############################################################


def main(args=None):

    rclpy.init(args=args)

    node = SOAGradientController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()