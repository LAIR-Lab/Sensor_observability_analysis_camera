#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Joints RTB's chain to camera_3 actually uses (affects J_SOA / q length).
ACTIVE_JOINTS = [
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4"
]

CONTROLLER_JOINTS = [
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5"
]

JOINT_LOWER_LIMITS = np.array([-2.243, -1.418, -1.927, -1.168, -2.0])
JOINT_UPPER_LIMITS = np.array([1.827, 2.08, 1.147, 2.0, 2.4])


class SOAGradientAscent(Node):

    def __init__(self):

        super().__init__("soa_gradient_ascent")
        self.step = np.deg2rad(4.0)          # was 5.0 -- still bigger than the original 3.5, less aggressive
        self.base_step = self.step          # ceiling the step can regrow to
        self.min_step = np.deg2rad(0.1)
        self.step_shrink = 0.5
        self.step_grow = 1.35               # was 1.2 -- recover from a shrink faster

        self.improvement_tol = 1e-4
        self.stall_patience = 5
        self.stall_count = 0

        self.tracking_mode = True

        self.tie_ratio = 0.85  # was 0.8 -- slightly stricter, 2-joint moves are the exception not the default

        self.max_iterations = 2_000_000
        self.max_rejections = 5

  
        # first number to dial back down.
        self.max_joint_speed = np.deg2rad(400.0)  # rad/s -- was 550, pulled back for safety margin
        self.min_traj_time = 0.02                 # seconds, floor -- was 0.01

        self.q = None
        self.soa = None
        self.jsoa = None

        self.full_positions = {}

        self.last_soa = -1e9
        self.pending_soa = None

     
        self.joint_flip = np.ones(len(ACTIVE_JOINTS))
        self.last_moved_joint = None
        self.pending_moved_joint = None

        self.iteration = 0
        self.rejections = 0
        self.goal_active = False
        self.finished = False
        self._settle_timer = None
        self._server_confirmed = False

        ############################################
        # Subscribers
        ############################################

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

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

   
        self.timer = self.create_timer(
            0.1,
            self.optimize
        )

    def joint_callback(self, msg):

        joint_map = dict(zip(msg.name, msg.position))

        self.full_positions.update(joint_map)

        if all(n in joint_map for n in ACTIVE_JOINTS):
            self.q = np.array([
                joint_map[n]
                for n in ACTIVE_JOINTS
            ])

    def soa_callback(self, msg):

        self.soa = msg.data

    def jacobian_callback(self, msg):

        data = np.array(msg.data)

        if data.shape[0] == len(ACTIVE_JOINTS):
            self.jsoa = data

    def optimize(self):

        if self.finished or self.goal_active:
            return

        if self.q is None or self.soa is None or self.jsoa is None:
            return

        grad_abs = np.abs(self.jsoa)
        max_grad = np.max(grad_abs)

        if max_grad < 1e-8:

            self.get_logger().info(
                "Zero gradient this tick -- skipping, will retry next tick."
            )
            return

        dominant_joint = int(np.argmax(grad_abs))

        sorted_idx = np.argsort(grad_abs)[::-1]
        runner_up = int(sorted_idx[1]) if len(ACTIVE_JOINTS) > 1 else None

        if runner_up is not None and grad_abs[runner_up] >= self.tie_ratio * max_grad:
            top_idx = np.array([dominant_joint, runner_up])
        else:
            top_idx = np.array([dominant_joint])

        weights = grad_abs[top_idx] / max_grad

        in_view = abs(self.soa) > 1e-9

        if self.tracking_mode:
            self.stall_count = 0

        elif in_view and self.iteration > 0:

            improvement = self.soa - self.last_soa
            if abs(improvement) < self.improvement_tol:
                self.stall_count += 1
            else:
                self.stall_count = 0

            if improvement < -self.improvement_tol:
                if self.last_moved_joint is not None:
                    self.joint_flip[self.last_moved_joint] *= -1
                    self.get_logger().info(
                        f"S dropped {-improvement:.4f} after moving "
                        f"{ACTIVE_JOINTS[self.last_moved_joint]}; flipping its "
                        f"direction (now {'+' if self.joint_flip[self.last_moved_joint] > 0 else '-'})."
                    )

                old_step = self.step
                self.step = max(self.min_step, self.step * self.step_shrink)

                if self.step != old_step:
                    self.get_logger().info(
                        f"Overshoot (S dropped {-improvement:.4f}); "
                        f"step {np.rad2deg(old_step):.2f} -> {np.rad2deg(self.step):.2f} deg"
                    )

            elif improvement > self.improvement_tol:
                old_step = self.step
                self.step = min(self.base_step, self.step * self.step_grow)

                if self.step != old_step:
                    self.get_logger().info(
                        f"Climbing well (S +{improvement:.4f}); "
                        f"step {np.rad2deg(old_step):.2f} -> {np.rad2deg(self.step):.2f} deg"
                    )

            # (No stall/kick response here anymore -- if it's flat, it
            # just keeps trying with whatever step it currently has.)

        ############################################
        # New joint target
        ############################################

        q_target = self.q.copy()

        for i, joint in enumerate(top_idx):
            direction = self.joint_flip[joint] * np.sign(self.jsoa[joint])

            q_target[joint] += (
                direction *
                self.step *
                weights[i]
            )

        q_target = np.clip(q_target, JOINT_LOWER_LIMITS, JOINT_UPPER_LIMITS)
        
        self.get_logger().info(
            f"""
====================================
Iteration : {self.iteration}
SOA : {self.soa:.4f}
Step : {np.rad2deg(self.step):.2f} deg
Dominant joint : {ACTIVE_JOINTS[dominant_joint]}
Moved joints : {[ACTIVE_JOINTS[j] for j in top_idx]}
Weights : {np.round(weights, 3).tolist()}
Target : {q_target}
====================================
"""
        )

        self.pending_soa = self.soa
        self.pending_moved_joint = dominant_joint
        self.send_goal(q_target)

    def send_goal(self, q_active):

        if not self._server_confirmed:
            if not self.client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("Trajectory server unavailable.")
                return
            self._server_confirmed = True

        active_map = dict(zip(ACTIVE_JOINTS, q_active.tolist()))

        positions = []

        for name in CONTROLLER_JOINTS:

            if name in active_map:
                positions.append(active_map[name])
            elif name in self.full_positions:
                positions.append(self.full_positions[name])
            else:
                self.get_logger().error(
                    f"No known position for '{name}', aborting goal."
                )
                return

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = CONTROLLER_JOINTS

        point = JointTrajectoryPoint()

        point.positions = positions

        max_delta = 0.0
        for name, target_pos in zip(CONTROLLER_JOINTS, positions):
            current_pos = self.full_positions.get(name, target_pos)
            max_delta = max(max_delta, abs(target_pos - current_pos))

        traj_time = max(self.min_traj_time, max_delta / self.max_joint_speed)
        sec = int(traj_time)
        nanosec = int((traj_time - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

        goal.trajectory.points.append(point)

        self.goal_active = True

        future = self.client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.goal_active = False
            self.rejections += 1

            self.get_logger().warn(
                f"Goal rejected ({self.rejections}/{self.max_rejections})."
            )

            if self.rejections >= self.max_rejections:
                self.get_logger().error(
                    f"{self.max_rejections} consecutive goal rejections -- "
                    f"controller config likely needs attention, but continuing "
                    f"to retry rather than stopping."
                )
                self.rejections = 0

            return

        self.rejections = 0
        self.last_soa = self.pending_soa
        self.last_moved_joint = self.pending_moved_joint
        self.iteration += 1

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.goal_finished
        )

    def goal_finished(self, future):

        self.goal_active = False

        result = future.result().result

        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(
                f"Trajectory finished with error_code="
                f"{result.error_code} ({result.error_string}), "
                f"not SUCCESSFUL."
            )

        if not self.finished:
            self._settle_timer = self.create_timer(0.05, self._settle_and_optimize)

    def _settle_and_optimize(self):
        self._settle_timer.cancel()
        self.optimize()

def main(args=None):

    rclpy.init(args=args)

    node = SOAGradientAscent()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()