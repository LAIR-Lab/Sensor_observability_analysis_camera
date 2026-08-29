import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

ACTIVE_JOINTS = [
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5"
]

CONTROLLER_JOINTS = [
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5"
]

JOINT_LOWER_LIMITS = np.array([-2.243, -1.418, -1.927, -1.168, -2.0, -0.178])
JOINT_UPPER_LIMITS = np.array([1.827, 2.08, 1.147, 2.0, 2.4, 1.555])
JOINT_LIMIT_MARGIN = np.deg2rad(1.0)
PHYSICAL_BLOCK_FRACTION = 0.5
PHYSICAL_BLOCK_MIN_DELTA = np.deg2rad(0.5)


class SOAGradientAscent(Node):

    def __init__(self):

        super().__init__("soa_gradient_ascent")

        # Parameters

        self.step = np.deg2rad(1.0)
        self.base_step = self.step          # ceiling the step can regrow to
        self.min_step = np.deg2rad(0.1)
        self.step_shrink = 0.5
        self.step_grow = 1.35               # was 1.2 -- faster now

        self.improvement_tol = 1e-4
        self.stall_patience = 8
        self.stall_count = 0

        self.soa_acceptable = 1.0

        self.soa_slowdown_threshold = 0.85
        self.soa_slowdown_step_factor = 0.4
        self.soa_slowdown_speed_factor = 0.4

        self.max_iterations = 1000
        self.max_rejections = 5
        self.max_joint_speed = np.deg2rad(450.0)  # rad/s
        self.min_traj_time = 0.015                # seconds, floor -- was 0.03
        self.max_kicks = 50
        self.kick_deg = 0 #5.0
        self.kick_deg_min = 0 #2.0
        self.kick_deg_max = 0 #20.0
        self.kick_grow = 0 #1.5
        self.kick_shrink = 0 #0.6
        self.kick_count = 0
        self.pre_kick_best_soa = None
        self.q = None
        self.soa = None      
        self.jsoa = None      
        self.true_min = None  

        self.full_positions = {}

        self.last_soa = -1e9
        self.pending_soa = None
        self.joint_flip = np.ones(len(ACTIVE_JOINTS))
        self.last_moved_joint = None
        self.pending_moved_joint = None
        self.runtime_blocked_dir = np.zeros(len(ACTIVE_JOINTS))
        self.q_before_move = None
        self.pending_target_q = None

        self.best_soa = -1e9
        self.best_q = None
        self.skip_next_convergence_check = False

        self.iteration = 0
        self.rejections = 0
        self.goal_active = False
        self.finished = False
        self._settle_timer = None
        self._server_confirmed = False
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            1
        )

        self.create_subscription(
            Float64,
            "/soa/value",
            self.soa_callback,
            1
        )

        self.create_subscription(
            Float64MultiArray,
            "/soa/jacobian",
            self.jacobian_callback,
            1
        )

        self.create_subscription(
            Float64,
            "/soa/value_2",
            self.true_min_callback,
            1
        )

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )


        self.timer = self.create_timer(
            0.02,
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

    def true_min_callback(self, msg):
        self.true_min = msg.data

    def _check_physical_progress(self):

        if self.pending_moved_joint is None:
            return

        if self.q_before_move is None or self.pending_target_q is None:
            return

        j = self.pending_moved_joint

        intended_delta = self.pending_target_q[j] - self.q_before_move[j]

        if abs(intended_delta) < PHYSICAL_BLOCK_MIN_DELTA:
            return

        achieved_delta = self.q[j] - self.q_before_move[j]
        achieved_fraction = achieved_delta / intended_delta

        if achieved_fraction < PHYSICAL_BLOCK_FRACTION:

            direction = np.sign(intended_delta)
            self.runtime_blocked_dir[j] = direction

            self.get_logger().warn(
                f"{ACTIVE_JOINTS[j]} physically blocked moving "
                f"{'up' if direction > 0 else 'down'} -- only reached "
                f"{achieved_fraction * 100:.0f}% of the commanded move "
                f"(likely a surface/self-collision, not a joint limit). "
                f"Excluding that direction until it's asked to move back "
                f"the other way."
            )

    def optimize(self):

        if self.finished or self.goal_active:
            return

        if self.q is None or self.soa is None or self.jsoa is None:
            return

        self._check_physical_progress()

        if self.soa > self.best_soa:
            self.best_soa = self.soa
            self.best_q = self.q.copy()

        # Stop conditions

        if self.iteration >= self.max_iterations:

            self.get_logger().info("Maximum iterations reached.")
            self.finished = True
            return
        
        gate_score = self.true_min if self.true_min is not None else self.soa

        if gate_score >= self.soa_acceptable:

            self.get_logger().info(
                f"SOA acceptable (true min S={gate_score:.4f} >= {self.soa_acceptable}). Stopping."
            )
            self.finished = True
            return
        grad_abs = np.abs(self.jsoa)

        at_lower = self.q <= (JOINT_LOWER_LIMITS + JOINT_LIMIT_MARGIN)
        at_upper = self.q >= (JOINT_UPPER_LIMITS - JOINT_LIMIT_MARGIN)

        desired_direction = self.joint_flip * -np.sign(self.jsoa)

        blocked = (
            (at_lower & (desired_direction < 0)) |
            (at_upper & (desired_direction > 0)) |
            (self.runtime_blocked_dir == np.sign(desired_direction))
        )

        usable_grad_abs = np.where(blocked, 0.0, grad_abs)
        max_grad = np.max(usable_grad_abs)

        if max_grad < 1e-8:

            if np.any(blocked) and np.max(grad_abs) >= 1e-8:
                self._handle_stall(
                    f"All joints with real gradient are at their limits "
                    f"(URDF or physical surface): "
                    f"{[ACTIVE_JOINTS[j] for j in np.where(blocked)[0]]}"
                )
            else:
                self._handle_stall("Zero gradient")
            return

        dominant_joint = int(np.argmax(usable_grad_abs))
        top_idx = np.arange(len(ACTIVE_JOINTS))
        weights = usable_grad_abs[top_idx] / max_grad

        in_view = abs(self.soa) > 1e-9

        if in_view and self.iteration > 0 and self.skip_next_convergence_check:

            # tick's S against the pre-kick plateau value.
            self.skip_next_convergence_check = False
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

            if self.stall_count >= self.stall_patience and self.step <= self.min_step * 1.0001:

                self._handle_stall(
                    f"Flat for {self.stall_count} ticks at floor step (S={self.soa:.4f})"
                )
                return
            
        # New joint target

        slowdown_active = self.soa >= self.soa_slowdown_threshold

        effective_step = (
            self.step * self.soa_slowdown_step_factor
            if slowdown_active else
            self.step
        )

        q_target = self.q.copy()

        for i, joint in enumerate(top_idx):

            direction = self.joint_flip[joint] * np.sign(self.jsoa[joint])

            q_target[joint] += (
                direction *
                effective_step *
                weights[i]
            )

        q_target = np.clip(q_target, JOINT_LOWER_LIMITS, JOINT_UPPER_LIMITS)

        self.get_logger().info(
            f"""
====================================
Iteration : {self.iteration}
SOA (Gamma_min blend) : {self.soa:.4f}   True min : {self.true_min}
Step : {np.rad2deg(effective_step):.2f} deg{' (slowdown active, S >= ' + str(self.soa_slowdown_threshold) + ')' if slowdown_active else ''}
Dominant joint : {ACTIVE_JOINTS[dominant_joint]}
Moved joints : {[ACTIVE_JOINTS[j] for j in top_idx]}
Weights : {np.round(weights, 3).tolist()}
Target : {q_target}
====================================
"""
        )

        self.pending_soa = self.soa
        self.pending_moved_joint = dominant_joint
        self.send_goal(q_target, slow=slowdown_active)

    def _handle_stall(self, reason):

        gate_score = self.true_min if self.true_min is not None else self.soa

        if gate_score >= self.soa_acceptable:
            self.get_logger().info(f"{reason}; S already acceptable. Stopping.")
            self.finished = True
            return

        if self.kick_count < self.max_kicks:

            self.kick_count += 1
            if self.pre_kick_best_soa is not None:

                accepted = self.best_soa > self.pre_kick_best_soa + self.improvement_tol

                old_kick_deg = self.kick_deg

                if accepted:
                    self.kick_deg = max(self.kick_deg_min, self.kick_deg * self.kick_shrink)
                else:
                    self.kick_deg = min(self.kick_deg_max, self.kick_deg * self.kick_grow)

                self.get_logger().info(
                    f"Previous kick {'accepted (new best!)' if accepted else 'rejected'}; "
                    f"kick size {old_kick_deg:.1f} -> {self.kick_deg:.1f} deg"
                )

            self.pre_kick_best_soa = self.best_soa

            perturbation = np.random.uniform(
                -np.deg2rad(self.kick_deg),
                np.deg2rad(self.kick_deg),
                size=len(ACTIVE_JOINTS)
            )
            q_kick = np.clip(self.q + perturbation, JOINT_LOWER_LIMITS, JOINT_UPPER_LIMITS)

            self.get_logger().info(
                f"{reason}. Stuck below acceptable S ({self.soa_acceptable}) -- "
                f"trying random kick {self.kick_count}/{self.max_kicks} "
                f"(±{self.kick_deg:.1f} deg, best S so far: {self.best_soa:.4f})."
            )

            self.step = self.base_step
            self.stall_count = 0
            self.skip_next_convergence_check = True

            self.pending_soa = self.soa
            self.pending_moved_joint = None 
            self.send_goal(q_kick)
            return

        self.get_logger().info(
            f"{reason}. Exhausted {self.max_kicks} kicks, best S found = "
            f"{self.best_soa:.4f}. Returning to that pose and stopping."
        )

        if self.best_q is not None and not np.allclose(self.best_q, self.q, atol=1e-4):
            self.finished = True 
            self.send_goal(self.best_q)
        else:
            self.finished = True

    def send_goal(self, q_active, slow=False):

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

        effective_max_speed = (
            self.max_joint_speed * self.soa_slowdown_speed_factor
            if slow else
            self.max_joint_speed
        )

        max_delta = 0.0
        for name, target_pos in zip(CONTROLLER_JOINTS, positions):
            current_pos = self.full_positions.get(name, target_pos)
            max_delta = max(max_delta, abs(target_pos - current_pos))

        traj_time = max(self.min_traj_time, max_delta / effective_max_speed)
        sec = int(traj_time)
        nanosec = int((traj_time - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

        goal.trajectory.points.append(point)

        self.q_before_move = self.q.copy()
        self.pending_target_q = q_active.copy()

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
                self.get_logger().error("Too many rejections, stopping.")
                self.finished = True

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
        else:
            self.get_logger().info("Motion complete.")

        
        if not self.finished:
            self._settle_timer = self.create_timer(0.025, self._settle_and_optimize)

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