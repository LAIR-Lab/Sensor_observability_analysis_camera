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

# Full joint set the trajectory controller expects. joint_5 isn't part of
# the camera chain, so it's just held at its last known position.
CONTROLLER_JOINTS = [
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5"
]

# Real joint limits from so_finale.urdf.xacro (radians).
JOINT_LOWER_LIMITS = np.array([-2.243, -1.418, -1.927, -1.168, -2.0])
JOINT_UPPER_LIMITS = np.array([1.827, 2.08, 1.147, 2.0, 2.4])

# How close to a limit (radians) counts as "at" it for selection purposes.
JOINT_LIMIT_MARGIN = np.deg2rad(1.0)

# How much of the commanded move has to be "missing" after a goal
# completes for that direction to be treated as physically blocked
# (surface/self-collision, not a URDF limit). 0.5 = joint ended up less
# than half way to where it was told to go.
PHYSICAL_BLOCK_FRACTION = 0.5

# Minimum commanded delta (rad) worth checking for a physical block --
# below this, normal settle/measurement noise could look like a stall.
PHYSICAL_BLOCK_MIN_DELTA = np.deg2rad(0.5)


class SOAGradientAscent(Node):

    def __init__(self):

        super().__init__("soa_gradient_ascent")

        ############################################
        # Parameters
        ############################################

        self.step = np.deg2rad(3.5)
        self.base_step = self.step          # ceiling the step can regrow to
        self.min_step = np.deg2rad(0.1)
        self.step_shrink = 0.5
        self.step_grow = 1.35               # was 1.2 -- recover from a shrink faster

        self.improvement_tol = 1e-4
        self.stall_patience = 5
        self.stall_count = 0

        # Stop early once S is "good enough" -- no need to keep
        # fine-tuning toward a theoretical peak once visibility is
        # already solidly acceptable.
        self.soa_acceptable = 0.95

        # Once S gets close to acceptable, ease off -- smaller steps and
        # slower physical motion so we settle into the peak carefully
        # instead of risking an overshoot past soa_acceptable. Applied
        # as a multiplier on top of the existing adaptive step/speed,
        # not a replacement for them, so it doesn't interfere with the
        # overshoot-driven grow/shrink logic below.
        self.soa_slowdown_threshold = 0.85
        self.soa_slowdown_step_factor = 0.4
        self.soa_slowdown_speed_factor = 0.4

        self.max_iterations = 1000
        self.max_rejections = 5

        # Adaptive trajectory duration: don't always take a full second
        # per move -- scale it to how far the joints actually need to
        # travel, so small (near-convergence) steps execute quickly.
        # URDF declares velocity="10" rad/s (~573 deg/s) per joint.
        # Pushed from 300 -> 450 deg/s to move faster; still leaves
        # margin below the URDF ceiling for Gazebo's simulated
        # damping/friction to keep up. Verify empirically: if goals
        # start getting rejected or motion looks unstable, this is the
        # first number to dial back down.
        self.max_joint_speed = np.deg2rad(450.0)  # rad/s
        self.min_traj_time = 0.015                # seconds, floor -- was 0.03

        # Random-kick escape: greedy gradient ascent can only climb the
        # hill it's already on. If it stalls well below soa_acceptable,
        # apply a randomized joint perturbation and resume ascent from
        # there instead of giving up on the first local optimum found.
        #
        # Kick size is self-adaptive (basin-hopping style): if a kick
        # leads to a new best S ("accepted"), shrink the next kick to
        # exploit that region more; if it fails to beat the best S
        # found so far ("rejected"), grow the next kick to search
        # farther. This avoids either always taking oversized jumps
        # that blow away a good local optimum, or always taking tiny
        # jumps that can't escape a wide basin.
        self.max_kicks = 50
        self.kick_deg = 5.0
        self.kick_deg_min = 2.0
        self.kick_deg_max = 20.0
        self.kick_grow = 1.5
        self.kick_shrink = 0.6
        self.kick_count = 0

        # Best S known right before the most recent kick was sent --
        # compared against best_soa afterward to decide whether that
        # kick was "accepted" (found something better) or "rejected."
        self.pre_kick_best_soa = None

        ############################################
        # State
        ############################################

        self.q = None
        self.soa = None
        self.jsoa = None

        self.full_positions = {}

        self.last_soa = -1e9
        self.pending_soa = None

        # Per-joint direction correction, learned empirically. Base
        # direction is still -sign(J_SOA), but if the last accepted
        # move of a given joint made S *worse*, that joint's flip
        # toggles to -1 and stays flipped until the same thing happens
        # again. This sidesteps trusting the analytic gradient's sign
        # outright -- if a specific joint's Jacobian term has a lurking
        # sign error, this lets the controller discover the correct
        # direction from real cause-and-effect instead.
        self.joint_flip = np.ones(len(ACTIVE_JOINTS))
        self.last_moved_joint = None
        self.pending_moved_joint = None

        # Runtime-learned "physically blocked" directions, separate from
        # the static URDF limits above. +1 means "blocked trying to
        # increase this joint," -1 means "blocked trying to decrease it,"
        # 0 means no known block. Populated when a goal completes (or
        # even reports SUCCESSFUL) without the joint actually reaching
        # anywhere close to its commanded position -- i.e. it ran into
        # the table/ground/another link rather than a declared joint
        # limit. Cleared for a joint the moment it's asked to move the
        # opposite direction, so it isn't blacklisted forever.
        self.runtime_blocked_dir = np.zeros(len(ACTIVE_JOINTS))

        # Snapshot of q (active joints) and the full commanded target
        # taken right before a goal is sent, so that once it completes
        # we can measure how much of the commanded delta was actually
        # achieved.
        self.q_before_move = None
        self.pending_target_q = None

        # Best pose seen across the whole run (including pre-kick) --
        # if kicks don't find anything better, fall back to this rather
        # than leaving the arm wherever the last kick landed.
        self.best_soa = -1e9
        self.best_q = None

        # Set after a kick so the very next tick doesn't compare its
        # (likely much lower) post-kick S against the pre-kick plateau
        # value and immediately misread it as "overshoot."
        self.skip_next_convergence_check = False

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

        ############################################
        # Action Client
        ############################################

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        ############################################
        # Timer
        ############################################

        # This mostly acts as a fallback (startup, or recovering from an
        # unexpected stall) rather than the main pacing -- goal_finished
        # already re-triggers optimize() via _settle_timer right after
        # each move completes. Shrunk from 0.5s so that fallback path
        # isn't the slow one.
        self.timer = self.create_timer(
            0.1,
            self.optimize
        )

    ################################################

    def joint_callback(self, msg):

        joint_map = dict(zip(msg.name, msg.position))

        self.full_positions.update(joint_map)

        if all(n in joint_map for n in ACTIVE_JOINTS):
            self.q = np.array([
                joint_map[n]
                for n in ACTIVE_JOINTS
            ])

    ################################################

    def soa_callback(self, msg):

        self.soa = msg.data

    ################################################

    def jacobian_callback(self, msg):

        data = np.array(msg.data)

        if data.shape[0] == len(ACTIVE_JOINTS):
            self.jsoa = data

    ################################################

    def _check_physical_progress(self):

        # Compares where the last goal asked ACTIVE_JOINTS to go against
        # where they actually ended up, and marks any joint that fell far
        # short of its commanded delta as physically blocked in that
        # direction (surface/self-collision), independent of the URDF
        # limits. This is what keeps a repeatedly-obstructed joint (e.g.
        # joint_1 driving down into the table) from being re-selected
        # forever while the node believes each goal "succeeded."

        if self.pending_moved_joint is None:
            return

        if self.q_before_move is None or self.pending_target_q is None:
            return

        j = self.pending_moved_joint

        intended_delta = self.pending_target_q[j] - self.q_before_move[j]

        if abs(intended_delta) < PHYSICAL_BLOCK_MIN_DELTA:
            return

        achieved_delta = self.q[j] - self.q_before_move[j]

        # Fraction of the commanded move that was actually achieved,
        # signed so overshoot/correct-direction-but-short both read
        # sensibly.
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

    ################################################

    def optimize(self):

        if self.finished or self.goal_active:
            return

        if self.q is None or self.soa is None or self.jsoa is None:
            return

        # Check whether the previous move (if any) actually got where it
        # was commanded, before doing anything else this tick.
        self._check_physical_progress()

        if self.soa > self.best_soa:
            self.best_soa = self.soa
            self.best_q = self.q.copy()

        ############################################
        # Stop conditions
        ############################################

        if self.iteration >= self.max_iterations:

            self.get_logger().info("Maximum iterations reached.")
            self.finished = True
            return

        if self.soa >= self.soa_acceptable:

            self.get_logger().info(
                f"SOA acceptable (S={self.soa:.4f} >= {self.soa_acceptable}). Stopping."
            )
            self.finished = True
            return

        ############################################
        # Always move exactly the dominant joint (strongest
        # |gradient|), EXCLUDING any joint that's already at
        # its limit in the direction it would need to move --
        # otherwise the controller keeps re-selecting a joint
        # that physically can't go any further and makes no
        # real progress. Multi-joint variants all underperformed
        # single-joint mode (which reached S=0.92).
        ############################################

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
        top_idx = np.array([dominant_joint])
        weights = grad_abs[top_idx] / max_grad

        ############################################
        # Convergence check (only meaningful in view --
        # S is clamped to exactly 0 outside the FOV cone,
        # so comparing it there always looks "converged")
        ############################################

        # NOTE: this must check *distance from zero*, not sign. S is
        # clamped to exactly 0.0 outside the FOV cone (no info there --
        # skip the comparison), but a genuinely negative in-view S is
        # still a real reading and must NOT be treated the same as the
        # clamp, or regression/flip detection below silently stops
        # firing for any joint whose moves push S negative.
        in_view = abs(self.soa) > 1e-9

        if in_view and self.iteration > 0 and self.skip_next_convergence_check:

            # Fresh baseline just after a kick -- don't compare this
            # tick's S against the pre-kick plateau value.
            self.skip_next_convergence_check = False
            self.stall_count = 0

        elif in_view and self.iteration > 0:

            improvement = self.soa - self.last_soa

            # Near the peak, noise/step-floor granularity makes S wobble
            # by tiny amounts in either direction -- track how many
            # consecutive ticks have been essentially flat, rather than
            # requiring one single tick to land under the threshold
            # (which can miss forever if it's always *just* above it).
            if abs(improvement) < self.improvement_tol:
                self.stall_count += 1
            else:
                self.stall_count = 0

            if improvement < -self.improvement_tol:

                # Real regression (not just noise) -- the last accepted
                # move made S worse. Two responses, both aimed at the
                # joint that actually moved:
                #  1) flip its empirical direction, in case the
                #     analytic gradient's sign is simply wrong for it
                #  2) shrink step, in case the sign was right and this
                #     was a plain overshoot past the peak
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

                # Genuinely climbing -- grow the step back up (capped at
                # base_step) so an earlier shrink (possibly from noise)
                # doesn't permanently cripple the pace for the rest of
                # the run.
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

        ############################################
        # New joint target
        ############################################

        slowdown_active = self.soa >= self.soa_slowdown_threshold

        effective_step = (
            self.step * self.soa_slowdown_step_factor
            if slowdown_active else
            self.step
        )

        q_target = self.q.copy()

        for i, joint in enumerate(top_idx):

            # Jsoa (published by soa_jacobian_cam_node) is already dS/dq --
            # the -1/fov * dtheta/dq chain rule is baked in there. So
            # climbing S means moving WITH the gradient's sign, not
            # against it. (Previously this was -np.sign(...), which is
            # gradient descent on S and was why the arm converged to
            # theta=180 deg instead of 0.)
            direction = self.joint_flip[joint] * np.sign(self.jsoa[joint])

            q_target[joint] += (
                direction *
                effective_step *
                weights[i]
            )


        # Defensive: the selection above should already avoid picking a
        # blocked joint (URDF-limit or runtime-physical), but clip anyway
        # in case a move lands right at (or just past) the boundary.
        q_target = np.clip(q_target, JOINT_LOWER_LIMITS, JOINT_UPPER_LIMITS)

        ############################################
        # Logging
        ############################################

        self.get_logger().info(
            f"""
====================================
Iteration : {self.iteration}
SOA : {self.soa:.4f}
Step : {np.rad2deg(effective_step):.2f} deg{' (slowdown active, S >= ' + str(self.soa_slowdown_threshold) + ')' if slowdown_active else ''}
Dominant joint : {ACTIVE_JOINTS[dominant_joint]}
Moved joints : {[ACTIVE_JOINTS[j] for j in top_idx]}
Weights : {np.round(weights, 3).tolist()}
Target : {q_target}
====================================
"""
        )

        ############################################
        # Send trajectory (S is snapshotted now, only
        # committed as "last_soa" if the goal is accepted)
        ############################################

        self.pending_soa = self.soa
        self.pending_moved_joint = dominant_joint
        self.send_goal(q_target, slow=slowdown_active)

    ################################################

    def _handle_stall(self, reason):

        if self.soa >= self.soa_acceptable:
            # Shouldn't normally get here (the acceptable check runs
            # earlier), but just in case: good enough, no need to kick.
            self.get_logger().info(f"{reason}; S already acceptable. Stopping.")
            self.finished = True
            return

        if self.kick_count < self.max_kicks:

            self.kick_count += 1

            # Judge the PREVIOUS kick (if any) by whether it found a
            # new best S: accepted -> shrink (exploit nearby), rejected
            # -> grow (search farther). First kick uses the starting
            # kick_deg since there's nothing to judge yet.
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

            # Reset step/stall so ascent explores fully from the new
            # spot instead of resuming with a tiny, already-shrunk step.
            self.step = self.base_step
            self.stall_count = 0
            self.skip_next_convergence_check = True

            self.pending_soa = self.soa
            self.pending_moved_joint = None  # kick moves all joints, not one
            self.send_goal(q_kick)
            return

        # Out of kicks -- fall back to the best pose seen, if it's not
        # already where we are.
        self.get_logger().info(
            f"{reason}. Exhausted {self.max_kicks} kicks, best S found = "
            f"{self.best_soa:.4f}. Returning to that pose and stopping."
        )

        if self.best_q is not None and not np.allclose(self.best_q, self.q, atol=1e-4):
            self.finished = True  # set now so goal_finished won't re-trigger optimize()
            self.send_goal(self.best_q)
        else:
            self.finished = True

    ################################################

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

        # Scale trajectory time to how far the joints actually need to
        # move, instead of always taking a fixed 1s even for tiny steps.
        # near soa_slowdown_threshold, deliberately move slower (lower
        # speed cap -> longer traj_time for the same delta) to settle
        # into the peak carefully instead of risking an overshoot.
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

        # Snapshot state right before sending so _check_physical_progress
        # can later tell how much of this commanded move actually
        # happened, regardless of what the action result reports.
        self.q_before_move = self.q.copy()
        self.pending_target_q = q_active.copy()

        self.goal_active = True

        future = self.client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_response_callback
        )

    ################################################

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

        # Accepted -- commit the pre-move S and count this as a real iteration.
        self.rejections = 0
        self.last_soa = self.pending_soa
        self.last_moved_joint = self.pending_moved_joint
        self.iteration += 1

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.goal_finished
        )

    ################################################

    def goal_finished(self, future):

        self.goal_active = False

        result = future.result().result

        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            # The controller itself is reporting trouble (e.g. tolerance
            # violated) -- log it clearly. _check_physical_progress (run
            # at the top of the next optimize()) will still do the actual
            # measurement-based block detection, since some controller
            # configs report SUCCESSFUL purely because the time elapsed,
            # even if the joint never got where it was told to go.
            self.get_logger().warn(
                f"Trajectory finished with error_code="
                f"{result.error_code} ({result.error_string}), "
                f"not SUCCESSFUL."
            )
        else:
            self.get_logger().info("Motion complete.")

        # Don't wait for the next 0.5s timer tick, but give the SOA
        # topics (~20Hz) a brief moment to catch up with the new pose
        # first -- optimizing instantly risked reading stale S from
        # just before the move, which could look like a false
        # "overshoot" and shrink the step for no real reason.
        if not self.finished:
            self._settle_timer = self.create_timer(0.05, self._settle_and_optimize)

    def _settle_and_optimize(self):
        # one-shot: cancel self so this doesn't keep firing every 0.1s
        self._settle_timer.cancel()
        self.optimize()

####################################################

def main(args=None):

    rclpy.init(args=args)

    node = SOAGradientAscent()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()