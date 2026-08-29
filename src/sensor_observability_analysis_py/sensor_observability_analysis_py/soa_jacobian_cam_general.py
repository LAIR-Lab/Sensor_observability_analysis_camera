import numpy as np
import roboticstoolbox as rtb
from std_msgs.msg import Float64MultiArray, Float64
from ament_index_python.packages import get_package_share_directory
import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from tf2_ros import Buffer, TransformListener, TransformException

urdf_path = os.path.join(
    get_package_share_directory("digital_twin"),
    "urdf",
    "so_finale.urdf.xacro"
)

JOINT_ORDER = [
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5"
]

# Sharpness of the smooth-min (Gamma_min^k) blend across the two cameras.
# With only 2 cameras, gaps of ~0.1-0.9 in S occur -- k=20 keeps the
# gradient smooth right at a crossover while staying close to true
# hard-min elsewhere (see MATH.md for the derivation).
SOFTMAX_K = 20.0

# Smooth-max operator (log-sum-exp) and its exact gradient weights
# (soft-argmax). See MATH.md: Gamma_max^k(S) >= max(S) always, and
# grad Gamma_max^k = sum_c w_c * J_c with w_c = gamma_argmax_weights(S)[c] --
# no leftover covariance term, unlike a naive softmax-weighted average.

def gamma_max(S, k):

    S = np.asarray(S)
    m = S.max()

    return m + np.log(np.sum(np.exp(k * (S - m)))) / k


def gamma_argmax_weights(S, k):

    S = np.asarray(S)
    m = S.max()

    e = np.exp(k * (S - m))

    return e / e.sum()


# Smooth-min operator, via the identity Gamma_min^k(S) = -Gamma_max^k(-S).
# Unlike Gamma_max^k (which lets a weak camera be ignored), this
# concentrates the gradient on whichever camera is currently WORSE, so
# both cameras get pushed toward acceptable visibility instead of one
# being abandoned once it falls far enough behind. See MATH.md.

def gamma_min(S, k):

    return -gamma_max(-np.asarray(S), k)


def gamma_argmin_weights(S, k):

    return gamma_argmax_weights(-np.asarray(S), k)


class SOACameraJacobian(Node):

    def __init__(self):

        super().__init__("soa_camera_jacobian")

        self.robot = rtb.ERobot.URDF(
            urdf_path
        )

        self._log_joint_structure()

        self.fov = 1.047
        self.q = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.jsoa_pub = self.create_publisher(
            Float64MultiArray,
            "/soa/jacobian",
            10
        )

        self.jsoa_pub_2= self.create_publisher(
                    Float64MultiArray,
                    "/soa/jacobian_2",
                    10
                )

        self.theta_pub = self.create_publisher(
            Float64,
            "/soa/theta",
            10
        )

        self.theta_pub_2 = self.create_publisher(
                    Float64,
                    "/soa/theta_2",
                    10
                )

        self.soa_pub = self.create_publisher(
            Float64,
            "/soa/value",
            10
        )

        self.soa_pub_2 = self.create_publisher(
                    Float64,
                    "/soa/value_2",
                    10
                )

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.timer = self.create_timer(
            0.01,
            self.update
        )

    # Utility

    def _log_joint_structure(self):

        # One-time startup diagnostic: confirms whether jindex order
        # actually matches JOINT_ORDER. If jindex 0 isn't "joint_0"'s
        # link, joint_indices_for_end's assumption is wrong and the
        # padding will scatter gradients into the wrong slots.

        lines = ["Robot link/joint structure (link_name, isjoint, jindex, parent):"]

        for link in self.robot.links:

            parent_name = link.parent.name if link.parent is not None else None

            lines.append(
                f"  {link.name:20s} isjoint={link.isjoint!s:5s} "
                f"jindex={str(link.jindex):4s} parent={parent_name}"
            )

        self.get_logger().info("\n".join(lines))

    def skew(self, v):

        return np.array([
            [0,     -v[2],  v[1]],
            [v[2],   0,    -v[0]],
            [-v[1],  v[0],  0]
        ])

    def quat_to_rotmat(self, x, y, z, w):

        n = np.sqrt(x*x + y*y + z*z + w*w)

        if n < 1e-12:
            raise RuntimeError("Degenerate quaternion from TF")

        x, y, z, w = x / n, y / n, z / n, w / n

        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),         2*(x*z + y*w)],
            [2*(x*y + z*w),         1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [2*(x*z - y*w),         2*(y*z + x*w),         1 - 2*(x*x + y*y)]
        ])

    def joint_indices_for_end(self, end):

        # Walk from `end` back to the base, collecting the jindex
        # (position in the robot's own q vector) of every actuated
        # joint on the path. jacob0(q, end=end) returns one column per
        # joint on this same path, in the same base->end order, so
        # jindex tells us which slot in the full 6-vector each column
        # of that partial Jacobian belongs to.
        #
        # NOTE: link.name is the URDF *link* name, not the joint name
        # ("joint_0" etc.) -- do not filter on link.name, it won't
        # match JOINT_ORDER. jindex is assigned by rtb in URDF
        # declaration order, which we're assuming matches JOINT_ORDER
        # (confirmed via the startup diagnostic below -- check the log
        # if this assumption is ever wrong).

        idxs = []
        link = self.robot.link_dict[end]

        while link is not None:

            if link.isjoint:
                idxs.append(link.jindex)

            link = link.parent

        idxs.reverse()

        if not idxs:
            raise RuntimeError(
                f"No actuated joints found on path to '{end}' -- "
                f"check robot.link_dict['{end}'].parent chain"
            )

        return idxs

    def pad_to_full(self, Jsoa_partial, joint_indices):

        # Cameras on different branches of the kinematic tree see
        # different numbers of upstream joints, so jacob0() returns
        # narrower Jacobians for cameras closer to the base. This
        # scatters each partial gradient entry back to its correct
        # joint slot in the full JOINT_ORDER vector, zero elsewhere --
        # zero is the mathematically correct value for a joint that
        # genuinely doesn't move that camera at all.

        full = np.zeros(len(JOINT_ORDER))
        full[joint_indices] = Jsoa_partial

        return full

    #SOA

    def compute_soa(self, q, Ppoi):

        try:

            tf_cam = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_3",
                rclpy.time.Time()
            )

            tf_cam_2 = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_2",
                rclpy.time.Time()
            )

        except TransformException as ex:

            raise RuntimeError(f"TF lookup for camera_3/camera_2 failed: {ex}")

        Pc = np.array([
            tf_cam.transform.translation.x,
            tf_cam.transform.translation.y,
            tf_cam.transform.translation.z
        ])

        Pc_2 = np.array([   
            tf_cam_2.transform.translation.x,
            tf_cam_2.transform.translation.y,
            tf_cam_2.transform.translation.z
        ])

        Rc = self.quat_to_rotmat(
            tf_cam.transform.rotation.x,
            tf_cam.transform.rotation.y,
            tf_cam.transform.rotation.z,
            tf_cam.transform.rotation.w
        )

        Rc_2 = self.quat_to_rotmat(
            tf_cam_2.transform.rotation.x,
            tf_cam_2.transform.rotation.y,
            tf_cam_2.transform.rotation.z,
            tf_cam_2.transform.rotation.w
        )

        #camera optical axis (+X)
        #axis x is 0 
        a = Rc[:, 0]
        a_2 = Rc_2[:, 0]

        # Robot Jacobian

        Jc = np.asarray(
            self.robot.jacob0(
                q,
                end="camera_3"
            )
        )

        Jc_2 = np.asarray(
            self.robot.jacob0(
                q,
                end="camera_2"
            )
        )

        # Relative vector

        r = Ppoi - Pc

        r_2 = Ppoi - Pc_2

        d = np.linalg.norm(r)
        d_2 = np.linalg.norm(r_2)

        if d < 1e-8:
            raise RuntimeError("Camera coincides with POI")

        if d_2 < 1e-8:
                    raise RuntimeError("Camera coincides with POI")

        rhat = r / d

        rhat_2 = r_2 / d_2

        c = np.clip(np.dot(a, rhat), -1.0, 1.0)

        c_2 = np.clip(np.dot(a_2, rhat_2), -1.0, 1.0)

        theta = np.arccos(c)

        theta_2 = np.arccos(c_2)

        sigma = self.fov / 6.0
        # this equation is for linear function.
        S = 1.0 - theta / self.fov

        S_2 = 1.0 - theta_2 / self.fov

        c_safe = np.clip(c, -0.9994, 0.9994)  # keeps ~2 deg clear of either pole
        denom = np.sqrt(max(1e-12, 1.0 - c_safe * c_safe))

        c_safe_2 = np.clip(c_2, -0.9994, 0.9994)  # keeps ~2 deg clear of either pole
        denom_2 = np.sqrt(max(1e-12, 1.0 - c_safe_2 * c_safe_2))

        I = np.eye(3)

        dtheta_dp = (
            a @ (I - np.outer(rhat, rhat))
        ) / (
            d * denom
        )

        dtheta_dp_2 = (
                    a_2 @ (I - np.outer(rhat_2, rhat_2))
                ) / (
                    d_2 * denom_2
                )

        dS_dp = -dtheta_dp / self.fov

        dS_dp_2 = -dtheta_dp_2 / self.fov

        dtheta_dw = (
            rhat @ self.skew(a)
        ) / denom

        dtheta_dw_2 = (
                    rhat_2 @ self.skew(a_2)
                ) / denom_2

        dS_dw = -dtheta_dw / self.fov

        dS_dw_2 = -dtheta_dw_2 / self.fov

        dS_dxc = np.hstack((
            dS_dp,
            dS_dw
        ))

        dS_dxc_2 = np.hstack((
            dS_dp_2,
            dS_dw_2
        ))

        Jsoa = dS_dxc @ Jc

        Jsoa_2 = dS_dxc_2 @ Jc_2

        # Cameras sit on different branches, so Jc/Jc_2 can have
        # different numbers of columns -- pad both back to the full
        # joint vector so downstream code (aggregation, np.stack) can
        # always assume a fixed length.
        Jsoa = self.pad_to_full(
            Jsoa,
            self.joint_indices_for_end("camera_3")
        )

        Jsoa_2 = self.pad_to_full(
            Jsoa_2,
            self.joint_indices_for_end("camera_2")
        )

        return {
            "score": S,
            "Jsoa": Jsoa,
            "Pc": Pc,
            "axis": a,
            "Jcamera": Jc,
            "angle": theta,
            "Jc_shape":Jc.shape,
            "score_2": S_2,
            "Jsoa_2": Jsoa_2,
            "Pc_2": Pc_2,
            "axis_2": a_2,
            "Jcamera_2": Jc_2,
            "angle_2": theta_2,
            "Jc_shape_2": Jc_2.shape
        }

    def update(self):

        if self.q is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                "world",
                "red_sphere",
                rclpy.time.Time()
            )

        except TransformException as ex:

            self.get_logger().warn(str(ex))
            return

        try:
        
            tf_2 = self.tf_buffer.lookup_transform(
                "world",
                "red_sphere_2",
                rclpy.time.Time()
            )
        
        except TransformException as ex:

            self.get_logger().warn(str(ex))
            return

        Ppoi = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z
        ])

        Ppoi_2 = np.array([   
            tf_2.transform.translation.x,
            tf_2.transform.translation.y,
            tf_2.transform.translation.z
        ])

        try:

            result = self.compute_soa(
                self.q,
                Ppoi
            )

            result_2 = self.compute_soa(
                self.q,
                Ppoi_2
            ) 


        except RuntimeError as ex:

            self.get_logger().warn(str(ex))
            return

        S_arr = np.array([result["score"], result_2["score_2"]])
        J_stack = np.stack([result["Jsoa"], result_2["Jsoa_2"]])

        S_agg = gamma_min(S_arr, SOFTMAX_K)
        weights = gamma_argmin_weights(S_arr, SOFTMAX_K)
        Jsoa_agg = weights @ J_stack

        # True min, for stop-condition gating downstream -- S_agg is always
        # <= true_min (log-sum-exp is deflated by the blend for Gamma_min),
        # and "done" should mean BOTH cameras are acceptable, i.e. the
        # WORSE camera's score clears the threshold -- so gate on true_min,
        # not S_agg.
        true_min = S_arr.min()

        msg = Float64MultiArray()
        msg.data = Jsoa_agg.tolist()
        self.jsoa_pub.publish(msg)

        msg2 = Float64()
        msg2.data = S_agg
        self.soa_pub.publish(msg2)

        # camera_3 raw angle, unchanged for backward compat
        msg3 = Float64()
        msg3.data = result["angle"]
        self.theta_pub.publish(msg3)

        # camera_2 raw (padded) Jsoa_2 and angle -- debug-only, not used
        # by the controller, but useful for exactly this kind of
        # per-camera inspection
        msg_j2 = Float64MultiArray()
        msg_j2.data = result["Jsoa_2"].tolist()
        self.jsoa_pub_2.publish(msg_j2)

        msg_t2 = Float64()
        msg_t2.data = result["angle_2"]
        self.theta_pub_2.publish(msg_t2)

        msg2_2 = Float64()
        msg2_2.data = true_min
        self.soa_pub_2.publish(msg2_2)

        self.get_logger().info(
            "\n"
            "====================================\n"
            f"S_agg (Gamma_min) : {S_agg:.4f}   True min : {true_min:.4f}\n"
            f"SOA_3 : {result['score']:.4f}   SOA_2 : {result_2['score_2']:.4f}\n"
            f"Weights (w3, w2) : {np.round(weights, 3).tolist()}\n\n"
            f"J_SOA_agg:\n{Jsoa_agg}\n\n"
            f"Theta_3 : {result['angle']:.4f}   Theta_2 : {result_2['angle_2']:.4f}\n"
            "===================================="
        )

    def joint_callback(self, msg):

        joint_map = dict(zip(msg.name, msg.position))

        self.q = np.array([
            joint_map[n]
            for n in JOINT_ORDER
        ])


def main(args=None):

    rclpy.init(args=args)

    node = SOACameraJacobian()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()