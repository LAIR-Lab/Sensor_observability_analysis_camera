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

class SOACameraJacobian(Node):

    def __init__(self):

        super().__init__("soa_camera_jacobian")

        self.robot = rtb.ERobot.URDF(
            urdf_path
        )

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

        self.theta_pub = self.create_publisher(
            Float64,
            "/soa/theta",
            10
        )

        self.soa_pub = self.create_publisher(
            Float64,
            "/soa/value",
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

    #SOA

    def compute_soa(self, q, Ppoi):

        try:

            tf_cam = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_3",
                rclpy.time.Time()
            )

        except TransformException as ex:

            raise RuntimeError(f"TF lookup for camera_3 failed: {ex}")

        Pc = np.array([
            tf_cam.transform.translation.x,
            tf_cam.transform.translation.y,
            tf_cam.transform.translation.z
        ])

        Rc = self.quat_to_rotmat(
            tf_cam.transform.rotation.x,
            tf_cam.transform.rotation.y,
            tf_cam.transform.rotation.z,
            tf_cam.transform.rotation.w
        )

        # camera optical axis (+X)
#axis x is 0 
        a = Rc[:, 0]

        # Robot Jacobian

        Jc = np.asarray(
            self.robot.jacob0(
                q,
                end="camera_3"
            )
        )

        # Relative vector

        r = Ppoi - Pc

        d = np.linalg.norm(r)

        if d < 1e-8:
            raise RuntimeError("Camera coincides with POI")

        rhat = r / d

        c = np.clip(np.dot(a, rhat), -1.0, 1.0)

        theta = np.arccos(c)

        sigma = self.fov / 6.0
        # this equation is for linear function.
        S = 1.0 - theta / self.fov

        #
        c_safe = np.clip(c, -0.9994, 0.9994)  # keeps ~2 deg clear of either pole
        denom = np.sqrt(max(1e-12, 1.0 - c_safe * c_safe))

        I = np.eye(3)

        dtheta_dp = (
            a @ (I - np.outer(rhat, rhat))
        ) / (
            d * denom
        )

        dS_dp = -dtheta_dp / self.fov


        dtheta_dw = (
            rhat @ self.skew(a)
        ) / denom

        dS_dw = -dtheta_dw / self.fov

        dS_dxc = np.hstack((
            dS_dp,
            dS_dw
        ))

        Jsoa = dS_dxc @ Jc

        return {
            "score": S,
            "Jsoa": Jsoa,
            "Pc": Pc,
            "axis": a,
            "Jcamera": Jc,
            "angle": theta,
            "Jc_shape":Jc.shape
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

        Ppoi = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z
        ])

        try:

            result = self.compute_soa(
                self.q,
                Ppoi
            )

        except RuntimeError as ex:

            self.get_logger().warn(str(ex))
            return

        msg = Float64MultiArray()
        msg.data = result["Jsoa"].tolist()
        self.jsoa_pub.publish(msg)
        msg2 = Float64()
        msg2.data = result["score"]
        self.soa_pub.publish(msg2)
        msg3 = Float64()
        msg3.data = result["angle"]
        self.theta_pub.publish(msg3)

        self.get_logger().info(
            "\n"
            "====================================\n"
            f"SOA : {result['score']:.4f}\n\n"
            f"J_SOA:\n{result['Jsoa']}\n\n"
            f"Camera Position:\n{result['Pc']}\n\n"
            f"Camera Axis:\n{result['axis']}\n"
            f"Theta:\n{result['angle']}\n"
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