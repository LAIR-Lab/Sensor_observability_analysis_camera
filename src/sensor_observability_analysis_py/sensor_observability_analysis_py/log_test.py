import argparse
import time

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


JOINT_ORDER = [
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5"
]


class JsoaLogger(Node):

    def __init__(self, duration):

        super().__init__("jsoa_logger")

        self.duration = duration
        self.start_time = time.time()

        self.t_jsoa = []
        self.jsoa_hist = []

        self.t_q = []
        self.q_hist = []

        self.create_subscription(
            Float64MultiArray,
            "/soa/jacobian",
            self.jsoa_callback,
            10
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.get_logger().info(
            f"Logging /soa/jacobian and /joint_states for {duration:.1f}s -- "
            f"send your trajectory goal now."
        )

    def jsoa_callback(self, msg):

        t = time.time() - self.start_time
        self.t_jsoa.append(t)
        self.jsoa_hist.append(list(msg.data))

    def joint_callback(self, msg):

        try:
            joint_map = dict(zip(msg.name, msg.position))
            q = [joint_map[n] for n in JOINT_ORDER]
        except KeyError:
            return  # message doesn't contain all joints yet

        t = time.time() - self.start_time
        self.t_q.append(t)
        self.q_hist.append(q)

    def time_remaining(self):
        return self.duration - (time.time() - self.start_time)


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=6.0,
                         help="Seconds to log before plotting")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = JsoaLogger(args.duration)

    while rclpy.ok() and node.time_remaining() > 0:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.get_logger().info(
        f"Done logging: {len(node.jsoa_hist)} J_SOA samples, "
        f"{len(node.q_hist)} joint_state samples"
    )

    jsoa_hist = np.array(node.jsoa_hist)      # (N, n_joints)
    t_jsoa = np.array(node.t_jsoa)
    q_hist = np.array(node.q_hist)            # (M, n_joints)
    t_q = np.array(node.t_q)

    node.destroy_node()
    rclpy.shutdown()

    if jsoa_hist.size == 0:
        print("No /soa/jacobian messages received -- check the SOA node is running "
              "and actually publishing during this window.")
        return

    n_joints = jsoa_hist.shape[1]

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    for j in range(n_joints):
        label = JOINT_ORDER[j] if j < len(JOINT_ORDER) else f"joint_{j}"
        axes[0].plot(t_jsoa, jsoa_hist[:, j], label=label)
    axes[0].set_ylabel("dS/dq_j")
    axes[0].set_title("J_SOA components during trajectory execution")
    axes[0].legend(loc="upper right", ncol=3, fontsize=8)
    axes[0].grid(True, alpha=0.3)

    if q_hist.size > 0:
        for j in range(q_hist.shape[1]):
            label = JOINT_ORDER[j] if j < len(JOINT_ORDER) else f"joint_{j}"
            axes[1].plot(t_q, q_hist[:, j], label=label)
        axes[1].set_ylabel("q (rad)")
        axes[1].set_xlabel("time (s)")
        axes[1].legend(loc="upper right", ncol=3, fontsize=8)
        axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("jsoa_live_log.png", dpi=150)
    print("Saved plot to jsoa_live_log.png")

    header = ",".join(JOINT_ORDER)
    np.savetxt("jsoa_live_log.csv", jsoa_hist, delimiter=",", header=header, comments="")
    print("Saved data to jsoa_live_log.csv")


if __name__ == "__main__":
    main()