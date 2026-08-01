#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


def quat_to_rot_matrix(x, y, z, w):

    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])


class SOATFNode(Node):

    def __init__(self):

        super().__init__("soa_tf_node")

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_frame", "camera_1")
        self.declare_parameter("poi_frame", "red_sphere")
        self.declare_parameter("obstacle_frame", "black_sphere")
        self.declare_parameter("obstacle_radius", 0.1)

        self.world_frame = self.get_parameter(
            "world_frame").value

        self.camera_frame = self.get_parameter(
            "camera_frame").value

        self.poi_frame = self.get_parameter(
            "poi_frame").value

        self.obstacle_frame = self.get_parameter(
            "obstacle_frame").value

        self.obstacle_radius = self.get_parameter(
            "obstacle_radius").value

        # -------------------------------------------------
        # TF
        # -------------------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # -------------------------------------------------
        # Timer
        # -------------------------------------------------

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

    # =====================================================
    # TF lookup
    # =====================================================

    def lookup_transform(self, frame):

        return self.tf_buffer.lookup_transform(
            self.world_frame,
            frame,
            rclpy.time.Time()
        )

    # =====================================================
    # Main loop
    # =====================================================

    def timer_callback(self):

        try:

            # ------------------------------------------
            # Lookup TF
            # ------------------------------------------

            cam_tf = self.lookup_transform(
                self.camera_frame
            )

            poi_tf = self.lookup_transform(
                self.poi_frame
            )

            obs_tf = self.lookup_transform(
                self.obstacle_frame
            )

            # ------------------------------------------
            # Positions
            # ------------------------------------------

            Pc = np.array([
                cam_tf.transform.translation.x,
                cam_tf.transform.translation.y,
                cam_tf.transform.translation.z
            ])

            Ppoi = np.array([
                poi_tf.transform.translation.x,
                poi_tf.transform.translation.y,
                poi_tf.transform.translation.z
            ])

            Pobs = np.array([
                obs_tf.transform.translation.x,
                obs_tf.transform.translation.y,
                obs_tf.transform.translation.z
            ])

            # ------------------------------------------
            # Camera orientation
            # ------------------------------------------

            R = quat_to_rot_matrix(
                cam_tf.transform.rotation.x,
                cam_tf.transform.rotation.y,
                cam_tf.transform.rotation.z,
                cam_tf.transform.rotation.w
            )

            # Camera looks along +X

            axis_camera = np.array([
                1.0,
                0.0,
                0.0
            ])

            axis_world = R @ axis_camera
            axis_world /= np.linalg.norm(axis_world)

            # ------------------------------------------
            # Camera -> POI
            # ------------------------------------------

            r_poi = Ppoi - Pc

            d_poi = np.linalg.norm(r_poi)

            r_poi /= d_poi

            # ------------------------------------------
            # Camera -> Obstacle
            # ------------------------------------------

            r_obs = Pobs - Pc

            d_obs = np.linalg.norm(r_obs)

            r_obs /= d_obs

            # ------------------------------------------
            # Viewing angle to POI
            # ------------------------------------------

            cos_theta_poi = np.dot(
                axis_world,
                r_poi
            )

            cos_theta_poi = np.clip(
                cos_theta_poi,
                -1.0,
                1.0
            )

            theta_poi = np.arccos(
                cos_theta_poi
            )

            # ------------------------------------------
            # Viewing angle to obstacle
            # ------------------------------------------

            cos_theta_obs = np.dot(
                axis_world,
                r_obs
            )

            cos_theta_obs = np.clip(
                cos_theta_obs,
                -1.0,
                1.0
            )

            theta_obs = np.arccos(
                cos_theta_obs
            )

            # ------------------------------------------
            # Angle between POI and obstacle
            # ------------------------------------------

            cos_phi = np.dot(
                r_poi,
                r_obs
            )

            cos_phi = np.clip(
                cos_phi,
                -1.0,
                1.0
            )

            phi = np.arccos(
                cos_phi
            )

            # ------------------------------------------
            # Obstacle cone angle
            # ------------------------------------------

            cone = np.arctan2(
                self.obstacle_radius,
                d_obs
            )

            # ------------------------------------------
            # Visibility
            # ------------------------------------------

            if d_obs < d_poi and phi < cone:
                visible = 0
            else:
                visible = 1

            # ------------------------------------------
            # Print
            # ------------------------------------------

            self.get_logger().info(

                "\n"
                "====================================\n"
                f"Camera -> POI distance : {d_poi:.3f} m\n"
                f"Camera -> Obs distance : {d_obs:.3f} m\n"
                "\n"
                f"POI cos(theta)         : {cos_theta_poi:.4f}\n"
                f"Obs cos(theta)         : {cos_theta_obs:.4f}\n"
                "\n"
                f"POI theta              : {np.degrees(theta_poi):.2f} deg\n"
                f"Obs theta              : {np.degrees(theta_obs):.2f} deg\n"
                "\n"
                f"phi                    : {np.degrees(phi):.2f} deg\n"
                f"cone                   : {np.degrees(cone):.2f} deg\n"
                "\n"
                f"Visible                : {visible}\n"
                "====================================",

                throttle_duration_sec=1.0

            )

        except TransformException as ex:

            self.get_logger().warn(
                f"TF Error: {ex}"
            )


def main():

    rclpy.init()

    node = SOATFNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()