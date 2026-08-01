from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    pkg_share = get_package_share_directory(
        "sensor_observability_analysis_py"
    )

    config = os.path.join(
        pkg_share,
        "config",
        "soa_camera_jacobian.yaml"
    )

    node_cam_tf_node = Node(
            package="sensor_observability_analysis_py",
            executable="soa_cam_tf_node",
            output="screen",
            parameters=[{
                "world_frame": "world",
                "camera_frame": "camera_1",
                "poi_frame": "red_sphere"
            }]
        )
    
    soa_node = Node(
        package="sensor_observability_analysis_py",
        executable="soa_camera_jacobian_node",
        name="soa_camera_jacobian_node",
        output="screen",
        parameters=[config],
    )

    log_node = Node(
        package="sensor_observability_analysis_py",
        executable="log_test_node",
        name="log_test_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([
        soa_node

    ])

#ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5], points: [{positions: [-0.35, 0.25, 0.1, 0.1, 1.5, 0.0], time_from_start: {sec: 1}}]}}"
