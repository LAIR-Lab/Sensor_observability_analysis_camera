from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="sensor_observability_analysis_py",
            executable="sensor_geometry_node"
        ),

        Node(
            package="sensor_observability_analysis_py",
            executable="soa_solver_node"
        ),

        Node(
            package="sensor_observability_analysis_py",
            executable="soa_visualizer_node"
        )

    ])