import os
from pathlib import Path
from unicodedata import digit
from ament_index_python.packages import get_package_share_directory
import random

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    digital_twin = get_package_share_directory("digital_twin")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        digital_twin, "urdf", "so_finale.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    send_goal = ExecuteProcess(
        cmd=[
            "ros2", "action", "send_goal",
            "/arm_controller/follow_joint_trajectory",
            "control_msgs/action/FollowJointTrajectory",
            """{
                trajectory: {
                    joint_names: [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5],
                    points: [{
                        positions: [-0.1, 0.25, -0.3, 0.35, 0.2, 0.0],
                        time_from_start: {sec: 1}
                    }]
                }
            }"""
        ],
        output="screen"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(digital_twin).parent.resolve())
            ]
        )
    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4 -r empty.sdf"]
                    )
                ]
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "so",
                   "-x", "0",
                   "-y", "0",
                   "-z", "0"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera1/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera2/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera3/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            "/camera1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera2/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera3/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30"
        ],
        parameters=[{"use_sim_time": True}]
    )
    
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30"
        ],
    )

    orbit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "orbit_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30"
        ],
        parameters=[{"use_sim_time": True}]
    )

    # Repeats at 10 Hz so it doesn't matter if orbit_controller is still
    # activating when this starts firing
    orbit_spin_cmd = TimerAction(
        period=7.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub", "-r", "10",
                "/orbit_controller/commands",
                "std_msgs/msg/Float64MultiArray",
                "{data: [0.5]}"
            ],
            output="screen"
        )]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(digital_twin, "rviz", "display.rviz")],
        parameters=[{"robot_description": robot_description,
                      "use_sim_time": True}]
    )

# to spawn : ros2 run ros_gz_sim create -world default -file /path/to/shape.sdf -name my_box -x 2.0 -y 0.0 -z 1.0

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        gz_bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        orbit_controller_spawner,
        rviz_node,
        TimerAction(period=5.0, actions=[send_goal]),
        orbit_spin_cmd
    ])


'''
    spawn_shape = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-world', 'default',
        '-file', sdf_file,
        '-name', 'custom_box',
        '-x', '1.0', '-y', '2.0', '-z', '0.1'
    ],
    output='screen'
)
'''

#just a demo: ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5], points: [{positions: [0.0, 1.0, 1.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
# default : ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5], points: [{positions: [-0.1, 0.25, -0.3, 0.35, 0.2, 0.0], time_from_start: {sec: 1}}]}}"