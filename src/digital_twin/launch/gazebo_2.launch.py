import os
from pathlib import Path
from unicodedata import digit
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    digital_twin = get_package_share_directory("digital_twin")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        digital_twin, "urdf", "so101.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    urdf_path = os.path.join(digital_twin, 'urdf', 'so101.urdf.xacro')
    
    config_path = os.path.join(digital_twin, 'config', 'so101_config.yaml')
    robot_description_content = Command(['xacro ', urdf_path])

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
                   "-name", "so101",
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

    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/ft_sensor_bd/wrench@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'
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

    ft_sensor_bd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_sensor_bd',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout','30'
                   ],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        ft_sensor_bd,
        bridge
        #rviz_node
    ])

'''
rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(digital_twin, "rviz", "so101.rviz")],
        parameters=[{"use_sim_time": True}]
    )
'''