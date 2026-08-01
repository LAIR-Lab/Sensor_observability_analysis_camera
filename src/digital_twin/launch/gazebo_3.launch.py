import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
    LogInfo,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    """Setup function to handle dynamic configuration"""
    model = LaunchConfiguration("model").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    
    return [
        LogInfo(msg=f"[Launch] Loading URDF from: {model}"),
        LogInfo(msg=f"[Launch] Using simulation time: {use_sim_time}"),
    ]


def generate_launch_description():
    """Generate launch description for ROS2 Jazzy with Gazebo Harmonic simulator"""
    
    digital_twin_dir = get_package_share_directory("digital_twin")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            digital_twin_dir, 
            "urdf", 
            "so_finale_urdf_FINAL.xacro"  # Use version WITHOUT YAML in URDF
        ),
        description="Absolute path to robot URDF/XACRO file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(digital_twin_dir).parent.resolve())]
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use simulation (Gazebo) clock if true"
    )
    
    verbose_arg = DeclareLaunchArgument(
        name="verbose",
        default_value="false",
        choices=["true", "false"],
        description="Enable verbose output"
    )
    
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="empty",
        description="Gazebo world to load (empty, camera_world, etc.)"
    )
    
    robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="digital_twin",
        description="Name of the robot entity in Gazebo"
    )
    
    gazebo_model_path = SetEnvironmentVariable(
        name="GZ_SIM_MODEL_PATH",
        value=[
            os.path.join(digital_twin_dir, "models"),
            str(Path(digital_twin_dir).parent.resolve())
        ]
    )
    
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish_frequency": 100.0,
                "frame_prefix": ""
            }
        ]
    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "publish_frequency": 50.0,
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }
        ]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[
            ("gz_args", [
                LaunchConfiguration("verbose"),
                " -r ",
                LaunchConfiguration("world"),
                ".sdf"
            ])
        ]
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", LaunchConfiguration("robot_name"),
            "-x", "0",
            "-y", "0",
            "-z", "0.5",
            "-R", "0",
            "-P", "0",
            "-Y", "0"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    gz_ros2_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    gz_ros2_bridge_camera = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    # ===== KEY CHANGE: Pass YAML config via launch file instead of plugin =====
    config_file = os.path.join(digital_twin_dir, "config", "ros2_config.yaml")
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30"
        ],
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30"
        ],
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"controller_config_file": config_file}  # Pass YAML here
        ]
    )
    
    return LaunchDescription([
        gazebo_resource_path,
        gazebo_model_path,
        
        model_arg,
        use_sim_time_arg,
        verbose_arg,
        world_arg,
        robot_name_arg,
        
        OpaqueFunction(function=launch_setup),
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        
        gazebo,
        gz_spawn_entity,
        
        gz_ros2_bridge_clock,
        gz_ros2_bridge_camera,
        
        RegisterEventHandler(
            OnProcessStart(
                target_action=gz_spawn_entity,
                on_start=[
                    LogInfo(msg="Robot spawned, starting controllers..."),
                    joint_state_broadcaster_spawner,
                    arm_controller_spawner,
                ]
            )
        ),
    ])