import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('digital_twin')
    urdf_path = os.path.join(pkg_share, 'urdf', 'camera.urdf.xacro')

    # Read URDF content
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 2. Gazebo Sim Launch Entry (Using empty.sdf as basic staging layout)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. Robot State Publisher to parse TF frames
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. Spawner Node to push the URDF entity into Gazebo Simulation
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc, '-name', 'camera_cube', '-z', '0.1']
    )

    # 5. Topic Bridge Mapping (Bridges Gazebo Sim Images to ROS 2)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cube/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        gz_spawn_entity,
        gz_bridge
    ])
