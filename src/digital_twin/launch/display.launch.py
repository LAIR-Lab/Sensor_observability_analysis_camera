from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory 


def generate_launch_description():
    model_arg = DeclareLaunchArgument(name='model', 
                                      default_value=os.path.join(get_package_share_directory('digital_twin'), 'urdf', 'so_finale.urdf.xacro'),
                                      description='Absolute path to robot urdf file'
                                      )
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
         package="joint_state_publisher_gui",
         executable="joint_state_publisher_gui",
         output="screen",
         parameters=[
             {
                 "robot_description": robot_description,
                 #"use_sim_time": LaunchConfiguration("use_sim_time")
            }
         ]
     )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('digital_twin'), 'rviz', 'display.rviz')]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])

'''
Add this later:

    camera_node = Node(
        package='camera_state_publisher',
        executable='camera_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
'''