from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory # to get package share directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        name ='model',
        default_value=os.path.join(get_package_share_directory('bumperbot_description'), 'urdf', 'bot.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    # Process the xacro file
    # Use Command substitution to call xacro and get the robot description
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration('model')]), value_type=str) 

    # Create a robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description}]
    )

    # Create a joint_state_publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    #rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('bumperbot_description'), 'rviz', 'model.rviz')]
    )

    return LaunchDescription([
        # Add launch actions here
        model_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])