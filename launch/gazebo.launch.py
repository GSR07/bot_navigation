from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
from pathlib import Path


def generate_launch_description():
    # Get package share dir
    pkg_share = get_package_share_directory("bumperbot_description")

    # URDF/Xacro argument
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_share, "urdf", "bot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    # Process xacro -> robot_description
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    # Ensure Gazebo finds meshes
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_RESOURCE_PATH",
        value=str(Path(pkg_share).parent.resolve())  # .../share
    )


    # Launch Gazebo (Ignition / Fortress in Humble)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", [" -v 4 -r empty.sdf"])]
    )

    # Spawn entity from robot_description
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "bumperbot",
            "-z", "0.05"  # Slightly above ground to avoid collision issues
        ]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity
    ])
