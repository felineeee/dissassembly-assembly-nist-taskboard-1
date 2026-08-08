import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package="project_description").find(
        "project_description"
    )

    default_model_path = os.path.join(pkg_share, "urdf", "sam_bot", "sam_bot.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "nav2.rviz")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="gui", default_value="True"),
            DeclareLaunchArgument(name="model", default_value=default_model_path),
            DeclareLaunchArgument(
                name="rvizconfig", default_value=default_rviz_config_path
            ),
            robot_state_publisher_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
