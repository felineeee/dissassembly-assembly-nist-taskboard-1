
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    robot_path = get_package_share_directory("abb_irb1200_support")

    robot_description_file = xacro.process_file(
        os.path.join(
            robot_path,
            "urdf",
            "irb1200_5_90.xacro"
        ),
        mappings={
            "use_fake_hardware": "true",
            "use_gazebo": "false"
        }
    )
    robot_config_file = os.path.join(robot_path, "config", "joint_names_irb1200_5_90.yaml")

    robot_description = {"robot_description": robot_description_file.toxml()}
    robot_config = {"robot_config": robot_config_file}

    controller_manager_node = Node(
        package = "controller_manager",
        executable = "ros2_control_node",
        parameters = [robot_description, robot_config_file, {"use_sim_time": False}],
    )
    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "both",
        parameters = [robot_description, {"use_sim_time": False}],
    )
    rviz2_node = Node(
        package = "rviz2",
        executable = "rviz2",
        output = "log",
        arguments = ["-d", os.path.join(robot_path, "rviz", "urdf_description.rviz")],
        parameters = [{"use_sim_time": False}]
    )

    joint_state_broadcaster_node = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters = [{"use_sim_time": False}]
    )
    forward_position_controller_node = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "position_control",
            "--controller-manager", "/controller_manager",
            "--parameter-file",
            os.path.join(robot_path, "config", "joint_names_irb1200_5_90.yaml"),
        ],
        parameters = [{"use_sim_time": False}]
    )

    return LaunchDescription([
        controller_manager_node,
        robot_state_publisher_node,
        rviz2_node,
        joint_state_broadcaster_node,
        forward_position_controller_node
    ])



