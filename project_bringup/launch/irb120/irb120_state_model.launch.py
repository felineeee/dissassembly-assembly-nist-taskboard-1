from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    project_description_path = get_package_share_directory("project_description")

    moveit_config = (
        MoveItConfigsBuilder("irb120_gripper", package_name="project_description")
        .robot_description(
            file_path="urdf/irb120_gripper/irb120_gripper.xacro",
        )
        .robot_description_semantic(
            file_path="urdf/irb120_gripper/irb120_gripper.srdf",
        )
        .joint_limits(file_path="config/irb120_3_58/irb120_joint_limits.yaml")
        .robot_description_kinematics(
            file_path="config/irb120_3_58/irb120_kinematics.yaml"
        )
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/irb120_3_58/irb120_moveit_controllers.yaml",
            moveit_manage_controllers=True,
        )
        .to_moveit_configs()
    )

    moveit_params = moveit_config.to_dict()

    moveit_params.update(
        {
            "planning_scene_monitor_options": {
                "name": "planning_scene_monitor",
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            }
        }
    )

    return LaunchDescription(
        [
            Node(
                package="irb1200_moveit",
                executable="model_and_state_service",
                name="model_and_state_service",
                output="screen",
                parameters=[moveit_params],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    "moveit_ros_move_group:=debug",
                    "--log-level",
                    "moveit_ros_planning_interface:=debug",
                    "--log-level",
                    "moveit_core:=debug",
                    "--log-level",
                    "ompl:=debug",
                ],
            ),
        ]
    )
