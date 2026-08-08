import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    project_description_path = get_package_share_directory("project_description")
    moveit_config = (
        MoveItConfigsBuilder("irb1200_5_90", package_name="project_description")
        .robot_description(
            file_path=os.path.join(
                project_description_path, "urdf", "irb1200_5_90.urdf.xacro"
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(
                project_description_path, "urdf", "irb1200_5_90.srdf"
            )
        )
        .joint_limits(
            file_path=os.path.join(
                project_description_path, "config", "joint_limits.yaml"
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                project_description_path, "config", "kinematics.yaml"
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                project_description_path, "config", "moveit_irb1200_5_90.yaml"
            )
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            Node(
                package="my_moveit",
                executable="motion_planning_api",
                name="motion_planning_api",
                output="screen",
                parameters=[moveit_config.to_dict()],
            )
        ]
    )
