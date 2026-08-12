from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("irb1200_assembly", package_name="project_description")
        .robot_description(
            file_path="urdf/irb1200_5_90/irb1200_5_90_world.xacro"
        )  # Let this be for now
        .robot_description_semantic(file_path="urdf/2f_gripper/2f_gripper.srdf")
        .joint_limits(file_path="config/irb1200_5_90/irb1200_joint_limits.yaml")
        .robot_description_kinematics(
            file_path="config/irb1200_5_90/irb1200_kinematics.yaml"
        )
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    task_constructor_node = Node(
        package="project_moveit",
        executable="task_constructor_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription([task_constructor_node])
