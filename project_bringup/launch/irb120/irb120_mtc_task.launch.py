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

    mtc_node = Node(
        package="project_moveit",
        executable="planning_mtc",
        name="task_constructor",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([mtc_node])
