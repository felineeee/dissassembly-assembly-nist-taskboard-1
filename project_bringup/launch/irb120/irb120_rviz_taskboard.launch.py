import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    project_description_path = get_package_share_directory("project_description")

    moveit_config = (
        MoveItConfigsBuilder("irb120_assembly", package_name="project_description")
        .robot_description(file_path="urdf/irb120_3_58/irb120_3_58_world.xacro")
        .robot_description_semantic(file_path="urdf/irb120_gripper/irb120_gripper.srdf")
        .joint_limits(file_path="config/irb120_3_58/irb120_joint_limits.yaml")
        .robot_description_kinematics(
            file_path="config/irb120_3_58/irb120_kinematics.yaml"
        )
        .sensors_3d(None)
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 1. Robot State Publisher (Publishes TF tree from robot_description)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 2. Joint State Publisher GUI (Manual Sliders for RViz visualization)
    joint_state_sliders = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # 3. RViz2 Node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(project_description_path, "rviz", "with_pcl.rviz"),
        ],
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([robot_state_publisher_node, joint_state_sliders, rviz])
