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
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    moveit_params = moveit_config.to_dict()

    moveit_params.update(
        {
            "use_sim_time": False,
            "moveit_controller_manager": "moveit_fake_controller_manager/MoveItFakeControllerManager",
            "moveit_fake_controller_manager": {
                "controller_names": [
                    "irb120_arm_controller",
                    "onrobot_2fg7_controller",
                ],
                "irb120_arm_controller": {
                    "type": "interpolate",
                    "joints": [
                        "joint_1",
                        "joint_2",
                        "joint_3",
                        "joint_4",
                        "joint_5",
                        "joint_6",
                    ],
                },
                "onrobot_2fg7_controller": {
                    "type": "interpolate",
                    "joints": ["gripper_gripper_joint"],
                },
            },
        }
    )

    # Robot State Publisher (Publishes TF tree from robot_description)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Joint State Publisher (Standard non-GUI node so MTC can control joint state)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    # Scene Loader
    scene_loader_node = Node(
        package="project_perception",
        executable="scene_loader",
        name="scene_loader",
        output="screen",
    )

    # RViz2 Node (Updated to moveit_params)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(project_description_path, "rviz", "without_pcl.rviz"),
        ],
        output="screen",
        parameters=[moveit_params],
    )

    # MTC Node
    mtc_task_node = Node(
        package="project_moveit",
        executable="planning_mtc",
        name="task_constructor",
        output="screen",
        parameters=[moveit_params],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
            move_group_node,
            scene_loader_node,
            rviz,
            mtc_task_node,
        ]
    )
