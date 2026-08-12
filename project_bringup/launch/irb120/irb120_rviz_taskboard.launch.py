import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    project_description_path = get_package_share_directory("project_description")

    # IRB120 + 2fg7 (Reach issue)
    # Migrating to ABB1200 due to reach issue
    # moveit_config = (
    #     MoveItConfigsBuilder("irb120_assembly", package_name="project_description")
    #     .robot_description(file_path="urdf/irb120_3_58/irb120_3_58_world.xacro")
    #     .robot_description_semantic(file_path="urdf/irb120_gripper/irb120_gripper.srdf")
    #     .joint_limits(file_path="config/irb120_3_58/irb120_joint_limits.yaml")
    #     .robot_description_kinematics(
    #         file_path="config/irb120_3_58/irb120_kinematics.yaml"
    #     )
    #     .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
    #     .to_moveit_configs()
    # )

    # IRB1200 + 2fg7 (Gripper too big)
    # moveit_config = (
    #     MoveItConfigsBuilder("irb120_assembly", package_name="project_description")
    #     .robot_description(
    #         file_path="urdf/irb120_3_58/irb120_3_58_world.xacro"
    #     )  # Let this be for now
    #     .robot_description_semantic(file_path="urdf/irb120_gripper/irb120_gripper.srdf")
    #     .joint_limits(file_path="config/irb1200_5_90/irb1200_joint_limits.yaml")
    #     .robot_description_kinematics(
    #         file_path="config/irb1200_5_90/irb1200_kinematics.yaml"
    #     )
    #     .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
    #     .to_moveit_configs()
    # )

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

    moveit_params = moveit_config.to_dict()

    moveit_params.update(
        {
            "use_sim_time": False,
            "capabilities": "move_group/ExecuteTaskSolutionCapability",
            "planning_scene_monitor": {
                "publish_geometry_updates": False,
                "publish_planning_scene_frequency": 1.0,
            },
            "trajectory_execution": {
                "allowed_execution_duration_scaling": 2.0,
                "allowed_goal_duration_margin": 0.5,
                "execution_duration_monitoring": False,
                "allowed_start_tolerance": 0.05,
            },
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_simple_controller_manager": {
                "controller_names": [
                    "irb120_arm_controller",
                    "onrobot_2fg7_controller",
                ],
                "irb120_arm_controller": {
                    "type": "FollowJointTrajectory",
                    "action_ns": "follow_joint_trajectory",
                    "default": True,
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
                    "type": "FollowJointTrajectory",
                    "action_ns": "follow_joint_trajectory",
                    "default": True,
                    "joints": ["gripper_gripper_joint"],
                },
            },
        }
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
            os.path.join(project_description_path, "rviz", "without_pcl.rviz"),
        ],
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # 3. MoveGroup
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_sliders,
            move_group_node,
            rviz,
        ]
    )
