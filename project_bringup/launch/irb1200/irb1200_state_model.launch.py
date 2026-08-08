from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory


# Defining our own config trough SRDF, URDF and Kinematics files
def generate_launch_description():
    project_description_path = get_package_share_directory("project_description")
    moveit_config = (
        MoveItConfigsBuilder("irb1200_5_90", package_name="project_description")
        .robot_description(
            file_path=os.path.join(
                project_description_path, "urdf", "irb1200_5_90.xacro"
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
        .sensors_3d(None)
        # .sensors_3d(
        #     file_path=os.path.join(project_description_path, "config", "sensors.yaml")
        # )
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    move_group_config = MoveItConfigsBuilder
    moveit_params = moveit_config.to_dict()
    moveit_params["planning_pipelines"] = ["ompl"]
    
    # Force boolean values for scene publishing
    moveit_params["publish_planning_scene"] = True
    moveit_params["publish_geometry_updates"] = True
    moveit_params["publish_state_updates"] = True
    moveit_params["publish_transforms_updates"] = True

    if "ompl" in moveit_params:
        moveit_params["ompl"]["planning_plugins"] = ["ompl_interface/OMPLPlanner"]

    return LaunchDescription(
        [
            Node(
                package="irb1200_moveit",
                executable="robot_model_and_state_service",
                name="robot_model_and_state_service",
                output="screen",
                parameters=[moveit_params],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_params],
            ),
        ]
    )
