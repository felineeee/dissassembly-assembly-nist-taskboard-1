import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Automatically find the path inside your package
    project_description_path = get_package_share_directory("project_description")
    default_yaml_path = os.path.join(
        project_description_path, "config", "taskboard1", "taskboard_v2.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="irb1200_perception",
                executable="scene_loader",
                name="scene_loader",
                output="screen",
                parameters=[{"scene_yaml_path": default_yaml_path}],
            )
        ]
    )
