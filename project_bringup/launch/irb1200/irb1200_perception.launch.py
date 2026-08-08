import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory("ros_gz_sim")
    project_description_pkg_path = FindPackageShare("project_description")

    return LaunchDescription(
        [
            Node(
                package="irb1200_perception",
                executable="perception_server",
                name="perception_server",
                output="screen",
            ),
            Node(
                package="irb1200_perception",
                executable="dummy_node",
                name="dummy_node",
                output="screen",
            ),
        ]
    )
