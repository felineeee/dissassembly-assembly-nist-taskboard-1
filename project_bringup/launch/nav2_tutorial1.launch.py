import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# Navigating with a Physical Turtlebot 3
def generate_launch_description():
    pkg_share = FindPackageShare(package="project_description").find(
        "project_description"
    )
    nav2_bringup_path = get_package_share_directory("nav2_bringup")
    turtlebot3_bringup_path = get_package_share_directory("turtlebot3_bringup")

    turtlebot3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_path, "launch", "robot.launch.py")
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_path, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": os.path.join(
                pkg_share, "config", "sam_bot", "nav2_params.yaml"
            ),
        }.items(),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(nav2_bringup_path, "rviz", "nav2_default_view.rviz"),
        ],
    )

    return LaunchDescription(
        [
            turtlebot3_launch,
            nav2_launch,
            rviz_node,
        ]
    )
