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


# Navigating while Mapping (SLAM)
# Store saved map:
#     ros2 run nav2_map_server map_saver_cli -f ~/map


def generate_launch_description():
    pkg_share = FindPackageShare(package="project_description").find(
        "project_description"
    )
    nav2_bringup_path = get_package_share_directory("nav2_bringup")
    turtlebot3_bringup_path = get_package_share_directory("turtlebot3_bringup")
    slam_toolbox_bringup_path = get_package_share_directory("slam_toolbox")

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
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_bringup_path, "launch", "online_async_launch.py")
        )
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
    send_goal_command = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "-1",
            "/goal_pose",
            "geometry_msgs/msg/PoseStamped",
            '{header: {frame_id: "map"}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}',
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            turtlebot3_launch,
            nav2_launch,
            slam_toolbox_launch,
            rviz_node,
            send_goal_command,
        ]
    )
