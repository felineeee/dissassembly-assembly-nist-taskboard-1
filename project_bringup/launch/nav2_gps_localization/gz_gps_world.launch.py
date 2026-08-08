import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
import xacro
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node


# @warning this launch has error on the world.sdf, most likely on how it depend on some online resource
def generate_launch_description():
    description_pkg_path = get_package_share_directory("project_description")
    ros_gz_sim_pkg_path = get_package_share_directory("ros_gz_sim")
    turtlebot3_pkg_path = get_package_share_directory("nav2_minimal_tb3_sim")

    # gz_sim
    gz_launch_path = PathJoinSubstitution(
        [ros_gz_sim_pkg_path, "launch", "gz_sim.launch.py"]
    )

    # world
    # Saving to temp because it expect path
    world_sdf_path = os.path.join(
        description_pkg_path, "world", "sonoway", "sonoway.sdf.xacro"
    )
    world_sdf_config = xacro.process_file(world_sdf_path)
    temp_world_sdf = "/tmp/processed_sonoway.sdf"
    with open(temp_world_sdf, "w") as f:
        f.write(world_sdf_config.toxml())

    print(f"World file: {temp_world_sdf}")

    # robot
    turtlebot3_gz_launch = PathJoinSubstitution(
        [turtlebot3_pkg_path, "launch", "spawn_tb3_gps.launch.py"]
    )
    turtlebot3_sdf = PathJoinSubstitution(
        [turtlebot3_pkg_path, "urdf", "gz_waffle_gps.sdf.xacro"]
    )

    #   urdf_object
    turtlebot3_urdf = PathJoinSubstitution(
        [turtlebot3_pkg_path, "urdf", "turtlebot3_waffle_gps.urdf"]
    )
    #   urdf_path
    turtlebot3_urdf_path = os.path.join(
        turtlebot3_pkg_path, "urdf", "turtlebot3_waffle_gps.urdf"
    )
    with open(turtlebot3_urdf_path, "r") as infp:
        turtlebot3_urdf_content = infp.read()

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            # GZ GUI and server combined
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch_path),
                launch_arguments={"gz_args": f"-v4 -r {temp_world_sdf}"}.items(),
                # launch_arguments={"gz_args": f"-v4 -r"}.items(),  # blank gz launch
            ),
            # Turtlebot3 launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(turtlebot3_gz_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "robot_sdf": turtlebot3_sdf,
                    "x_pose": "2.0",
                    "y_pose": "-2.5",
                    "z_pose": "0.33",
                    "roll": "0.0",
                    "pitch": "0.0",
                    "yaw": "0.0",
                }.items(),
            ),
            # Robot state publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": turtlebot3_urdf_content,
                    }
                ],
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
        ]
    )
