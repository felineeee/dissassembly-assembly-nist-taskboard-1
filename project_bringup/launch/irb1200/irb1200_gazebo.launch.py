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
    gz_launch_path = PathJoinSubstitution(
        [ros_gz_sim_pkg_path, "launch", "gz_sim.launch.py"]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                PathJoinSubstitution([project_description_pkg_path, ".."]),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH",
                PathJoinSubstitution([project_description_pkg_path, "plugins"]),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch_path),
                launch_arguments={
                    "gz_args": [
                        PathJoinSubstitution(
                            [
                                project_description_pkg_path,
                                "urdf/irb1200_5_90/irb1200_5_90.sdf",
                            ]
                        )
                    ],
                    "on_exit_shutdown": "True",
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    # 1. The Point Cloud
                    "/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    # 2. The Color Image
                    "/rgbd_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                    # 3. The Camera Calibration Metadata
                    "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    # 4. The Depth Image
                ],
                # remappings=[
                #     ("/rgbd_camera/points", "/camera/points"),
                #     ("/rgbd_camera", "/camera/image"),
                #     ("/rgbd_camera/camera_info", "/camera/camera_info"),
                #     ("/rgbd_camera/depth_image", "/camera/depth_image"),
                # ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
            ),
        ]
    )
