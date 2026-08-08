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
                                "urdf/irb120_3_58/irb120_3_58.sdf",
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
                    # Overhead Camera Bridge
                    "overhead_camera/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "overhead_camera/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "overhead_camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "overhead_camera/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    # Wrist Camera Bridge
                    "wrist_camera/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "wrist_camera/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "wrist_camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "wrist_camera/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
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
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",  # THE ROTATION (Gazebo X-forward -> ROS Z-forward)
                    "base_link",
                    "abb_irb120_3_58/wrist_camera_link/rgbd_wrist",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",  # THE ROTATION (Gazebo X-forward -> ROS Z-forward)
                    "base_link",
                    "overhead_camera/overhead_link/rgbd_overhead",
                ],
            ),
            # Rviz, deactivate for now
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="screen",
            #     arguments=[
            #         "-d",
            #         os.path.join(
            #             project_description_pkg_path, "rviz", "planning_scene.rviz"
            #         ),
            #     ],
            # ),
        ]
    )
