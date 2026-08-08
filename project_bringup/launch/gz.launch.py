import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, IncludeLaunchDescription

def generate_launch_description():
    xacro_path = 'urdf/irb1200_5_90.xacro'

    robot_description = PathJoinSubstitution([
        get_package_share_directory('abb_irb1200_support'),
        xacro_path
    ])

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'screen',
        parameters= [{
                'robot_description':Command(['xacro ', robot_description, ' use_gazebo:=true']),
                'use_sim_time': True
            }]
    )
    
    spawn_node = Node(package='ros_gz_sim', executable='create',
                      arguments = [
                          '-name', 'abb_irb1200_5_90',
                          '-topic', '/robot_description'
                          ], output = 'screen'
                      )
    
    ignition_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments=[('gz_args', ' -r -v 4 empty.sdf')]
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'
    )
    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'position_control'], output='screen'
    )
    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'velocity_control'], output='screen'
    )
    load_sinu_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'sine_controller'], output='screen'
    )

    pkg_share_path = os.path.join(get_package_share_directory('abb_irb1200_support'), '..')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + pkg_share_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = pkg_share_path

    return LaunchDescription([
        ignition_gazebo_node,
        robot_state_publisher_node,
        spawn_node,
        bridge_node,
        load_joint_state_broadcaster,
        load_position_controller,
        load_velocity_controller,
        load_sinu_controller
    ])

