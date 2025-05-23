import os
import subprocess
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution, FindExecutable
from launch_ros.actions import Node


def stop_lidar(context: LaunchContext):
    ros2_exec = FindExecutable(name='ros2').perform(context)
    subprocess.run([ros2_exec, "service", "call", "/stop_motor", "std_srvs/srv/Empty"], check=True)


def launch_setup(context: LaunchContext):
    pkg = 'turbopi_ros'
    sim = context.perform_substitution(LaunchConfiguration('sim')).lower() in ['true', '1', 'yes']
    path = get_package_share_directory(pkg)
    xacro_file = os.path.join(path, 'description', 'turbopi.urdf.xacro')
    ctrl = os.path.join(path, 'config', 'turbopi_controllers.yaml')
    slam = os.path.join(path, 'config', 'slam_toolbox.yaml')

    # Generate robot_description via xacro
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        TextSubstitution(text=" "),
        TextSubstitution(text=xacro_file),
        TextSubstitution(text=" "),
        TextSubstitution(text=f"use_hardware:={'mock' if sim else 'robot'}"),
    ])
    robot_description = {
        "robot_description": robot_description_content.perform(context)
    }

    # Core nodes
    cm = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[robot_description, ctrl], output='screen'
    )
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description], output='screen'
    )

    # Controller spawners
    joint_spawn = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )
    mecanum_spawn = Node(
        package='controller_manager', executable='spawner',
        arguments=['mecanum_drive_controller', '-c', '/controller_manager'],
        output='screen'
    )
    pos_spawn = Node(
        package='controller_manager', executable='spawner',
        arguments=['position_controllers', '-c', '/controller_manager'],
        output='screen'
    )

    # Sensor nodes
    bat = Node(package=pkg, executable='battery_node')
    ir = Node(package=pkg, executable='infrared_node')
    sn = Node(package=pkg, executable='sonar_node')

    # SLAM
    slam_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        parameters=[slam, {'use_sim_time': sim}],
        output='screen'
    )

    # Start LiDAR
    start = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'), "service", "call", "/start_motor", "std_srvs/srv/Empty"],
        output='screen'
    )

    # Stop LiDAR when SLAM exits
    stop_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=slam_node,
            on_exit=[
                LogInfo(msg="SLAM exited — stopping LiDAR..."),
                OpaqueFunction(function=stop_lidar),
                LogInfo(msg="LiDAR stopped."),
            ],
        )
    )

    # Launch order
    return [
        bat, ir, sn, cm, rsp,
        RegisterEventHandler(OnProcessStart(target_action=cm, on_start=[joint_spawn])),
        RegisterEventHandler(OnProcessExit(target_action=joint_spawn, on_exit=[mecanum_spawn])),
        RegisterEventHandler(OnProcessExit(target_action=mecanum_spawn, on_exit=[pos_spawn])),
        RegisterEventHandler(OnProcessExit(target_action=pos_spawn, on_exit=[start, slam_node])),
        stop_handler,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='False', description='Use mock hardware'),
        OpaqueFunction(function=launch_setup),
    ])
