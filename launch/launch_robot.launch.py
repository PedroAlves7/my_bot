import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package.
    # Force sim time to be enabled
    package_name = 'my_bot'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','joystick.launch.py'
        )])
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # This node waits for the /robot_description topic to be published.
    # We use 'ros2 topic echo --once' which exits after the first message,
    # making it a reliable trigger for the next action.
    wait_for_robot_description = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', '/robot_description'],
        name='wait_for_robot_description',
        output='screen'
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    # We use OnProcessExit to start the controller_manager only after
    # the /robot_description topic has been published, ensuring the
    # parameter is available. This is more robust than a fixed TimerAction.
    start_controller_manager_when_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_robot_description,
            on_exit=[controller_manager],
        )
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # Use OnProcessStart to ensure the spawner only runs when the controller_manager is active.
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Use OnProcessStart to ensure the spawner only runs when the controller_manager is active.
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        wait_for_robot_description,
        start_controller_manager_when_ready,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])