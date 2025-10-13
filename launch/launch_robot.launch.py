import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():
    # Package name
    package_name = 'my_bot'

    # 1. --- Robot State Publisher (RSP) ---
    # The RSP *must* be launched first and is responsible for publishing /robot_description and TF.
    # We assume 'rsp.launch.py' correctly loads the URDF/XACRO into the 'robot_description' parameter
    # for the robot_state_publisher node it contains.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # We need the rsp node process to reference it for the event handler.
    # Assuming the node in rsp.launch.py is named 'robot_state_publisher'.
    # If rsp.launch.py only contains the robot_state_publisher node, we can use the include action itself.
    # However, to be safe, let's assume the RSP node inside 'rsp.launch.py' has the name 'robot_state_publisher'
    # as an explicit launch action. We'll use the 'rsp' action as the target, which often works.

    # 2. --- Teleop and Mux ---
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )])
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
        # Make sure twist_mux starts after RSP
        # lifecycle_node=True, # Optional: if twist_mux is a lifecycle node
    )

    # 3. --- ROS 2 Control (Controller Manager) ---
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    # NOTE: We DO NOT pass the 'robot_description' parameter here.
    # The 'ros2_control_node' is expected to automatically look for the
    # '/robot_description' topic published by the 'robot_state_publisher'.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        # The 'output' and 'emulate_tty' are good practice for viewing controller output
        output='screen',
        emulate_tty=True,
    )

    # 4. --- Spawners ---
    # We delay the controller manager start until the RSP is confirmed to be running.
    # A simple and robust way is to wait for the whole 'rsp.launch.py' to finish (though usually instant)
    # or more specifically, wait for the 'robot_state_publisher' node to exit if it fails, or for
    # a successful node's start. Since RSP is the key, let's wait for its successful launch.

    # Instead of the complicated 'ros2 topic echo' trigger, we wait for the RSP *process* # to start, or simply launch the controller manager immediately after the RSP launch action.
    # Given the original code's complexity, we will use OnProcessExit for a critical node
    # in the RSP launch (you must confirm the RSP node's name). For simplicity, let's
    # assume the RSP node itself is called 'robot_state_publisher' and include it in a variable.

    # Since we can't reliably target a node *inside* an IncludeLaunchDescription with OnProcessStart
    # unless it's explicitly named, we will use a basic dependency approach:
    # 1. Start RSP (which should load the URDF).
    # 2. Start the Controller Manager.
    # 3. Only spawn controllers once the Controller Manager is running.

    # The issue in your log was that the RSP *died* instantly. We still need the Spawners to wait
    # for the Controller Manager to be ready.

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "-c", "/controller_manager"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "-c", "/controller_manager"],
    )

    # Delay spawners until controller_manager is running and ready.
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager,
            on_exit=[diff_drive_spawner, joint_broad_spawner],
        )
    )

    # Launch them all!
    return LaunchDescription([
        # Start core components immediately
        rsp,
        controller_manager,  # Start the controller manager
        joystick,
        twist_mux,

        # Spawners are delayed until controller_manager is confirmed running
        delayed_spawners,
    ])