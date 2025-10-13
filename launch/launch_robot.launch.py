import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot'

    # 1. Inicia o RSP e outros nós independentes
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

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
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # 2. O "Gatilho Sentinela" que espera pelo /robot_description
    wait_for_topic = ExecuteProcess(
        cmd=['bash', '-c',
             'while [ -z "$(ros2 topic list | grep /robot_description)" ]; do sleep 1; done'],
        name='wait_for_robot_description',
        output='screen'
    )

    # 3. Definição dos nós do ros2_control
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file]
    )

    diff_drive_spawner = Node(package="controller_manager", executable="spawner", arguments=["diff_cont"])
    joint_broad_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_broad"])

    # 4. A LÓGICA EM CASCATA CORRIGIDA

    # Gatilho 1: Inicia o controller_manager QUANDO o sentinela (wait_for_topic) TERMINA
    start_cm_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_topic,
            on_exit=[controller_manager],
        )
    )

    # Gatilho 2: Inicia os spawners QUANDO o controller_manager COMEÇA
    start_spawners_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                diff_drive_spawner,
                joint_broad_spawner
            ],
        )
    )

    # 5. Lista final de ações
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        wait_for_topic,
        start_cm_handler,
        start_spawners_handler
    ])