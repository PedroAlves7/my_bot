import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot'

    # 1. Inclui o seu rsp.launch.py, que inicia o robot_state_publisher (lento)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Nós que podem iniciar em paralelo sem problemas
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

    # 2. O "Gatilho Sentinela"
    # Este processo espera ativamente pelo tópico /robot_description e só termina quando o encontra.
    wait_for_topic = ExecuteProcess(
        cmd=['bash', '-c',
             'while [ -z "$(ros2 topic list | grep /robot_description)" ]; do echo "Aguardando pelo tópico /robot_description..."; sleep 1; done && echo "Tópico /robot_description encontrado!"'],
        name='wait_for_robot_description',
        output='screen'
    )

    # 3. Definição dos nós do ros2_control (sem iniciá-los ainda)
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # 4. O EventHandler que inicia o ros2_control
    # Ele é acionado QUANDO o processo "sentinela" (wait_for_topic) TERMINA COM SUCESSO.
    start_ros2_control_when_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_topic,
            on_exit=[
                controller_manager,
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
        start_ros2_control_when_ready
    ])