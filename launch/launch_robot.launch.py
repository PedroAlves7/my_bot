import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot'

    # 1. Inicia o Robot State Publisher a partir do seu launch file dedicado (rsp.launch.py)
    # Este é o nó lento que precisa terminar primeiro.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Inicia o joystick e o twist_mux em paralelo, pois não têm dependências críticas
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

    # 2. Define o Controller Manager, mas NÃO o inicia ainda.
    # Esta é a versão CORRIGIDA, sem o parâmetro 'robot_description'.
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file]  # <-- A correção crucial está aqui!
    )

    # Define os spawners, mas também NÃO os inicia ainda.
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

    # 3. Lógica de Atraso com Event Handlers
    # Inicia o controller_manager SOMENTE APÓS o robot_state_publisher ter sido iniciado.
    # Damos a ele a chance de publicar o /robot_description.
    delayed_controller_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rsp,  # Gatilho: o nó dentro do rsp.launch.py
            on_start=[controller_manager],
        )
    )

    # Inicia o diff_drive_spawner SOMENTE APÓS o controller_manager ter sido iniciado.
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    # Inicia o joint_broad_spawner SOMENTE APÓS o controller_manager ter sido iniciado.
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Retorna a lista de ações a serem executadas
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])