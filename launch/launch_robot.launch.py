import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot'

    # =============================================================================
    # ETAPA 1: DEFINIR TODOS OS NÓS
    # =============================================================================

    # --- Argumentos de Lançamento ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # --- Robot State Publisher ---
    # Processa o XACRO para obter a descrição do robô
    xacro_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'robot.urdf.xacro')
    robot_description_config = Command(
        ['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    # Define o nó do robot_state_publisher com a descrição do robô
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': use_sim_time}]
    )

    # --- Outros Nós ---
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )])
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # --- Nós do ROS2 Control (sem o robot_description) ---
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file]
    )

    diff_drive_spawner = Node(package="controller_manager", executable="spawner", arguments=["diff_cont"])
    joint_broad_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_broad"])

    # =============================================================================
    # ETAPA 2: ORQUESTRAR A INICIALIZAÇÃO EM CASCATA
    # =============================================================================

    # Gatilho 1: Inicia o controller_manager SOMENTE APÓS o robot_state_publisher iniciar
    delayed_controller_manager_starter = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_robot_state_publisher,
            on_start=[controller_manager],
        )
    )

    # Gatilho 2: Inicia os spawners SOMENTE APÓS o controller_manager iniciar
    delayed_spawners_starter = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                diff_drive_spawner,
                joint_broad_spawner
            ],
        )
    )

    # =============================================================================
    # ETAPA 3: RETORNAR A LISTA DE AÇÕES
    # =============================================================================

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        DeclareLaunchArgument('use_ros2_control', default_value='true', description='Use ros2_control if true'),

        # Nós que podem iniciar imediatamente
        node_robot_state_publisher,
        joystick,
        twist_mux,

        # Gatilhos que gerenciam o resto da inicialização
        delayed_controller_manager_starter,
        delayed_spawners_starter
    ])