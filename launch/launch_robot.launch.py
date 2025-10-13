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

    # --- 1. Lógica do Robot State Publisher ---

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Processa o arquivo URDF usando xacro a partir do diretório correto
    pkg_path = get_package_share_directory(package_name)

    # ==============================================================================
    # == CORREÇÃO AQUI: 'description' em vez de 'urdf' ==
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    # ==============================================================================

    robot_description_config = Command(
        ['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': use_sim_time}]
    )

    # --- 2. Outros Nós (Joystick, Twist Mux) ---
    # ... (o resto do seu código para joystick e twist_mux continua igual)
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

    # --- 3. Lógica do ROS2 Control (com o gatilho corrigido) ---
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file]
    )

    diff_drive_spawner = Node(package="controller_manager", executable="spawner", arguments=["diff_cont"])
    joint_broad_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_broad"])

    delayed_controller_manager_starter = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_robot_state_publisher,
            on_start=[controller_manager],
        )
    )
    delayed_spawners_starter = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                diff_drive_spawner,
                joint_broad_spawner
            ],
        )
    )

    # --- 4. Retorna a Descrição do Lançamento ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        DeclareLaunchArgument('use_ros2_control', default_value='true', description='Use ros2_control if true'),

        node_robot_state_publisher,
        joystick,
        twist_mux,

        delayed_controller_manager_starter,
        delayed_spawners_starter
    ])