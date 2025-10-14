import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'my_bot'

    # --- Etapa 1: Gerar a descrição do robô UMA VEZ ---
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'robot.urdf.xacro'
    )
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path, ' use_ros2_control:=true']),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # --- Etapa 2: Robot State Publisher ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # --- Etapa 3: Twist Mux ---
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # --- Etapa 4: Controller Manager ---
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output='screen',
        emulate_tty=True,
        arguments=['--log-level', 'info'],
    )

    # --- Etapa 5: Spawners ---
    # A lógica de iniciar os spawners depois do controller_manager está correta.
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager-timeout", "30"],
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager-timeout", "30"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # --- Etapa 6: Montar e retornar a Launch Description ---
    return LaunchDescription([
        rsp_node,
        twist_mux,
        controller_manager, # Lançado diretamente, sem o delay que não é mais necessário
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])