import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'my_bot' #<--- CONFIRME SE O NOME DO PACOTE ESTÁ CORRETO

    # --- Etapa 1: Gerar a descrição do robô UMA VEZ, a partir do arquivo XACRO ---
    # Esta é a nossa "fonte única da verdade".
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'robot.urdf.xacro' #<--- MUDE PARA O NOME DO SEU ARQUIVO XACRO
    )
    # Usamos ParameterValue para processar o xacro e passar os argumentos necessários
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path, ' use_ros2_control:=true']),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # --- Etapa 2: Criar o Robot State Publisher ---
    # Agora passamos a descrição diretamente para ele, em vez de usar IncludeLaunchDescription.
    # Isso simplifica tudo.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # --- Etapa 3: Configurar o Twist Mux ---
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

    # --- Etapa 4: Criar o Controller Manager ---
    # Ele recebe EXATAMENTE a mesma descrição do robô que o RSP. Sem conflitos!
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        # The 'output' and 'emulate_tty' are good practice for viewing controller output
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    # Usar TimerAction para garantir que o RSP tenha tempo de iniciar antes do CM (boa prática)
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # --- Etapa 5: Criar os Spawners (Lógica de delay mantida, pois é correta) ---
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
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
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
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
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])