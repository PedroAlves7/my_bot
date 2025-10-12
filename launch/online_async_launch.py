import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo  # <--- ADICIONE LogInfo AQUI
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # =============================================================================
    # ADICIONE ESTA AÇÃO DE LOG AQUI. É A NOSSA PROVA.
    # =============================================================================
    proof_of_life_log = LogInfo(
        msg="==============================================================\n"
            "--- EXECUTANDO A VERSÃO CORRETA E SIMPLIFICADA DO LAUNCH FILE! ---\n"
            "=============================================================="
    )

    # O resto do seu arquivo simplificado...
    default_params_file = os.path.join(
        get_package_share_directory("my_bot"),
        'config',
        'mapper_params_online_async.yaml'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Caminho completo para o arquivo de parâmetros.'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Usar o relógio de simulação'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    ld = LaunchDescription()

    # Adicione a nossa prova de vida como a primeira ação
    ld.add_action(proof_of_life_log)

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld