import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Argumentos de Lançamento ---

    # 1. Argumento para o arquivo de parâmetros
    # Ele aponta para o seu arquivo por padrão, tornando o comando mais curto
    default_params_file = os.path.join(
        get_package_share_directory("my_bot"),
        'config',
        'mapper_params_online_async.yaml'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Caminho completo para o arquivo de parâmetros a ser usado.'
    )

    # 2. Argumento para o tempo de simulação
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',  # O padrão deve ser False para um robô real
        description='Usar o relógio de simulação (Gazebo)'
    )

    # Obter os valores dos argumentos
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Nó do SLAM Toolbox ---

    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,  # Passa o arquivo de parâmetros diretamente
            {'use_sim_time': use_sim_time}
        ]
    )

    # --- Descrição do Lançamento ---

    ld = LaunchDescription()

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld