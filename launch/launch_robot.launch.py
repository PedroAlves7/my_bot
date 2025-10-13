import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot'

    # --- Argumentos de Lançamento ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # --- Carregamento do URDF ---
    # O XACRO é processado aqui e o resultado será usado pelo controller_manager
    xacro_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro')
    robot_description_config = Command(
        ['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    # --- Nós do ROS2 Control ---
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    # O Controller Manager é o "MESTRE": recebe o URDF diretamente
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description_config},
            controller_params_file
        ]
    )

    # --- Robot State Publisher ---
    # O Robot State Publisher é "SUBORDINADO": não recebe o URDF, irá subscrever ao tópico
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]  # Note que não passamos o robot_description aqui
    )

    # --- Spawners e outros nós ---
    # Não precisam de atraso, pois o controller_manager iniciará rapidamente agora
    diff_drive_spawner = Node(package="controller_manager", executable="spawner", arguments=["diff_cont"])
    joint_broad_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_broad"])

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

    # --- Lista Final de Ações ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        DeclareLaunchArgument('use_ros2_control', default_value='true', description='Use ros2_control if true'),

        controller_manager,
        node_robot_state_publisher,
        joystick,
        twist_mux,
        diff_drive_spawner,
        joint_broad_spawner
    ])