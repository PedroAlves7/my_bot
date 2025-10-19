# cartographer.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    pkg_share_dir = get_package_share_directory('my_bot') # <-- CHANGE THIS

    # Define the path to the LUA configuration file
    lua_config_path = os.path.join(
        pkg_share_dir,
        'config',
        'cartographer_config.lua'
    )

    # Cartographer SLAM node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=['-configuration_filename', lua_config_path],
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('scan', '/scan') # Remap the 'scan' topic if your LiDAR publishes on a different name
        ]
    )

    # Cartographer Occupancy Grid node
    # This node takes the map data from Cartographer and creates the occupancy grid for Nav2
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node.',
        name='cartographer_occupancy_grid_node.',
        output='screen',
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        cartographer_node,
        cartographer_occupancy_grid_node
    ])