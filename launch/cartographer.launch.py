# cartographer.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    pkg_share_dir = get_package_share_directory('my_bot') # <-- CHANGE THIS

    # Cartographer SLAM node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=['-configuration_directory', os.path.join(pkg_share_dir, 'config'),
                   '-configuration_basename', 'cartographer_config.lua'],
        parameters=[{'use_sim_time': False},
                    {'tracking_frame': 'chassis'}],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/bno055/imu')
        ]
    )

    # Cartographer Occupancy Grid node
    # This node takes the map data from Cartographer and creates the occupancy grid for Nav2
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        cartographer_node,
        cartographer_occupancy_grid_node
    ])