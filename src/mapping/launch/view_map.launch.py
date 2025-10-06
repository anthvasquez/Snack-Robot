import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
import xacro


def generate_launch_description():
    package_name = 'mapping'
    map_config = os.path.join(get_package_share_directory(
        package_name), 'maps', 'apartment_map2.yaml')
    rviz_config = os.path.join(get_package_share_directory(
        package_name), 'rviz', 'map_server.rviz')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'yaml_filename': map_config}
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
