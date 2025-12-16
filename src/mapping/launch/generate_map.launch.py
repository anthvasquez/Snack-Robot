from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    cartographer_dir = os.path.join(
        get_package_share_directory('mapping'), 'config')
    cartographer_basename = 'cartographer.lua'
    rviz_config = os.path.join(
        get_package_share_directory('mapping'), 'rviz', 'generate_map.rviz')

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('control_hardware'),
                'launch',
                'ignition_sim_basic_telemetry.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        control_launch,
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', cartographer_dir,
                '-configuration_basename', cartographer_basename
            ],
            remappings=[
                ('/odom', '/diff_drive_controller/odom')
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
