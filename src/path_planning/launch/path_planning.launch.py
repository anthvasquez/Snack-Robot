from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    config_folder = os.path.join(
        get_package_share_directory('path_planning'), 'config')
    planner_yaml = os.path.join(config_folder, 'planner_server.yaml')
    controller_yaml = os.path.join(config_folder, 'controller_server.yaml')
    bt_navigator_yaml = os.path.join(config_folder, 'bt_navigator.yaml')
    recovery_yaml = os.path.join(config_folder, 'recovery.yaml')
    local_costmap_yaml = os.path.join(config_folder, 'local_costmap.yaml')
    global_costmap_yaml = os.path.join(config_folder, 'global_costmap.yaml')
    behaviors_xml = os.path.join(
        get_package_share_directory('path_planning'), 'config', 'behavior.xml')

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                planner_yaml,
                global_costmap_yaml
            ]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                controller_yaml,
                local_costmap_yaml
            ]
        ),
        Node(
            package='twist_stamper',
            executable='twist_stamper',
            output='screen',
            parameters=[{'frame_id': 'base_link', 'use_sim_time': True}],
            remappings=[
                ('cmd_vel_in', 'cmd_vel'),
                ('cmd_vel_out', '/diff_drive_controller/cmd_vel')
            ]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml,
                        {'default_nav_to_pose_bt_xml': behaviors_xml}]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recovery_server',
            output='screen',
            parameters=[recovery_yaml]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='nav2_lifecycle_manager',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': [
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'recovery_server']}
            ]
        )
    ])
