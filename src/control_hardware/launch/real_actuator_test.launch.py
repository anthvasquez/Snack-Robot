from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
import xacro
import os


def generate_launch_description():
    gui_declaration = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start robot test with rviz'
    )
    gui = LaunchConfiguration('gui')

    rviz_file = os.path.join(get_package_share_directory('control_hardware'),
                             'rviz', 'snack_robot_viz.rviz')
    xacro_file = os.path.join(get_package_share_directory('control_hardware'),
                              'urdf', 'real_snack_robot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare('control_hardware'),
            'config',
            'controller_manager.yaml'
        ]
    )

    return LaunchDescription([
        gui_declaration,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': False}]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[controller_yaml],
            remappings=[
                ("~/robot_description", "/robot_description")
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                'joint_state_broadcaster',
                'diff_drive_controller'
                ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            condition=IfCondition(gui)
        )
    ])
