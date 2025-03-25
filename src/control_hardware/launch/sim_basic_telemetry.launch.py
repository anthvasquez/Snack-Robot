from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import xacro
import os

def generate_launch_description():
    xacro_filepath = os.path.join(get_package_share_directory('control_hardware'), 'urdf', 'ros2_control.xacro')
    robot_description = xacro.process_file(xacro_filepath).toxml()

    # Launch Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Spawn the robot defined on 'robot_description'
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'snack_robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # start_gazebo_ros_spawner_cmd = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-name', "snack_robot",
    #         '-topic', "robot_description",
    #         '-x', '0.0',
    #         '-y', '0.0',
    #         '-z', '0.01'
    #     ],
    #     output='screen'
    # )
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster']
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diff_drive_controller', "--ros-args", "-r", "/diff_drive_controller/cmd_vel:=/cmd_vel"]
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster]
        )
    )

    delayed_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_controller]
            )
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
            ),
        gazebo_node,
        spawn_entity,
        delayed_joint_state_broadcaster,
        delayed_diff_drive_controller
    ])