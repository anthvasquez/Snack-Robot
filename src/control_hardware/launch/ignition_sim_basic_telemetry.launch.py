from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import xacro
import os

def generate_launch_description():
    xacro_filepath = os.path.join(get_package_share_directory('control_hardware'), 'urdf', 'ignition_ros2_control.xacro')
    robot_description = xacro.process_file(xacro_filepath).toxml()

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Launch Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]
    )

    # Spawn the robot defined on 'robot_description'
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'snack_robot',
                   '-z', '0.1',
                   '-allow_renaming', 'true'],
    )
    
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

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
            ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen'
        ),
        gazebo_node,
        ignition_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=[joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_controller]
            )
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        )
    ])