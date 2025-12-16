import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    localization_config = os.path.join(
        get_package_share_directory("localization"),
        "config",
        "robot_localization_config.yaml",
    )
    amcl_config = os.path.join(
        get_package_share_directory("localization"),
        "config", "amcl_config.yaml"
    )
    map_desc = os.path.join(
        get_package_share_directory("mapping"), "maps", "apartment_map2.yaml"
    )

    rviz_config = os.path.join(
        get_package_share_directory("localization"), "rviz",
        "localization.rviz"
    )

    gui = LaunchConfiguration("gui", default=True)

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("control_hardware"),
                        "launch",
                        "ignition_sim_basic_telemetry.launch.py",
                    ]
                )
            ]
        )
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="gui",
                default_value=gui,
                description="set to false to prevent opening rviz",
            ),
            control_launch,
            Node(
                package="robot_localization",
                executable="ekf_node",
                output="screen",
                parameters=[localization_config],
            ),
            amcl,
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"use_sim_time": True},
                            {"yaml_filename": map_desc}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_node",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": ["map_server", "amcl"]},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                condition=IfCondition(gui),
                parameters=[{"use_sim_time": True}],
                arguments=["-d", rviz_config],
            ),
        ]
    )
