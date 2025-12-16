from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    models_pkg = 'my_3d_models'
    pkg_gazebo_ros = get_package_share_directory('ros_ign_gazebo')

    world = os.path.join(get_package_share_directory(models_pkg),
                         'worlds', 'apartment.sdf')

    return LaunchDescription([
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(pkg_gazebo_ros, 'launch', 'ign_gazebo.launch.py')
             ),
             launch_arguments={'gz_args': world}.items(),
         ),
     ])
