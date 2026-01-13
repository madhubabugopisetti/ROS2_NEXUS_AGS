
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory("ags_description")
    world_file = os.path.join(pkg_path, "worlds", "world.sdf")

    # Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    return LaunchDescription([
        gazebo
    ])
