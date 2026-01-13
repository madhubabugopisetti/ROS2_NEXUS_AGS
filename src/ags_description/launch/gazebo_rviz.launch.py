
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory("ags_description")
    world_file = os.path.join(pkg_path, "worlds", "world.sdf")
    xacro_file = os.path.join(pkg_path, "urdf", "ags.xacro")

    # Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    # Spawn robot
    spawn_robot = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'nexus_ags',
                '-file', xacro_file
            ],
            output='screen'
        )]
    )
    return LaunchDescription([
        gazebo,
        spawn_robot
    ])
