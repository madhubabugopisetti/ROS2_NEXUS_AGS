
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory("ags_description")
    world_file = os.path.join(pkg_path, "worlds", "world.sdf")
    xacro_file = os.path.join(pkg_path, "urdf", "ags.xacro")
    rviz_file = os.path.join(pkg_path, "rviz", "ags.rviz")

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
                '-topic', 'robot_description'
            ],
            output='screen'
        )]
    )

    #
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": Command(["xacro ", xacro_file])}]
    )

    # --- bridge ---
    bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen"
    )

    # --- RViz ---
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        bridge,
        rsp,
        spawn_robot,
        rviz
    ])
