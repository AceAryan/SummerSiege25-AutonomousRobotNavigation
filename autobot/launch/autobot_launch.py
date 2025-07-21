from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('autobot'),
        'urdf',
        'my_robot.urdf'
    )

    return LaunchDescription([
        # Start Gazebo Harmonic (gz sim)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Spawn robot into simulation
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-name', 'my_robot', '-file', urdf_file],
            output='screen'
        )
    ])
