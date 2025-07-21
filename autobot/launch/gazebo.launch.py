from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    your_package = get_package_share_directory('your_robot_package')
    urdf_path = os.path.join(your_package, 'urdf', 'my_robot.urdf')

    with open(urdf_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])

