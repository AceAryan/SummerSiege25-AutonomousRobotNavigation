from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Path to URDF
    urdf_path = os.path.join(
    get_package_share_directory('autobot'),  
    'urdf',
    'my_robot.urdf'
     )
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()


    # Process URDF
    robot_description = xacro.process_file(urdf_path).toxml()

    return LaunchDescription([
        Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_description_content}],
            ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
