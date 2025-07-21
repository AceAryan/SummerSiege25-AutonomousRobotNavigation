import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    # Paths
    pkg_autobot_worlds = get_package_share_directory('autobot_worlds')
    world_path = os.path.join(pkg_autobot_worlds, 'worlds', 'custom_env.sdf')
    urdf_file = os.path.join(pkg_autobot_worlds, 'urdf', 'turtlebot3_waffle_custom.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # Launch Gazebo with custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -g {world_path}'}.items()
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description_content}],
    )

    # Spawn TurtleBot3 Waffle in the corner
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'turtlebot3_waffle',
                   '-x', '-4.0', '-y', '-6.0', '-z', '0.01',
                   '-Y', '0.0',
                   '-allow_renaming', 'true'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('gui', default_value='true', description='Set to "false" to run headless.'),
        gazebo_launch,
        robot_state_publisher_node,
        spawn_robot_node
    ])
