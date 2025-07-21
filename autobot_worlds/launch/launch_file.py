import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments ---
    # Get the TurtleBot3 model type
    robot_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # Path to your custom world file (typically .sdf for Ignition Gazebo)
    pkg_my_robot_worlds = get_package_share_directory('autobot_worlds')
    world_file_name = 'custom_env.sdf' # Use .sdf for Ignition Gazebo
    world_path = os.path.join(pkg_my_robot_worlds, 'worlds', world_file_name)

    # Launch arguments for Gazebo (via ros_gz_sim)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    # --- Robot Description (URDF/XACRO) ---
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    xacro_file_path = os.path.join(
        pkg_turtlebot3_description,
        'urdf',
        f'turtlebot3_{robot_model}.urdf.xacro'
    )
    robot_description_content = Command(['xacro ', xacro_file_path])

    # --- Launch Ignition Gazebo (Harmonic) via ros_gz_sim ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # Pass the world file and other args using gz_args
        # -s: server only (no gui), -g: with gui (default), -r: real time factor
        launch_arguments={'gz_args': [
            '-r ', # Start in real time
            (LaunchConfiguration('gui') == 'true').to_value() + ' -g ' if 'true' else '', # Start Gazebo GUI if gui is true
            world_path
        ]}.items()
    )

    # --- Publish Robot Description to Parameter Server ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description_content}],
    )

    # --- Spawn the TurtleBot3 Robot in Gazebo ---
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'turtlebot3_' + robot_model, # Model name in Gazebo
                   '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-Y', '0.0', # Yaw
                   '-allow_renaming', 'true'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Bridge Topics between Gazebo and ROS 2 ---
    # This is ESSENTIAL for Ignition Gazebo (unlike Classic Gazebo where plugins often direct-publish)
    # The topic names on the Gazebo side often follow /model/<model_name>/<sensor_name>
    # Make sure these match the names defined in your TurtleBot3 URDF's <gazebo> extensions.
    ros_gz_bridge_nodes = [
        # Command Velocity (ROS 2 /cmd_vel -> Gazebo /model/turtlebot3_burger/cmd_vel)
        Node(
            package='ros_gz_bridge',
            executable='ros_gz_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            remappings=[
                ('/cmd_vel', '/model/turtlebot3_' + robot_model + '/cmd_vel')
            ]
        ),
        # LIDAR Scan (Gazebo /scan -> ROS 2 /scan)
        Node(
            package='ros_gz_bridge',
            executable='ros_gz_bridge',
            name='lidar_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            ],
            remappings=[
                ('/scan', '/model/turtlebot3_' + robot_model + '/scan')
            ]
        ),
        # Odometry (Gazebo /odom -> ROS 2 /odom)
        Node(
            package='ros_gz_bridge',
            executable='ros_gz_bridge',
            name='odom_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ],
            remappings=[
                ('/odom', '/model/turtlebot3_' + robot_model + '/odom')
            ]
        ),
        # IMU (Gazebo /imu -> ROS 2 /imu)
        Node(
            package='ros_gz_bridge',
            executable='ros_gz_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            ],
            remappings=[
                ('/imu', '/model/turtlebot3_' + robot_model + '/imu')
            ]
        ),
        # You might need to add more bridges depending on your robot's sensors (e.g., camera)
    ]

    # --- Return LaunchDescription ---
    return LaunchDescription([
        # Declare arguments for user override
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run headless.'),

        # Core nodes
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot_node,
    ] + ros_gz_bridge_nodes) # Add all bridge nodes to the launch description