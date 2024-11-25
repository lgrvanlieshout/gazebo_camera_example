import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    # Setup project paths
    pkg_name = 'camera'
    
    default_robot_name = 'cam'
    gazebo_launch_file_path = 'launch'
    urdf_file_path = 'description/camera.urdf.xacro'
    world_file_path = 'worlds/ground_plane.world'
    
    # Set the path to needed files and folders
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share_gazebo = FindPackageShare(package=pkg_name).find(pkg_name)
    
    default_urdf_model_path = os.path.join('src', pkg_name, urdf_file_path)
    gazebo_launch_file_path = os.path.join('src', pkg_name, gazebo_launch_file_path)
    default_world_path = os.path.join('src', pkg_name, world_file_path)
    
    # Launch configuration variables for simulation
    robot_name = LaunchConfiguration('robot_name')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    
    # Declare launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value=default_robot_name,
        description='The name for the robot')
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Full path to the world model file to load')
    
    # Start Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -v 4 ', world])])
    
    # Process the URDF.xacro file
    robot_urdf = xacro.process_file(default_urdf_model_path)
    
    # Start robot state publisher
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf.toxml(), 'use_sim_time': use_sim_time}]
    )
    
    # Spawn the robot
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_urdf.toxml(),
            '-name', robot_name,
            '-allow_renaming', 'true',
            ],
        output='screen'
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    
    # Add actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    
    return ld
