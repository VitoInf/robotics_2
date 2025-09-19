import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        description='Name of the robot to spawn')
    
    declare_sdf_file_cmd = DeclareLaunchArgument(
        'sdf_file',
        description='Path to the SDF file')
    
    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        description='Robot description content')
    
    declare_x_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position to spawn robot')
    
    declare_y_cmd = DeclareLaunchArgument(
        'y_pose', 
        default_value='0.0',
        description='Y position to spawn robot')
    
    declare_z_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.05', 
        description='Z position to spawn robot')
    
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Yaw orientation to spawn robot')

    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    sdf_file = LaunchConfiguration('sdf_file')
    robot_description = LaunchConfiguration('robot_description')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose') 
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
            'frame_prefix': [robot_name, '/']
        }]
    )
    
    # Gazebo Spawner
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', sdf_file,
            '-robot_namespace', robot_name,
            '-x', x_pose,
            '-y', y_pose, 
            '-z', z_pose,
            '-R', '0.0',
            '-P', '0.0',
            '-Y', yaw
        ]
    )

    ld = LaunchDescription()
    
    # Arguments
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_sdf_file_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)
    
    # Launch Nodes
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
    
    return ld