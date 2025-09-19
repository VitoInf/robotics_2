import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    multirobot_dir = get_package_share_directory('multirobot_sim')
    turtlebot3_gz_dir = get_package_share_directory('turtlebot3_gazebo')
    world = os.path.join(multirobot_dir, 'worlds', 'field.world')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Start Gazebo
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[multirobot_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
         cmd=['gzclient'],
         cwd=[multirobot_dir], output='screen')

    # URDF & SDFs
    urdf = os.path.join(turtlebot3_gz_dir, 'urdf', 'turtlebot3_burger.urdf')
    leader_sdf = os.path.join(turtlebot3_gz_dir, 'models', 'turtlebot3_burger', 'model.sdf')
    left_follower_sdf = os.path.join(multirobot_dir, 'models', 'left_follower', 'model.sdf')
    right_follower_sdf = os.path.join(multirobot_dir, 'models', 'right_follower', 'model.sdf')

    # Read robot description
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # Spawn Leader Robot
    spawn_leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(multirobot_dir, 'launch', 'robot_spawner.launch.py')
        ]),
        launch_arguments={
            'robot_name': '',
            'sdf_file': leader_sdf,
            'robot_description': robot_description,
            'x_pose': '9.5',
            'y_pose': '0.0',
            'z_pose': '0.05',
            'yaw': '3.14'
        }.items()
    )

    # Spawn Left Follower Robot
    spawn_left_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(multirobot_dir, 'launch', 'robot_spawner.launch.py')
        ]),
        launch_arguments={
            'robot_name': 'left_follower',
            'sdf_file': left_follower_sdf,
            'robot_description': robot_description,
            'x_pose': '11.5',
            'y_pose': '-2.0',
            'z_pose': '0.05',
            'yaw': '3.14'
        }.items()
    )

    # Spawn Right Follower Robot
    spawn_right_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(multirobot_dir, 'launch', 'robot_spawner.launch.py')
        ]),
        launch_arguments={
            'robot_name': 'right_follower',
            'sdf_file': right_follower_sdf,
            'robot_description': robot_description,
            'x_pose': '11.5',
            'y_pose': '2.0',
            'z_pose': '0.05',
            'yaw': '3.14'
        }.items()
    )

    # Polar-Based Controllers
    leader_controller = Node(
            package='multirobot_sim',
            executable='leader_controller',
            name='leader_controller',
        )

    lft_flwr_controller = Node(
            package='multirobot_sim',
            executable='follower_controller',
            name='left_follower_controller',
            parameters=[{
                'robot_name': 'left_follower',
                'offset_x': -2.0,
                'offset_y': 2.0
            }]
        )
    
    rgt_flwr_controller = Node(
            package='multirobot_sim',
            executable='follower_controller',
            name='right_follower_controller',
            parameters=[{
                'robot_name': 'right_follower',
                'offset_x':-2.0,
                'offset_y': -2.0
            }]
        )

    # start the rviz
    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(multirobot_dir, 'rviz', 'multirobot.rviz')]
        )
    
    # Launch SLAM using SLAM Toolbox 
    slam_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items())



    # world->odom transform, Ground Truth Pose
    lft_flwr_localization_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='left_follower',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'left_follower/odom'])
    
    rgt_flwr_localization_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='right_follower',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'right_follower/odom'])

    ld = LaunchDescription()       
    ld.add_action(declare_use_sim_time_cmd)
     
    # Start Simulation
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(rviz_cmd)

    # Spawn robots
    ld.add_action(spawn_leader)
    ld.add_action(spawn_left_follower)
    ld.add_action(spawn_right_follower)

    # Launch Controllers
    ld.add_action(leader_controller)
    ld.add_action(lft_flwr_controller)
    ld.add_action(rgt_flwr_controller)

    # Launching Static Transforms
    ld.add_action(lft_flwr_localization_cmd) 
    ld.add_action(rgt_flwr_localization_cmd)

    # Launch SLAM 
    ld.add_action(slam_cmd)

    return ld