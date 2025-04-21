from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

def generate_launch_description():
    # locate files
    pkg_share = get_package_share_directory('agv_control')
    urdf_file  = os.path.join(pkg_share, 'urdf',   'agv.urdf')
    world_file = os.path.join(pkg_share, 'worlds','agv_world.world')
    gazebo_share = get_package_share_directory('gazebo_ros')

    # sanity checks
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f'URDF not found: {urdf_file}')
    if not os.path.exists(world_file):
        raise FileNotFoundError(f'World not found: {world_file}')

    # Validate URDF on disk
    try:
        subprocess.run(
            ['check_urdf', urdf_file],
            check=True, capture_output=True, text=True
        )
        print('URDF validation passed')
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f'URDF validation failed:\n{e.stderr}')

    # 1) Launch Gazebo with ROS plugins (init + factory)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 2) robot_state_publisher for your URDF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'publish_robot_description': True,
            # enable sim time so your interface and tf use /clock
            'use_sim_time': True,
        }]
    )

    # 3) spawn_entity to inject your AGV model into Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_agv',
        output='screen',
        arguments=[
            '-entity', 'agv',
            '-file',    urdf_file,
            '-x',       '0', '-y', '0', '-z', '0.3',
            '-timeout', '30'
        ]
    )

    return LaunchDescription([
        LogInfo(msg='⏳ Starting Gazebo…'),
        gazebo,
        # give Gazebo 5s to load init+factory and start up
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='🚀 Publishing robot_state_publisher and spawning AGV'),
                rsp,
                spawn,
            ],
        ),
    ])
