from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

def generate_launch_description():
    # Get package share directory
    pkg = get_package_share_directory('agv_control')
    world = os.path.join(pkg, 'worlds', 'agv_world.world')
    urdf = os.path.join(pkg, 'urdf', 'agv.urdf')

    # Verify files exist
    if not os.path.exists(urdf):
        raise FileNotFoundError(f'URDF file not found: {urdf}')
    if not os.path.exists(world):
        raise FileNotFoundError(f'World file not found: {world}')

    # Validate URDF
    try:
        result = subprocess.run(['check_urdf', urdf], check=True, capture_output=True, text=True)
        LogInfo(msg=f'URDF validation passed: {result.stdout}')
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f'URDF validation failed: {e.stderr}')

    # Start Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': 'true'}.items()
    )

    # Publish robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='agv',
        output='screen',
        parameters=[{
            'robot_description': open(urdf).read(),
            'use_sim_time': True
        }],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    # Spawn entity
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='agv',
        output='screen',
        arguments=['-entity', 'agv', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.3', '-timeout', '30', '--ros-args', '--log-level', 'DEBUG']
    )

    # AGV interface node
    interface = Node(
        package='agv_control',
        executable='agv_interface',
        namespace='agv',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': '/agv/odom'
        }],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    return LaunchDescription([
        LogInfo(msg='Starting Gazebo with /agv namespace…'),
        gazebo_launch,
        LogInfo(msg='Validating URDF…'),
        LogInfo(msg='Loading differential drive plugin: libgazebo_ros_diff_drive.so'),
        LogInfo(msg='Launching nodes with /agv namespace after 5s delay…'),
        TimerAction(
            period=5.0,
            actions=[rsp, spawn, interface]
        )
    ])