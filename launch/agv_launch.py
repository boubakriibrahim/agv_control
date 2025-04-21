from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

def generate_launch_description():
    pkg_share = get_package_share_directory('agv_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'agv.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'agv_world.world')

    # Verify files exist
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f'URDF file not found: {urdf_file}')
    if not os.path.exists(world_file):
        raise FileNotFoundError(f'World file not found: {world_file}')

    # Validate URDF
    try:
        subprocess.run(['check_urdf', urdf_file], check=True, capture_output=True, text=True)
        print('URDF validation passed')
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f'URDF validation failed: {e.stderr}')

    # Launch Gazebo with ROS plugins
    gazebo_cmd = [
        'gazebo', '--verbose',
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
        world_file
    ]

    # Node: publish robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read()
        }]
    )

    # Node: spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'agv', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.3', '-timeout', '30'],
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg=f'Loading world: {world_file}'),
        ExecuteProcess(cmd=gazebo_cmd, output='screen'),
        LogInfo(msg='Waiting for Gazebo to initialize plugins...'),
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='Publishing robot description'),
                robot_state_publisher,
                LogInfo(msg='Spawning AGV in Gazebo'),
                spawn_entity
            ]
        )
    ])
