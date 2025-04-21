from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('agv_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'agv.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'agv_world.world')

    # Verify files exist
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f'URDF file not found: {urdf_file}')
    if not os.path.exists(world_file):
        raise FileNotFoundError(f'World file not found: {world_file}')

    return LaunchDescription([
        LogInfo(msg=f'Loading world: {world_file}'),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),
        LogInfo(msg='Waiting for Gazebo to start...'),
        ExecuteProcess(
            cmd=['sleep', '10'],
            output='screen'
        ),
        LogInfo(msg=f'Spawning AGV with URDF: {urdf_file}'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'agv', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.3'],
            output='screen'
        ),
    ])