from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('agv_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'agv.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'agv_world.world')

    # 1) Launch Gazebo with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
    )

    return LaunchDescription([
        LogInfo(msg=f"[agv_sim] Loading world: {world_file}"),
        gazebo,

        # 2) Publish robot_description
        LogInfo(msg="[agv_sim] Starting robot_state_publisher"),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_file, 'r').read()
            }],
        ),

        # 3) Spawn the AGV into Gazebo
        LogInfo(msg="[agv_sim] Spawning AGV into Gazebo"),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'agv',
                '-file', urdf_file,
                '-x', '0', '-y', '0', '-z', '0.3',
                '-timeout', '30'
            ],
            output='screen',
        ),
    ])
