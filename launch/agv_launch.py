from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('agv_control')
    world = os.path.join(pkg, 'worlds', 'agv_world.world')
    urdf  = os.path.join(pkg, 'urdf',   'agv.urdf')

    # 1) start Gazebo (loads ros_init & ros_factory plugins)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # 2) publish robot_state_publisher & spawn_entity
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf).read(),
            'use_sim_time': True
        }]
    )
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity','agv','-file',urdf,'-x','0','-y','0','-z','0.3','-timeout','30'],
        output='screen'
    )

    # 3) **YOUR** AGV interface node
    interface = Node(
        package='agv_control',
        executable='agv_interface',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        LogInfo(msg='Starting Gazebo…'),
        gazebo_launch,
        TimerAction(
            period=5.0,  # give Gazebo time to come up
            actions=[rsp, spawn, interface]
        )
    ])
