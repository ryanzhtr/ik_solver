import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    chain_start = LaunchConfiguration('chain_start')
    chain_end = LaunchConfiguration('chain_end')
    timeout = LaunchConfiguration('timeout')
    epsilon = LaunchConfiguration('epsilon')

    pkg_share = FindPackageShare('ik_solver').find('ik_solver')
    urdf_file = os.path.join(pkg_share, 'launch', 'xarm7.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument('chain_start', default_value='link_base'),
            DeclareLaunchArgument('chain_end', default_value='link7'),
            DeclareLaunchArgument('timeout', default_value='0.01'),
            DeclareLaunchArgument('epsilon', default_value='0.00005'),
            Node(
                package='ik_solver',
                executable='ik_solver',
                output='screen',
                parameters=[
                    {
                        'robot_description': robot_desc,
                        'chain_start': chain_start,
                        'chain_end': chain_end,
                        'timeout': timeout,
                        'epsilon': epsilon,
                    }
                ],
            ),
        ]
    )