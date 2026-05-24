import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav_map = get_package_share_directory('nav_map')
    nav2_params = os.path.join(pkg_nav_map, 'params', 'dijkstra.yaml')
    nav2_map = os.path.join(pkg_nav_map, 'maps', 'simple_map.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav_map, 'launch', 'spawn_robot.launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('turtlebot3_navigation2'),
                    'launch',
                    'navigation2.launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': nav2_params,
                'map': nav2_map
            }.items(),
        ),

        Node(
            package='nav_map',
            executable='run_navigation_test.py',
            name='run_navigation_test',
            output='screen',
        ),
    ])
