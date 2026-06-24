import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_bringup = get_package_share_directory(
        'nav_bringup'
    )

    pkg_astar = get_package_share_directory(
        'nav_astar'
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_bringup,
                'launch',
                'sim_obstacles.launch.py'
            )
        )
    )

    slam = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[{
        'use_sim_time': True
    }]
)

    astar = Node(
        package='nav_astar',
        executable='astar_node',
        name='astar_node',
        output='screen',
        parameters=[
            os.path.join(
                pkg_astar,
                'config',
                'astar_params.yaml'
            ),
            {'use_sim_time': use_sim_time}
        ]
    )

    rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=['-d', os.path.join(
        get_package_share_directory('nav_astar'),
        'config',
        'rviz',
        'nav.rviz'
    )]
)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),
        sim,
        slam,
        astar,
        rviz,
    ])