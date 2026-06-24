import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # pacote do turtlebot3
    tb3_pkg = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    # pacote do seu projeto
    pkg_nav_bringup = get_package_share_directory(
        'nav_bringup'
    )

    # RViz
    rviz_config = os.path.join(
        pkg_nav_bringup,
        'config',
        'rviz',
        'nav.rviz'
    )

    # launch do turtlebot3
    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tb3_pkg,
                'launch',
                'sim_empty.launch.py'
            )
        )
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
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

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz,
        slam,
    ])