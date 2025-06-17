from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rtabmap_f1tenth')
    config = os.path.join(pkg_share, 'config', 'rtabmap_default.yaml')

    config_la = DeclareLaunchArgument(
        'config', default_value=config,
        description='RTAB-Map parameters')

    return LaunchDescription([
        config_la,
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[LaunchConfiguration('config')]
        ),
        Node(
            package='rtabmap_ros',
            executable='rtabmapviz',
            name='rtabmapviz',
            output='screen',
            parameters=[LaunchConfiguration('config')]
        ),
        Node(
            package='rtabmap_f1tenth',
            executable='imu_lowpass_filter',
            name='imu_filter'
        )
    ])
