from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')
    raceline_csv = os.path.join(pkg_share, 'config', 'raceline.csv')

    pure_pursuit_node = Node(
        package='f1tenth_stack',
        executable='pure_pursuit',
        name='pure_pursuit',
        parameters=[{
            'raceline_csv': raceline_csv,
            'lookahead_distance': 1.0,
            'speed': 2.0
        }]
    )

    tf_node = Node(
        package='f1tenth_stack',
        executable='tf_publisher',
        name='tf_publisher'
    )

    return LaunchDescription([
        pure_pursuit_node,
        tf_node,
    ])
