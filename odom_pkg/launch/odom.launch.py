import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    odom_node = launch_ros.actions.Node(
        package='odom_pkg',
        executable='odom.py',
        name='odom_pkg',
        output='screen',
        parameters=[
            # {'use_sim_time': True},
            {'wheel_base': 2.0},
            {'wheel_radius': 0.3},
            {'track_width': 1.2}
        ]
    )
    
    return launch.LaunchDescription([
        odom_node
    ])
