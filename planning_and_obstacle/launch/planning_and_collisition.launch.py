import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    publisher_node_planner = launch_ros.actions.Node(
        package='planning_and_obstacle',
        executable='optimal_planner_node',
        name='optimal_planner_node',
        output='screen',

    )
    
    return launch.LaunchDescription([
        publisher_node_planner
    ])