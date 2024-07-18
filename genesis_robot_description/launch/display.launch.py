
#!/usr/bin/env python3

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch_ros


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='genesis_robot_description').find('genesis_robot_description')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/sdv_config.rviz')

    fws_robot_sim_path = os.path.join(
        get_package_share_directory('fws_robot_sim'))


    urdf_file_name = 'genesis_robot.urdf.xacro'

    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('genesis_robot_description'),
        'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
                        'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
                    }],
        arguments=[urdf])
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='{message}'),

        joint_state_publisher_node,
        robot_state_publisher_node, 
        rviz_node
    ])