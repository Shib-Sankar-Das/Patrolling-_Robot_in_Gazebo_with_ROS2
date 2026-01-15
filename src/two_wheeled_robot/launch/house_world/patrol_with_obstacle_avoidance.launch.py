# Author: Patrol Robot Team
# Description: Launch patrol robot with obstacle avoidance safety node
#              This launch file starts the obstacle avoidance monitoring node
#              alongside the main patrol robot

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'two_wheeled_robot'
    
    # Get package share directory
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    avoidance_mode = LaunchConfiguration('avoidance_mode')
    safe_distance = LaunchConfiguration('safe_distance')
    warning_distance = LaunchConfiguration('warning_distance')
    use_obstacle_avoidance = LaunchConfiguration('use_obstacle_avoidance')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_avoidance_mode_cmd = DeclareLaunchArgument(
        name='avoidance_mode',
        default_value='monitor',
        description='Obstacle avoidance mode: monitor, active, or autonomous')
    
    declare_safe_distance_cmd = DeclareLaunchArgument(
        name='safe_distance',
        default_value='0.25',
        description='Safe distance for obstacle avoidance (meters) - reduced for indoor')
    
    declare_warning_distance_cmd = DeclareLaunchArgument(
        name='warning_distance',
        default_value='0.4',
        description='Warning distance for obstacle detection (meters) - reduced for indoor')
    
    declare_use_obstacle_avoidance_cmd = DeclareLaunchArgument(
        name='use_obstacle_avoidance',
        default_value='true',
        description='Whether to run the obstacle avoidance safety node')
    
    # Obstacle Avoidance Node - Safety layer that monitors LIDAR
    obstacle_avoidance_node = Node(
        condition=IfCondition(use_obstacle_avoidance),
        package=package_name,
        executable='obstacle_avoidance.py',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'avoidance_mode': avoidance_mode,
            'safe_distance': safe_distance,
            'warning_distance': warning_distance,
            'scan_topic': '/scan',
            'cmd_vel_topic': '/cmd_vel',
            'enabled': True,
        }]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_avoidance_mode_cmd)
    ld.add_action(declare_safe_distance_cmd)
    ld.add_action(declare_warning_distance_cmd)
    ld.add_action(declare_use_obstacle_avoidance_cmd)
    
    # Add nodes
    ld.add_action(obstacle_avoidance_node)
    
    return ld
