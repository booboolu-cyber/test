#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image',
        description='Topic for camera images'
    )
    
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='150',
        description='Goal X coordinate'
    )
    
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='150',
        description='Goal Y coordinate'
    )
    
    # Create the path planning node
    pathplanning_node = Node(
        package='rosbot_pathplanning',
        executable='pathplanning',
        name='pathplanning',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        image_topic_arg,
        goal_x_arg,
        goal_y_arg,
        pathplanning_node,
    ])