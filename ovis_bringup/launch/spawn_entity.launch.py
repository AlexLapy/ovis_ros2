import os

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ovisBot',
                                   '-x', '0.0',
                                   '-y', '0.0' ],
                        output='screen'
    )

    return LaunchDescription([
        spawn_entity,
    ])
