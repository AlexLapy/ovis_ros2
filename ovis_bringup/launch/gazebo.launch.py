import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch import LaunchContext



def generate_launch_description():

    package_bringup='ovis_bringup'
    package_control='ovis_control'
    package_description='ovis_description'

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_description),'launch','rsp_servo.launch.py')]),
                launch_arguments={'use_sim_time': "true"}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_bringup),'config','gazebo_params.yaml')
    gazebo_world_file = os.path.join(get_package_share_directory(package_bringup),'worlds','empty.world')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                                      'world': gazebo_world_file}.items()
    )

    servo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ovis_servo'), 'launch', 'servo_gazebo.launch.py'
                    )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ovisBot',
                                   '-x', '0.0',
                                   '-y', '0.0' ],
                        output='screen'
    )

    # Controllers
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                    "--controller-manager", "/controller_manager"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_broad_spawner],
        )
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ovis_manipulator_controller",
                    "--controller-manager", "/controller_manager"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
                target_action=joint_broad_spawner,
                on_exit=[diff_drive_spawner],
        )
    )

    delayed_servo = TimerAction(period=15.0, actions=[servo])
    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity])

    return LaunchDescription([
        rsp,
        gazebo,
        #delayed_servo,
        spawn_entity,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
    ])
