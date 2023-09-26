import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    xacro_file = "ovis_standalone.xacro"
    package_description = "ovis_description"
    use_sim_time = LaunchConfiguration('use_sim_time')

    print("Fetching URDF ==>")
    pkg_path = os.path.join(get_package_share_directory(package_description))
    xacro_file = os.path.join(pkg_path, 'urdf', xacro_file)
    # Pass the ros2_control arg to the xacro builder
    robot_description_config = Command(['xacro ', xacro_file])
    

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
      #  name='ovis_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_config}],
        output="screen"
    )

    # RVIZ Configuration
    #rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'config', 'view_arm.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': use_sim_time}],
            #arguments=['-d', rviz_config_dir]
            )
    
    jsp_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen')


    # create and return launch description object
    return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use sim time if true'),

            rsp_node,
            rviz_node,
            jsp_node,
        ])