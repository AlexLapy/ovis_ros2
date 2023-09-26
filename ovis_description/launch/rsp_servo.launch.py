import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    xacro_file = "ovis_gazebo.urdf.xacro"
    package_description = "ovis_moveit_config"
    use_sim_time = LaunchConfiguration('use_sim_time')

    print("Fetching URDF ==>")
    pkg_path = os.path.join(get_package_share_directory("ovis_moveit_config"))
    xacro_file = os.path.join(pkg_path, 'config', "ovis_gazebo.urdf.xacro")
    kine_file = os.path.join(pkg_path, 'config', "kinematics.yaml")

    moveit_config = (
        MoveItConfigsBuilder("ovis")
        .robot_description(file_path=xacro_file)
        .robot_description_kinematics(file_path=kine_file)
        .to_moveit_configs()
    )
    # Create a robot_state_publisher node
    #params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': True},],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
            
        node_robot_state_publisher
    ])
