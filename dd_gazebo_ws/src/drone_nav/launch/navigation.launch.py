from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
import os
import launch
import launch_ros


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='drone_nav').find('drone_nav')
    default_model_path = os.path.join(pkg_share, 'description/drone_model.urdf')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                    description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str), 'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='drone_nav',
            executable='stabilized_tf_broadcaster',
            name='stabilized_broadcaster'),
        Node(
            package='drone_nav',
            executable='footprint_tf_broadcaster',
            name='footprint_broadcaster'),
        Node(
            package='drone_nav',
            executable='odom_publisher_broadcaster',
            name='odom_publisher_broadcaster'),
        Node(
            package='mode_state_publisher',
            executable='mode_state_publisher',
            parameters=[{
                'drone_description': launch_ros.descriptions.ParameterValue(
                    Command(['xacro ', LaunchConfiguration('model')]), value_type=str
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            name='odom_publisher_broadcaster')
    ])