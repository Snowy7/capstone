from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('robot')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'my_robot.urdf.xacro'])
    
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'),
                    ' ',
                    xacro_file,
                ])
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='robot',  # your package containing encoders_to_joint_state
            executable='encoders_to_joint_state',
            name='encoders_to_joint_state',
            output='screen',
            parameters=[{'ticks_per_rev': 2480.0}],
        ),
    ])