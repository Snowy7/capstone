# launch/mecanum_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # get package share directory
    motor_share_dir = get_package_share_directory('motor')
    
    return LaunchDescription([
        Node(package='motor', executable='mecanum_controller', name='mecanum_controller'),
        Node(
            package='motor',
            executable='mecanum_odometry',
            name='mecanum_odometry',
            parameters=[motor_share_dir + '/config/mecanum_odometry.yaml']
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.2,
                'autorepeat_rate': 20.0
            }]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[
                motor_share_dir + '/config/ps5.yaml'
            ],                
            remappings=[
                ('/cmd_vel', 'cmd_vel')  # optional remap
            ]
        )
    ])