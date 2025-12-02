from asyncio import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Launch arguments
    workspace_arg = DeclareLaunchArgument(
        'workspace_path',
        default_value='~/ros2_ws',
        description='Path to ROS 2 workspace'
    )
    
    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value='./src/robot/config/slam_toolbox.yaml',
        description='Path to SLAM config (relative to workspace)'
    )
    
    amcl_config_arg = DeclareLaunchArgument(
        'amcl_config',
        default_value='./src/robot/config/amcl.yaml',
        description='Path to AMCL config (relative to workspace)'
    )
    
    nav2_config_arg = DeclareLaunchArgument(
        'nav2_config',
        default_value='./src/robot/config/nav2_params.yaml',
        description='Path to Nav2 config (relative to workspace)'
    )
    
    default_map_arg = DeclareLaunchArgument(
        'default_map',
        default_value='./maps/my_map.yaml',
        description='Default map file (relative to workspace)'
    )
    
    # Mode manager node
    docking_controller_node = Node(
        package='ev_docking',
        executable='docking_controller',
        name='docking_controller',
        output='screen',
        emulate_tty=True
    )

    # system_monitor
    system_monitor_node = Node(
        package='ev_docking',
        executable='system_monitor',
        name='system_monitor',
        output='screen',
        emulate_tty=True
    )
    
    # State Manager
    state_manager_node = Node(
        package='robot',
        executable='state_manager',
        name='state_manager',
        output='screen',
        emulate_tty=True,
    )
    
    # launch ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1 port:=9090
    ros_bridge_node = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch',
            'rosbridge_websocket_launch.xml'
        ),
        launch_arguments={
            'address': '0.0.0.0',
            'port': '9090'
        }.items()
    )
    
    # excute command to run web server "npm run start" in the directory ~/web-ros-controller
    # as a background process but make sure it is killed when the launch file is closed
    web_server_process = ExecuteProcess(
        cmd=['sudo', 'npm', 'run', 'start'],
        cwd=os.path.expanduser('~/web-ros-controller'),
        output='screen',
        shell=True,
        name='web_server_dev_process'
    )
    
    return LaunchDescription([
        workspace_arg,
        slam_config_arg,
        amcl_config_arg,
        nav2_config_arg,
        default_map_arg,
        docking_controller_node,
        ros_bridge_node,
        state_manager_node,
        system_monitor_node,
        web_server_process
    ])