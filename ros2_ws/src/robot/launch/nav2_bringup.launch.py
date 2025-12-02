from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the share directory for nav2_bringup
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Path to your custom Nav2 parameter file (if you have one)
    # If not, the default nav2_bringup params will be used, which is fine
    # as we are setting launch arguments to override AMCL/map server settings.
    # default_params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
    
    # Path to the default slam toolbox config file
    slam_config_file = '/home/engneedo/ros2_ws/src/robot/config/nav2_params.yaml'

    return LaunchDescription([
        # 2. Launch the rest of the Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={'use_sim_time': 'False', # Must match SLAM setting
                              'use_composition': 'True',
                              'params_file': slam_config_file
                              }.items(),
            # Note: We do *not* set 'slam:=False' here, as the default 
            # navigation_launch.py expects 'slam' to be True to use slam_toolbox
            # if we are doing simultaneous localization and mapping.
        ),
    ])
