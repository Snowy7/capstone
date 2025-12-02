#!/usr/bin/env python3
# launch/bringup_all.launch.py
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Paths and packages
    robot_pkg = FindPackageShare('robot')
    motor_pkg = FindPackageShare('motor')
    ydlidar_pkg = FindPackageShare('ydlidar_ros2_driver')
    slam_pkg = FindPackageShare('slam_toolbox')
    nav2_pkg = FindPackageShare('nav2_bringup')

    # Files
    robot_description_launch = PathJoinSubstitution([robot_pkg, 'launch', 'description.launch.py'])
    motor_bringup_launch = PathJoinSubstitution([motor_pkg, 'launch', 'mecanum_bringup.launch.py'])
    ydlidar_launch = PathJoinSubstitution([ydlidar_pkg, 'launch', 'ydlidar.py'])
    #slam_localization_launch = PathJoinSubstitution([slam_pkg, 'launch', 'localization_launch.py'])
    #nav2_navigation_launch = PathJoinSubstitution([nav2_pkg, 'launch', 'navigation_launch.py'])

    # Local config files (adjust if your package/paths differ)
    slam_params = PathJoinSubstitution([robot_pkg, 'config', 'slam_loc.yaml'])
    nav2_params = PathJoinSubstitution([robot_pkg, 'config', 'nav2_params.yaml'])

    # 1) Robot description (URDF, TF static)
    description_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch)
    )

    # 2) Motors + odometry + teleop etc (your bringup)
    motor_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motor_bringup_launch)
    )

    # 3) LiDAR driver
    ydlidar_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch)
    )

    # 4) SLAM Toolbox (localization) – delayed start to let TF + scan warm up
    """ slam_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_localization_launch),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false',
        }.items(),
    )
    slam_delayed = TimerAction(period=2.0, actions=[slam_ld]) """

    # 5) Nav2 bringup – start after SLAM is publishing map->odom
    """ nav2_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch),
        launch_arguments={
            'params_file': nav2_params,
        }.items(),
    ) """
    #nav2_delayed = TimerAction(period=4.0, actions=[nav2_ld])

    return LaunchDescription([
        description_ld,
        motor_ld,
        ydlidar_ld,
        #slam_delayed,
        #nav2_delayed,
    ])