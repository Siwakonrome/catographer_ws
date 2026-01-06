from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource








def generate_launch_description():
    return LaunchDescription([
        

        DeclareLaunchArgument(
            name='use_rtabmap',
            default_value='true',
            choices=['true', 'false'],
            description='Use RTAB-Map for SLAM and localization'
        ),

        DeclareLaunchArgument(
            name='localize_only',
            default_value='false',
            choices=['true', 'false'],
            description='Localize only, do not change loaded map'
        ),

        DeclareLaunchArgument(
            name='use_nav2',
            default_value='true',
            choices=['true', 'false'],
            description='Launch Nav2 navigation stack'
        ),

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Launch RViz2 for visualization'
        ),

        

        # Launch RTAB-Map for SLAM/Localization ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('yahboom_bringup'),
                    'launch',
                    'rtabmap_2d_lidar_slam.launch.py'
                ])
            ),
            launch_arguments={
                'localize_only': LaunchConfiguration('localize_only'),
                'use_sim_time': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_rtabmap'))
        ),

        # Launch Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': PathJoinSubstitution([
                    FindPackageShare('yahboom_bringup'),
                    'params',
                    'nav2_params.yaml'
                ])
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_nav2'))
        ),

        # Launch RViz with Nav2 config
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'rviz_launch.py'
                ])
            ),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            launch_arguments={
                'use_sim_time': 'false',
            }.items(),
        ),
    ])