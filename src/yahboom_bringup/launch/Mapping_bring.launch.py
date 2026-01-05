from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for Cartographer SLAM mapping with real sensors.

    Args:
        use_sim_time (bool): Use simulation/rosbag clock if true, system clock if false.
                            For real sensors: set to false (default)
                            For rosbag playback: set to true and use --clock flag
    """

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo/rosbag) clock if true, system clock if false'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include Cartographer launch file
    carto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yahboom_bringup"),
                "Catographer.launch.py"
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # LaserScan to PointCloud converter node
    laser_to_point_node = Node(
        package='laserscan_to_point_pulisher',
        executable='laserscan_to_point_pulisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Sensor synchronization and rewrite node
    sync_rewrite_sensors_node = Node(
        package='sync_rewrite_sensors',
        executable='sync_rewrite_sensors',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz2 visualization
    rviz_config_file = os.path.join(
        get_package_share_directory('yahboom_bringup'),
        'rvizs',
        'mapping.rviz'
    )

    rviz_display_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            carto,
            laser_to_point_node,
            rviz_display_node,
            sync_rewrite_sensors_node,
        ]
    )
