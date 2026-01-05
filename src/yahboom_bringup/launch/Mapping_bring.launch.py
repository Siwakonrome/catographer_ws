from launch import LaunchDescription
#from launch.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():


    
    
    carto = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("yahboom_bringup"),
                "",
                "Catographer.launch.py",
            )),
        )
    
    laser_to_point_node = Node(
        package='laserscan_to_point_pulisher',
        executable='laserscan_to_point_pulisher',
        output="screen"
    )

    sync_rewrite_sensors_node = Node(
        package='sync_rewrite_sensors',
        executable='sync_rewrite_sensors',
        output="screen"
    )


    #rf2o_laser_odometry = IncludeLaunchDescription(PythonLaunchDescriptionSource(
     #   [os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch'),'/rf2o_laser_odometry.launch.py'])
    #)
    
    rviz_display_node = Node(
        package='rviz2',
        executable="rviz2",
        output="screen"
    )

    return LaunchDescription(
        [
            carto,
            #rf2o_laser_odometry,
            laser_to_point_node,
            rviz_display_node,
            sync_rewrite_sensors_node,
        ]
    )
