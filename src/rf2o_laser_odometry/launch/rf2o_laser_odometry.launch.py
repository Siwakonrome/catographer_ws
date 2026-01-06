from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        # Declare use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic' : '/scan_sync',       # TOPIC (Scan)
                'odom_topic' : '/odom_rf2o',             # TOPIC (Odom)
                'publish_tf' : False,
                'base_frame_id' : 'base_footprint_sync', # FRAME (Base footprint)
                'odom_frame_id' : 'odom_sync',           # FRAME (Odom)
                'init_pose_from_topic' : '',
                'freq' : 10.0,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
