import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource






def generate_launch_description():

    #  ros2 param get /rtabmap use_sim_time
    # Boolean value is: False
    # fibo@fibo-ASUS-TUF-Gaming-A15-FA507UV-FA507UV:~$ ros2 param get /sync_rewrite_node use_sim_time
    # Boolean value is: False
    # fibo@fibo-ASUS-TUF-Gaming-A15-FA507UV-FA507UV:~$ 



    # Conditionally set Mem/IncrementalMemory based on localize_only
    # When localize_only=true, set to 'false', otherwise 'true'
    localize_only = LaunchConfiguration('localize_only')

    parameters_mapping = [{
                'frame_id': 'base_footprint_sync',  # Match odometry child_frame_id
                'odom_frame_id': 'odom_sync',
                'map_frame_id': 'map',
                'publish_tf': True,

                # ===== Sensors =====
                'subscribe_scan': True,
                'scan_topic': '/scan_sync', # ⭐ ADD Scan topic
                'subscribe_rgb': False,
                'subscribe_depth': False,


                # ===== Database path (ROS parameter) =====
                'database_path': '/home/fibo/catographer_ws/maps/rtabmap.db',
        

                # ===== Mapping mode =====
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Mem/STMSize': '10',

                'use_action_for_goal': True,
                

                # ===== Sync / Time =====
                # Use approx_sync=True when /odom is derived from /scan (rf2o)
                'approx_sync': True,
                'queue_size': 30,          # Increased buffer for better sync
                'sync_queue_size': 30,     # Sync queue size

                'wait_for_transform': 0.2,
                'tf_delay': 0.05,
                'tf_tolerance': 0.1,
                'use_sim_time': LaunchConfiguration('use_sim_time'),

                # RTAB-Map parameters for 2D laser scan - reduced update frequency
                'Reg/Force3DoF': 'true',
                'Optimizer/GravitySigma': '0',
    }]

    parameters_localization = [{
                'frame_id': 'base_footprint_sync',  # Match odometry child_frame_id
                'odom_frame_id': 'odom_sync',
                'map_frame_id': 'map',
                'publish_tf': True,

                # ===== Sensors =====
                'subscribe_scan': True,
                'scan_topic': '/scan_sync', # ⭐ ADD Scan topic
                'subscribe_rgb': False,
                'subscribe_depth': False,


                # ===== Use existing DB =====
                'database_path': '/home/fibo/catographer_ws/maps/rtabmap.db',

                # ===== Localization-only =====
                'Mem/IncrementalMemory': 'false',     # ❌ ไม่สร้าง node ใหม่
                'Mem/InitWMWithAllNodes': 'true',     # ✅ โหลด graph ทั้งหมด
                'Mem/STMSize': '1',                   # สั้นมากพอ


                # ===== 🔴 ปิด visual loop closure engine =====
                'Rtabmap/LoopClosure/Strategy': '0',
                'Vis/FeatureType': '0',
                'Vis/MaxFeatures': '0',

                # ===== Sync / TF =====
                'use_action_for_goal': True,
                'approx_sync': True,  # Use approx sync for rf2o odometry
                'queue_size': 30,
                'sync_queue_size': 30,
                'wait_for_transform': 0.2,
                'tf_delay': 0.05,
                'tf_tolerance': 0.1,
                'use_sim_time': LaunchConfiguration('use_sim_time'),

                # ===== LiDAR odom assumptions =====
                'Reg/Force3DoF': 'true',
                'Optimizer/GravitySigma': '0',
            }]

    
    
    

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            name='localize_only',
            default_value='false',
            choices=['true', 'false'],
            description='Localize only, do not change loaded map'
        ),

        DeclareLaunchArgument(
            name='use_rtabmapviz',
            default_value='false',
            choices=['true', 'false'],
            description='Start rtabmapviz node'
        ),



        # ===============================
        # Sync & Rewrite all topics
        # ===============================
        Node(
            package='sync_rewrite_sensors',
            executable='sync_rewrite_sensors_and_tf',
            name='sync_rewrite_sensors_and_tf_node',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),


        # RF2O Laser Odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rf2o_laser_odometry'),
                    'launch',
                    'rf2o_laser_odometry.launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),


        # RTAB-Map SLAM node with 2D LaserScan
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters_mapping,
            remappings=[ 
                ('scan', '/scan_sync'),
            ],
            arguments=[
                '--ros-args', '--log-level', 'info'  # Changed to info for debugging
            ],
        ),

        # RTAB-Map visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_footprint_sync',  # Match RTAB-Map node
                'odom_frame_id': 'odom_sync',
                'subscribe_odom_info': True,
                'subscribe_scan': True,
                'scan_topic': '/scan_sync', # ⭐ ADD Scan topic
                'approx_sync': True,  # Match RTAB-Map node setting
                'queue_size': 30,
                'sync_queue_size': 30,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[ 
                ('scan', '/scan_sync'),
            ],
            condition=IfCondition(LaunchConfiguration('use_rtabmapviz'))
        ),
    ])

