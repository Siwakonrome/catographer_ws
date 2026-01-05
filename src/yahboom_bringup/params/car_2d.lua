include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint_sync",
  published_frame = "base_footprint_sync",
  odom_frame = "odom",
  -- Change true to false if you don't want to provide odometry data
  provide_odom_frame = true,
  -- Change false to true to publish only 2D pose
  publish_frame_projected_to_2d = true,
  -- Change false to true to use odometry data
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  -- Change 0 to 1 to use one laser scanner
  num_laser_scans = 1,
  -- Change 1 to 0 to not use multi-echo laser scans
  num_multi_echo_laser_scans = 0,
  -- Change 10 to 1, 1/1=1 means no subdivision
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


-- Change false to true to enable 2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

-- Change 0 to 0.10, ignore all measurements smaller than robot radius
TRAJECTORY_BUILDER_2D.min_range = 0.10
-- Change 30 to 3.5, limit to maximum laser scan range, smaller values generally more accurate
TRAJECTORY_BUILDER_2D.max_range = 3.5
-- Change 5 to 3, maximum value for sensor data exceeding valid range
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- Change true to false to not use IMU data, you can enable it and compare the results
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- Change false to true to use real-time loop closure detection for front-end scan matching
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- Change 1.0 to 0.1 to increase sensitivity to motion
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- Change 0.55 to 0.65, minimum score for Fast CSM, optimization only occurs above this score
POSE_GRAPH.constraint_builder.min_score = 0.65
-- Change 0.6 to 0.7, minimum score for global localization, below this score indicates inaccurate global localization
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- Set to 0 to disable global SLAM
POSE_GRAPH.optimize_every_n_nodes = 0

return options

