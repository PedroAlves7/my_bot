-- cartographer_config.lua

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = false,       -- Set to false because you don't have wheel odometry
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,        -- Set to 1 for a single 2D LiDAR
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- Since you don't have an IMU, this is set to false
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- You can adjust this value based on your LiDAR's range
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
-- Increase this value. The default is often around 1e2.
-- This tells the optimizer to prioritize rotation matching.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4e2

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- Add these lines if they don't exist under POSE_GRAPH
POSE_GRAPH.matcher_translation_weight = 5e2
-- Increase this value significantly.
POSE_GRAPH.matcher_rotation_weight = 2e3

return options