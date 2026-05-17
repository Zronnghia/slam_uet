include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",         
  published_frame = "odom",  
  odom_frame = "odom",
  provide_odom_frame = false,          
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1, -- CHUẨN 1 LIDAR
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

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1 

TRAJECTORY_BUILDER_3D.min_range = 0.3
TRAJECTORY_BUILDER_3D.max_range = 10.0
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.02
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.2
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 120 

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 6 

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 60 
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 5
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options