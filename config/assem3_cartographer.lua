include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  
  -- Cấu hình 2 Lidar song song
  num_laser_scans = 2, 
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  
  -- Các tham số Sampling bắt buộc của ROS 2 Humble
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- Kích hoạt chế độ 2D
MAP_BUILDER.use_trajectory_builder_2d = true

-- ==========================================
-- CẤU HÌNH CƠ BẢN (CHỐNG LỖI VÀ CHỐNG NHÒE)
-- ==========================================
options.trajectory_builder.trajectory_builder_2d.min_range = 0.1
options.trajectory_builder.trajectory_builder_2d.max_range = 5.0
options.trajectory_builder.trajectory_builder_2d.use_imu_data = false
options.trajectory_builder.trajectory_builder_2d.use_online_correlative_scan_matching = true
options.trajectory_builder.trajectory_builder_2d.min_z = 0.05
options.trajectory_builder.trajectory_builder_2d.max_z = 1.5
options.trajectory_builder.trajectory_builder_2d.missing_data_ray_length = 0.0

-- Sửa lỗi nhòe: Gộp đủ 2 tia Lidar rồi mới tính toán
options.trajectory_builder.trajectory_builder_2d.num_accumulated_range_data = 2

-- ==========================================
-- [NÂNG CẤP 1] TỐI ƯU HÓA ĐIỂM ẢNH (VOXEL FILTER)
-- ==========================================
options.trajectory_builder.trajectory_builder_2d.voxel_filter_size = 0.025
options.trajectory_builder.trajectory_builder_2d.adaptive_voxel_filter.max_length = 0.025
options.trajectory_builder.trajectory_builder_2d.adaptive_voxel_filter.min_num_points = 400
options.trajectory_builder.trajectory_builder_2d.adaptive_voxel_filter.max_range = 15.0
options.trajectory_builder.trajectory_builder_2d.submaps.grid_options_2d.resolution = 0.025
options.trajectory_builder.trajectory_builder_2d.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.89
options.trajectory_builder.trajectory_builder_2d.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.498
options.trajectory_builder.trajectory_builder_2d.max_range = 6.0
-- ==========================================
-- [NÂNG CẤP 2] TĂNG ĐỘ CHÍNH XÁC GÓC TƯỜNG (CERES SOLVER)
-- ==========================================
options.trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.occupied_space_weight = 25.0
options.trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.translation_weight = 20.0
options.trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.rotation_weight = 50.0
options.trajectory_builder.trajectory_builder_2d.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
-- ==========================================
-- [NÂNG CẤP 3] ĐÓNG VÒNG CHỐNG TRÔI (LOOP CLOSURE)
-- ==========================================
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70

return options