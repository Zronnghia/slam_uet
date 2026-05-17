cmake_minimum_required(VERSION 3.8)
project(slam_uet)

# Thiết lập chuẩn C++14 hoặc 17 để hỗ trợ các thư viện hiện đại
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Tìm các gói phụ thuộc (Dependencies)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Node TinySLAM ---
add_executable(tinyslam_node src/tinyslam_node.cpp)
ament_target_dependencies(tinyslam_node 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  geometry_msgs
)

# --- Node ASLAM ---
add_executable(aslam_node src/aslam_node.cpp)
ament_target_dependencies(aslam_node 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  geometry_msgs
)

# --- CÀI ĐẶT (INSTALL) ---

# 1. Cài đặt các file thực thi (Nodes)
install(TARGETS
  tinyslam_node
  aslam_node
  DESTINATION lib/${PROJECT_NAME}
)

# 2. CÀI ĐẶT THƯ MỤC LAUNCH VÀ CONFIG (PHẦN QUAN TRỌNG BẠN CẦN THÊM)
# Việc này giúp ros2 launch tìm thấy file .launch.py và .lua
install(DIRECTORY 
  launch 
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()