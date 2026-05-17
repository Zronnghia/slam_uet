# slam_uet

`slam_uet` là một gói (package) phát triển trên nền tảng **ROS 2 Humble** (Biên dịch bằng CMake), đóng vai trò làm bộ não xử lý các thuật toán **SLAM (Simultaneous Localization and Mapping)** thuần túy. Package tích hợp linh hoạt cả giải pháp SLAM 2D và 3D, từ các thư viện chuẩn công nghiệp (Google Cartographer) đến các thuật toán mã nguồn mở viết bằng C++ tối giản (`TinySLAM`, `ASLAM`).

Gói được thiết kế theo kiến trúc module hóa độc lập, sẵn sàng tiếp nhận luồng dữ liệu cảm biến từ mọi mô hình robot di động (phần cứng thực tế hoặc môi trường mô phỏng Gazebo như `assem3`).

---

## Các tính năng cốt lõi

- **Hỗ trợ đa thuật toán:** Tích hợp đồng thời 4 giải pháp SLAM (Cartographer 2D/3D, TinySLAM, ASLAM).
- **Tối ưu hóa CPU:** Cấu hình Cartographer được tối ưu hóa sâu (sử dụng `motion_filter`) giúp giảm tải tính toán đồ thị lên tới 80% khi robot không di chuyển.
- **Phân định tọa độ chuẩn chuẩn chuẩn:** Thiết lập cây tọa độ TF rạch ròi theo chuẩn `REP-105` (`map` → `odom` → `base_footprint`), triệt tiêu hoàn toàn lỗi xung đột tọa độ (Nghịch lý hai cha).

---

## Cấu trúc Package

```text
slam_uet/
├── CMakeLists.txt                  # Cấu hình biên dịch mã nguồn C++ (CMake)
├── package.xml                     # Khai báo thông tin gói và các thư viện phụ thuộc
├── config/
│   ├── assem3_cartographer.lua     # Tham số cấu hình Cartographer SLAM 2D
│   └── assem3_cartographer3d.lua   # Tham số cấu hình Cartographer SLAM 3D
├── include/                        # Thư mục chứa các file header thư viện (.hpp)
├── launch/
│   ├── cartographer.launch.py      # Launch file kích hoạt Cartographer 2D
│   └── cartographer_3d.launch.py   # Launch file kích hoạt Cartographer 3D
└── src/
    ├── aslam_node.cpp              # Mã nguồn thuật toán ASLAM tùy biến nâng cao (C++)
    └── tinyslam_node.cpp           # Mã nguồn thuật toán TinySLAM tối giản (C++)
```
---

## Giao tiếp Topic dữ liệu (Dành cho nhà phát triển)

Để package có thể xử lý chính xác, thực thể robot phát dữ liệu cần đảm bảo cung cấp đúng các luồng topic sau:

**Topics đầu vào (Subscribed Topics):**
- **Lidar 3D PointCloud:** /points2 (sensor_msgs/msg/PointCloud2)

- **Lidar 2D LaserScan:** /scan (sensor_msgs/msg/LaserScan)

- **Dữ liệu Quán tính:** /imu (sensor_msgs/msg/Imu)

- **Dữ liệu Quán tính bánh xe:** /odom (nav_msgs/msg/Odometry)

**Topics đầu ra (Published Topics):**
- **Bản đồ lưới 2D chiếm chỗ:** /map (nav_msgs/msg/OccupancyGrid)

- **Danh sách các phân vùng bản đồ 3D:** /submap_list (cartographer_ros_msgs/msg/SubmapList)

---
## Yêu cầu hệ thống & Biên dịch

**Cài đặt các thư viện phụ thuộc**
Mở terminal và cài đặt bộ công cụ SLAM hệ thống của ROS 2 Humble:
```bash
sudo apt update
sudo apt install ros-humble-cartographer \
                 ros-humble-cartographer-ros \
                 ros-humble-cartographer-rviz \
                 ros-humble-nav2-map-server -y
```
**Biên dịch không gian làm việc**
Kích hoạt biên dịch gói slam_uet trong không gian làm việc ROS 2 (ros2_ws):
```bash
cd ~/ros2_ws
colcon build --packages-select slam_uet
source install/setup.bash
```
---
## Hướng dẫn Khởi chạy thuật toán & Lưu trữ bản đồ

> **Lưu ý quan trọng:** Trước khi khởi chạy bất kỳ node SLAM nào, hãy đảm bảo hệ thống phần cứng robot hoặc môi trường mô phỏng Gazebo đã được bật và đang publish đầy đủ các topic cảm biến (`/points2`, `/scan`, `/odom`, `/imu`).

### 1. Dành cho thuật toán SLAM 3D (Cartographer 3D)

**Bước 1: Kích hoạt bộ giải thuật**

Mở terminal và khởi chạy file launch dành riêng cho 3D:
```bash
ros2 launch slam_uet cartographer_3d.launch.py
```
**Bước 2: Cấu hình quan sát trên RViz2** 

Để hiển thị đám mây điểm 3D tích lũy theo thời gian thực:

- Đổi Fixed Frame thành map.

- Thêm Display: Add ➔ By topic ➔ /points2 (dạng PointCloud2).

- Mở rộng cấu hình PointCloud2:

    - Style: Points | Size: 2 (Giúp hiển thị hạt mịn).

    - Color Transformer: AxisColor (Trục Z - tạo hiệu ứng chuyển màu theo độ cao).

    - Decay Time: 10000 (Giữ lại lịch sử tia quét để tạo khối 3D toàn cảnh).

**Bước 3: Lưu trữ bản đồ 3D (.pbstream)**

Sau khi robot đã quét toàn bộ không gian, mở một Terminal mới để chốt quỹ đạo và lưu file:
```bash
# 1. Khóa quỹ đạo, ngừng nhận dữ liệu mới
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

# 2. Xuất dữ liệu bản đồ nguyên bản
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '~/maps/my_3d_map.pbstream'}"
```

### 2. Dành cho thuật toán SLAM 2D (Cartographer, TinySLAM, ASLAM)
Các thuật toán này phù hợp để quét nhanh mặt bằng kiến trúc phẳng. Bạn có thể chọn chạy 1 trong 3 thuật toán sau:
```bash
# Lựa chọn 1: Google Cartographer 2D
ros2 launch slam_uet cartographer.launch.py

# Lựa chọn 2: TinySLAM (C++)
ros2 run slam_uet tinyslam_node

# Lựa chọn 3: ASLAM (C++)
ros2 run slam_uet aslam_node
```

Khi bản đồ Occupancy Grid đã bao phủ kín không gian trên RViz, sử dụng công cụ của Nav2 để chụp lại bản đồ:
```bash
# Đảm bảo thư mục lưu trữ đã tồn tại
mkdir -p ~/maps

# Chụp ảnh mặt bằng và lưu thành 2 file: my_2d_map.pgm và my_2d_map.yaml
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_2d_map
```