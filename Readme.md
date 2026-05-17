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

## Giao tiếp Topic dữ liệu (Dành cho nhà phát triển)
Để package có thể xử lý chính xác, thực thể robot phát dữ liệu cần đảm bảo cung cấp đúng các luồng topic sau:

Topics đầu vào (Subscribed Topics):
- Lidar 3D PointCloud: /points2 (sensor_msgs/msg/PointCloud2)

- Lidar 2D LaserScan: /scan (sensor_msgs/msg/LaserScan)

- Dữ liệu Quán tính: /imu (sensor_msgs/msg/Imu)

- Dữ liệu Quán tính bánh xe: /odom (nav_msgs/msg/Odometry)

Topics đầu ra (Published Topics):
- Bản đồ lưới 2D chiếm chỗ: /map (nav_msgs/msg/OccupancyGrid)

- Danh sách các phân vùng bản đồ 3D: /submap_list (cartographer_ros_msgs/msg/SubmapList)
