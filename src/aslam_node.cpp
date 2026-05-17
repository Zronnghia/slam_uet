#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// --- CẤU HÌNH CĂN CHỈNH ---
const int MAP_SIZE = 1000;
const double MAP_RES = 0.025;
const double LOG_OCC = 4.75;         
const double LOG_FREE = -0.1;       
const double MAX_SAFE_RANGE = 6.0;  
const double OCC_STRICT_LOCK = 2.0; // <--- BIẾN "CỨU TINH" CỦA BẠN ĐÂY

class AslamNode : public rclcpp::Node {
public:
    AslamNode() : Node("aslam_node") {
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
        
        scan_sub_1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_1", rclcpp::SensorDataQoS(), std::bind(&AslamNode::scan_cb, this, std::placeholders::_1));
        scan_sub_2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_2", rclcpp::SensorDataQoS(), std::bind(&AslamNode::scan_cb, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                angular_v_ = std::abs(msg->twist.twist.angular.z);
            });

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        prob_map_.assign(MAP_SIZE * MAP_SIZE, 0.0);
        init_map_msg();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { publish_map(); });
    }

private:
    void init_map_msg() {
        map_msg_.info.resolution = MAP_RES;
        map_msg_.info.width = MAP_SIZE;
        map_msg_.info.height = MAP_SIZE;
        map_msg_.info.origin.position.x = -(MAP_SIZE * MAP_RES) / 2.0;
        map_msg_.info.origin.position.y = -(MAP_SIZE * MAP_RES) / 2.0;
        map_msg_.info.origin.orientation.w = 1.0;
        map_msg_.header.frame_id = "odom";
        map_msg_.data.assign(MAP_SIZE * MAP_SIZE, -1);
    }

    void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("odom", msg->header.frame_id, 
                                             msg->header.stamp, rclcpp::Duration::from_seconds(0.05));
        } catch (const tf2::TransformException & ex) { return; }

        double sx = tf.transform.translation.x;
        double sy = tf.transform.translation.y;
        double syaw = tf2::getYaw(tf.transform.rotation);

        std::lock_guard<std::mutex> lock(map_mutex_);

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double r = msg->ranges[i];
            if (std::isnan(r) || std::isinf(r)) r = MAX_SAFE_RANGE;

            bool is_hit = (r < msg->range_max - 0.2) && (r < MAX_SAFE_RANGE);
            double angle = msg->angle_min + i * msg->angle_increment + syaw;
            
            // RAYCASTING (Xóa vùng trắng)
            double free_limit = is_hit ? (r - MAP_RES * 4.0) : std::min(r, MAX_SAFE_RANGE);
            for (double d = 0.0; d < free_limit; d += MAP_RES * 0.8) {
                double wx = sx + d * std::cos(angle);
                double wy = sy + d * std::sin(angle);
                
                int mx = std::floor((wx - map_msg_.info.origin.position.x) / MAP_RES);
                int my = std::floor((wy - map_msg_.info.origin.position.y) / MAP_RES);

                if (mx >= 0 && mx < MAP_SIZE && my >= 0 && my < MAP_SIZE) {
                    int idx = my * MAP_SIZE + mx;
                    
                    // --- DÙNG BIẾN OCC_STRICT_LOCK Ở ĐÂY ---
                    if (prob_map_[idx] > OCC_STRICT_LOCK) break; 
                    
                    prob_map_[idx] += LOG_FREE;
                    prob_map_[idx] = std::clamp(prob_map_[idx], -2.0, 20.0);
                }
            }

            // VẼ VẬT CẢN (Hit)
            if (is_hit) {
                double wx = sx + r * std::cos(angle);
                double wy = sy + r * std::sin(angle);
                update_cell_hit(wx, wy);
            }
        }
    }

    void update_cell_hit(double wx, double wy) {
        int mx = std::floor((wx - map_msg_.info.origin.position.x) / MAP_RES);
        int my = std::floor((wy - map_msg_.info.origin.position.y) / MAP_RES);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                int nx = mx + dx;
                int ny = my + dy;
                if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE) {
                    int idx = ny * MAP_SIZE + nx;
                    prob_map_[idx] += LOG_OCC;
                    prob_map_[idx] = std::clamp(prob_map_[idx], -2.0, 20.0);
                }
            }
        }
    }

    void publish_map() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        for (int i = 0; i < MAP_SIZE * MAP_SIZE; ++i) {
            if (prob_map_[i] == 0) map_msg_.data[i] = -1;
            else if (prob_map_[i] > 1.0) map_msg_.data[i] = 100;
            else if (prob_map_[i] < -0.4) map_msg_.data[i] = 0;
        }
        map_msg_.header.stamp = this->now();
        map_pub_->publish(map_msg_);
    }

    std::vector<double> prob_map_;
    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_1_, scan_sub_2_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double angular_v_ = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AslamNode>());
    rclcpp::shutdown();
    return 0;
}