#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

class Controller : public rclcpp::Node {
public:
  Controller() : Node("controller_node") {
    // QoS profile for map (TRANSIENT_LOCAL so we get the latched map)
    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_qos.reliable();
    map_qos.keep_last(1);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos,
        std::bind(&Controller::mapCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/base_pose_ground_truth", 1,
        std::bind(&Controller::odomCallback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser1", 1,
        std::bind(&Controller::lidarCallback, this, std::placeholders::_1));

    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&Controller::controlLoop, this));

    // Navigation goal
    goal_x_ = 18.0;
    goal_y_ = 7.0;

    map_received_ = false;
    pose_received_ = false;
    lidar_received_ = false;

    RCLCPP_INFO(this->get_logger(),
                "Controller Node started. Waiting for map...");
  }

private:
  // ── Subscribers / Publishers / Timer ──────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── State ──────────────────────────────────────────────────────────────────
  bool map_received_;
  bool pose_received_;
  bool lidar_received_;

  // Robot pose  [x, y, yaw]
  double pose_x_ = 0.0;
  double pose_y_ = 0.0;
  double pose_yaw_ = 0.0;

  // Goal position
  double goal_x_;
  double goal_y_;

  // Lidar
  static constexpr int LIDAR_SLICES = 8;
  std::vector<double> lidar_data_;   // one averaged range per slice
  std::vector<double> lidar_angles_; // centre angle of each slice

  // ── Callbacks ──────────────────────────────────────────────────────────────
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Store whatever map fields you need here.
    // (Map/Visualizer classes are omitted per the brief — add yours as needed.)
    (void)msg;
    map_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Map received and processed.");
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto &q = msg->pose.pose.orientation;

    // Quaternion → yaw
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    pose_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;
    pose_received_ = true;
  }

  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const int n = static_cast<int>(msg->ranges.size());
    if (n == 0)
      return;

    const int slice_size = n / LIDAR_SLICES;
    lidar_data_.resize(LIDAR_SLICES);
    lidar_angles_.resize(LIDAR_SLICES);

    for (int i = 0; i < LIDAR_SLICES; ++i) {
      int start = i * slice_size;
      int end = start + slice_size;

      // Collect finite (non-inf) readings in this slice
      std::vector<float> valid;
      valid.reserve(slice_size);
      for (int j = start; j < end && j < n; ++j) {
        if (std::isfinite(msg->ranges[j]))
          valid.push_back(msg->ranges[j]);
      }

      if (valid.empty()) {
        lidar_data_[i] = std::numeric_limits<double>::infinity();
      } else {
        double sum = std::accumulate(valid.begin(), valid.end(), 0.0);
        lidar_data_[i] = sum / static_cast<double>(valid.size());
      }

      // Centre angle of this slice
      lidar_angles_[i] =
          msg->angle_min +
          (i + 0.5) * (msg->angle_max - msg->angle_min) / LIDAR_SLICES;
    }

    lidar_received_ = true;
  }

  // ── Control loop (10 Hz) ───────────────────────────────────────────────────
  void controlLoop() {
    if (!map_received_ || !pose_received_ || !lidar_received_)
      return;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.5;
    cmd.angular.z = 0.2;

    // Simple obstacle-stop: if the centre-forward slice is closer than 1.5 m,
    // stop
    const int mid = LIDAR_SLICES / 2;
    if (lidar_data_[mid] < 1.5) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
  }
};

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
