#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SimObserver : public rclcpp::Node
{
public:
  SimObserver()
  : Node("sim_observer")
  {
    publish_cmd_vel_ = declare_parameter("publish_cmd_vel", false);
    linear_x_ = declare_parameter("linear_x", 0.08);
    angular_z_ = declare_parameter("angular_z", 0.0);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&SimObserver::on_scan, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&SimObserver::on_odom, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SimObserver::on_timer, this));

    RCLCPP_INFO(get_logger(), "sim_observer started: listening to /scan and /odom");
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_count_ = msg->ranges.size();
    last_scan_min_ = std::numeric_limits<float>::infinity();

    for (const auto range : msg->ranges) {
      if (std::isfinite(range)) {
        last_scan_min_ = std::min(last_scan_min_, range);
      }
    }

    if (!std::isfinite(last_scan_min_)) {
      last_scan_min_ = 0.0;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & position = msg->pose.pose.position;
    const auto & linear = msg->twist.twist.linear;
    const auto & angular = msg->twist.twist.angular;

    last_x_ = position.x;
    last_y_ = position.y;
    last_linear_x_ = linear.x;
    last_angular_z_ = angular.z;
    received_odom_ = true;
  }

  void on_timer()
  {
    if (publish_cmd_vel_) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = linear_x_;
      cmd.angular.z = angular_z_;
      cmd_vel_pub_->publish(cmd);
    }

    RCLCPP_INFO(
      get_logger(),
      "scan points=%zu min_range=%.2f m | odom x=%.2f y=%.2f vx=%.2f wz=%.2f%s",
      last_scan_count_,
      last_scan_min_,
      last_x_,
      last_y_,
      last_linear_x_,
      last_angular_z_,
      received_odom_ ? "" : " (waiting for /odom)");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool publish_cmd_vel_{false};
  double linear_x_{0.0};
  double angular_z_{0.0};
  bool received_odom_{false};
  std::size_t last_scan_count_{0};
  float last_scan_min_{0.0};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_linear_x_{0.0};
  double last_angular_z_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimObserver>());
  rclcpp::shutdown();
  return 0;
}
