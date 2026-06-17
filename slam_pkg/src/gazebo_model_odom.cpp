#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class GazeboModelOdom : public rclcpp::Node
{
public:
  GazeboModelOdom()
  : Node("gazebo_model_odom")
  {
    model_name_ = declare_parameter("model_name", "g1_rl");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base");
    model_states_topic_ = declare_parameter("model_states_topic", "/gazebo/model_states");
    entity_state_service_ = declare_parameter("entity_state_service", "/get_entity_state");
    update_rate_ = declare_parameter("update_rate", 50.0);
    use_entity_state_service_ = declare_parameter("use_entity_state_service", true);
    publish_tf_ = declare_parameter("publish_tf", true);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    entity_state_client_ = create_client<gazebo_msgs::srv::GetEntityState>(entity_state_service_);

    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&GazeboModelOdom::on_model_states, this, std::placeholders::_1));

    if (use_entity_state_service_) {
      const auto period = std::chrono::duration<double>(1.0 / update_rate_);
      service_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&GazeboModelOdom::query_entity_state, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "Publishing /odom from Gazebo model '%s' (%s -> %s); service=%s topic=%s",
      model_name_.c_str(),
      odom_frame_.c_str(),
      base_frame_.c_str(),
      entity_state_service_.c_str(),
      model_states_topic_.c_str());
  }

private:
  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    const auto it = std::find(msg->name.begin(), msg->name.end(), model_name_);
    if (it == msg->name.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Waiting for Gazebo model '%s' in %s",
        model_name_.c_str(),
        model_states_topic_.c_str());
      return;
    }

    const auto index = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
    publish_odom(msg->pose[index], msg->twist[index], get_clock()->now());
  }

  void query_entity_state()
  {
    if (service_request_in_flight_) {
      return;
    }

    if (!entity_state_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Waiting for Gazebo service %s",
        entity_state_service_.c_str());
      return;
    }

    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = model_name_;
    request->reference_frame = "world";
    service_request_in_flight_ = true;

    entity_state_client_->async_send_request(
      request,
      [this](rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future) {
        service_request_in_flight_ = false;
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Gazebo service %s has no state for model '%s'",
            entity_state_service_.c_str(),
            model_name_.c_str());
          return;
        }
        publish_odom(response->state.pose, response->state.twist, rclcpp::Time(response->header.stamp));
      });
  }

  double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q) const
  {
    const auto siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const auto cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw) const
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  void publish_odom(
    const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Twist & twist,
    const rclcpp::Time & stamp)
  {
    const auto yaw = yaw_from_quaternion(pose.orientation);
    if (!initial_pose_set_) {
      initial_x_ = pose.position.x;
      initial_y_ = pose.position.y;
      initial_yaw_ = yaw;
      initial_pose_set_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Reset odom origin at Gazebo pose x=%.3f y=%.3f yaw=%.3f",
        initial_x_,
        initial_y_,
        initial_yaw_);
    }

    const auto dx = pose.position.x - initial_x_;
    const auto dy = pose.position.y - initial_y_;
    const auto cos_yaw = std::cos(initial_yaw_);
    const auto sin_yaw = std::sin(initial_yaw_);
    const auto odom_x = cos_yaw * dx + sin_yaw * dy;
    const auto odom_y = -sin_yaw * dx + cos_yaw * dy;
    const auto odom_yaw = yaw - initial_yaw_;
    const auto yaw_orientation = quaternion_from_yaw(odom_yaw);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = odom_x;
    odom.pose.pose.position.y = odom_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = yaw_orientation;
    odom.twist.twist = twist;
    odom_pub_->publish(odom);

    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = odom.pose.pose.position.z;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string model_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string model_states_topic_;
  std::string entity_state_service_;
  double update_rate_{50.0};
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_yaw_{0.0};
  bool use_entity_state_service_{true};
  bool publish_tf_{true};
  bool service_request_in_flight_{false};
  bool initial_pose_set_{false};

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr entity_state_client_;
  rclcpp::TimerBase::SharedPtr service_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboModelOdom>());
  rclcpp::shutdown();
  return 0;
}
