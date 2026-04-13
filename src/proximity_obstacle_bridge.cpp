#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rb10_rmpflow_rviz
{

namespace
{

class ProximityObstacleBridgeNode : public rclcpp::Node
{
public:
  ProximityObstacleBridgeNode()
  : Node("proximity_obstacle_bridge"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("fixed_frame", "base_link");
    declare_parameter("obstacle_topic", "obstacles");
    declare_parameter("publish_rate", 30.0);
    declare_parameter("obstacle_radius", 0.05);
    declare_parameter("valid_margin", 1e-3);
    declare_parameter("range_scale", 0.001);
    declare_parameter("trigger_distance", 0.15);
    declare_parameter("rmp_flag_gate_enabled", false);
    declare_parameter("rmp_flag_topic", "/RMP_flag");
    declare_parameter("rmp_active_flag_value", 1);
    declare_parameter(
      "range_topics",
      std::vector<std::string>{
        "proximity_distance1",
        "proximity_distance2",
        "proximity_distance3",
        "proximity_distance4"});
    declare_parameter(
      "sensor_frames",
      std::vector<std::string>{"tof_W", "tof_N", "tof_E", "tof_S"});

    fixed_frame_ = get_parameter("fixed_frame").as_string();
    obstacle_radius_ = get_parameter("obstacle_radius").as_double();
    valid_margin_ = get_parameter("valid_margin").as_double();
    range_scale_ = get_parameter("range_scale").as_double();
    trigger_distance_ = get_parameter("trigger_distance").as_double();
    rmp_flag_gate_enabled_ = get_parameter("rmp_flag_gate_enabled").as_bool();
    rmp_active_flag_value_ = static_cast<int>(get_parameter("rmp_active_flag_value").as_int());
    rmp_active_ = !rmp_flag_gate_enabled_;
    range_topics_ = get_parameter("range_topics").as_string_array();
    sensor_frames_ = get_parameter("sensor_frames").as_string_array();

    if (range_topics_.size() != sensor_frames_.size()) {
      throw std::runtime_error("range_topics and sensor_frames must have the same size");
    }

    latest_ranges_.resize(range_topics_.size());

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    obstacle_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      get_parameter("obstacle_topic").as_string(),
      10);

    if (rmp_flag_gate_enabled_) {
      flag_sub_ = create_subscription<std_msgs::msg::UInt8>(
        get_parameter("rmp_flag_topic").as_string(),
        10,
        std::bind(&ProximityObstacleBridgeNode::on_rmp_flag, this, std::placeholders::_1));
    }

    range_subs_.reserve(range_topics_.size());
    for (std::size_t index = 0; index < range_topics_.size(); ++index) {
      range_subs_.push_back(create_subscription<sensor_msgs::msg::Range>(
        range_topics_[index],
        10,
        [this, index](const sensor_msgs::msg::Range::SharedPtr msg) {
          latest_ranges_[index] = *msg;
        }));
    }

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("publish_rate").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ProximityObstacleBridgeNode::publish_obstacles, this));
  }

private:
  void publish_obstacles()
  {
    if (rmp_flag_gate_enabled_ && !rmp_active_) {
      clear_obstacles_once();
      return;
    }

    visualization_msgs::msg::MarkerArray msg;
    const auto stamp = now();

    for (std::size_t index = 0; index < latest_ranges_.size(); ++index) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = fixed_frame_;
      marker.header.stamp = stamp;
      marker.ns = "proximity_obstacles";
      marker.id = static_cast<int32_t>(index);
      marker.type = visualization_msgs::msg::Marker::SPHERE;

      const auto & range_msg = latest_ranges_[index];
      if (!range_msg.has_value() || !range_is_valid(*range_msg)) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        msg.markers.push_back(marker);
        continue;
      }

      const auto sensor_transform = lookup_sensor_transform(sensor_frames_[index]);
      if (!sensor_transform.has_value()) {
        continue;
      }

      const double range_m = range_msg->range * range_scale_;
      if (range_m > trigger_distance_) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        msg.markers.push_back(marker);
        continue;
      }
      const Eigen::Vector3d direction = sensor_transform->second * Eigen::Vector3d::UnitX();
      const Eigen::Vector3d center =
        sensor_transform->first + direction * (range_m + obstacle_radius_);

      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = center.x();
      marker.pose.position.y = center.y();
      marker.pose.position.z = center.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle_radius_ * 2.0;
      marker.scale.y = obstacle_radius_ * 2.0;
      marker.scale.z = obstacle_radius_ * 2.0;
      marker.color.r = 1.0F;
      marker.color.g = 0.8F;
      marker.color.b = 0.1F;
      marker.color.a = 0.85F;
      msg.markers.push_back(marker);
    }

    obstacle_pub_->publish(msg);
    obstacles_cleared_for_inactive_ = false;
  }

  void on_rmp_flag(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    const bool requested_active = static_cast<int>(msg->data) == rmp_active_flag_value_;
    rmp_active_ = requested_active;
    if (!requested_active) {
      obstacles_cleared_for_inactive_ = false;
    }
  }

  void clear_obstacles_once()
  {
    if (obstacles_cleared_for_inactive_) {
      return;
    }

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = fixed_frame_;
    clear_marker.header.stamp = now();
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;

    visualization_msgs::msg::MarkerArray clear_array;
    clear_array.markers.push_back(clear_marker);
    obstacle_pub_->publish(clear_array);
    obstacles_cleared_for_inactive_ = true;
  }

  bool range_is_valid(const sensor_msgs::msg::Range & msg) const
  {
    if (!std::isfinite(msg.range)) {
      return false;
    }
    return
      msg.range > (msg.min_range + valid_margin_) &&
      msg.range < (msg.max_range - valid_margin_);
  }

  std::optional<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> lookup_sensor_transform(
    const std::string & sensor_frame)
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(
        fixed_frame_, sensor_frame, tf2::TimePointZero);
      const Eigen::Quaterniond quat(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z);
      return std::make_pair(
        Eigen::Vector3d(
          tf.transform.translation.x,
          tf.transform.translation.y,
          tf.transform.translation.z),
        quat.normalized().toRotationMatrix());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "TF lookup failed for %s: %s",
        sensor_frame.c_str(),
        ex.what());
      return std::nullopt;
    }
  }

  std::string fixed_frame_;
  double obstacle_radius_{0.05};
  double valid_margin_{1e-3};
  double range_scale_{0.001};
  double trigger_distance_{0.3};
  bool rmp_flag_gate_enabled_{false};
  bool rmp_active_{true};
  bool obstacles_cleared_for_inactive_{false};
  int rmp_active_flag_value_{1};
  std::vector<std::string> range_topics_;
  std::vector<std::string> sensor_frames_;
  std::vector<std::optional<sensor_msgs::msg::Range>> latest_ranges_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flag_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_subs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

}  // namespace rb10_rmpflow_rviz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rb10_rmpflow_rviz::ProximityObstacleBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
