#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rb10_rmpflow_rviz/rb10_model.hpp"
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

std::vector<std::string> default_range_topics()
{
  std::vector<std::string> topics;
  topics.reserve(RB10Model::sensor_control_points.size());
  for (std::size_t index = 0; index < RB10Model::sensor_control_points.size(); ++index) {
    topics.emplace_back("proximity_distance" + std::to_string(index + 1));
  }
  return topics;
}

std::vector<std::string> default_sensor_frames()
{
  std::vector<std::string> frames;
  frames.reserve(RB10Model::sensor_control_points.size());
  for (const auto & sensor : RB10Model::sensor_control_points) {
    frames.emplace_back(sensor.frame_name);
  }
  return frames;
}

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
    declare_parameter("obstacle_radii", std::vector<double>{});
    declare_parameter("valid_margin", 1e-3);
    declare_parameter("range_scale", 0.001);
    declare_parameter("minimum_hold_distance", 0.05);
    declare_parameter("trigger_distance", 0.15);
    declare_parameter("trigger_distances", std::vector<double>{});
    declare_parameter("rmp_flag_gate_enabled", false);
    declare_parameter("rmp_flag_topic", "/RMP_flag");
    declare_parameter("rmp_active_flag_value", 1);
    declare_parameter("enable_proximity_distance_1_4", true);
    declare_parameter("sensor_enabled", std::vector<bool>{});
    declare_parameter("range_topics", default_range_topics());
    declare_parameter("sensor_frames", default_sensor_frames());

    fixed_frame_ = get_parameter("fixed_frame").as_string();
    obstacle_radius_ = get_parameter("obstacle_radius").as_double();
    valid_margin_ = get_parameter("valid_margin").as_double();
    range_scale_ = get_parameter("range_scale").as_double();
    minimum_hold_distance_ = std::max(
      get_parameter("minimum_hold_distance").as_double(),
      0.0);
    trigger_distance_ = get_parameter("trigger_distance").as_double();
    rmp_flag_gate_enabled_ = get_parameter("rmp_flag_gate_enabled").as_bool();
    rmp_active_flag_value_ = static_cast<int>(get_parameter("rmp_active_flag_value").as_int());
    rmp_active_ = !rmp_flag_gate_enabled_;
    enable_proximity_distance_1_4_ =
      get_parameter("enable_proximity_distance_1_4").as_bool();
    range_topics_ = get_parameter("range_topics").as_string_array();
    sensor_frames_ = get_parameter("sensor_frames").as_string_array();
    const auto obstacle_radii = get_parameter("obstacle_radii").as_double_array();
    const auto trigger_distances = get_parameter("trigger_distances").as_double_array();
    sensor_enabled_ = get_parameter("sensor_enabled").as_bool_array();

    if (range_topics_.size() != sensor_frames_.size()) {
      throw std::runtime_error("range_topics and sensor_frames must have the same size");
    }
    if (!obstacle_radii.empty() && obstacle_radii.size() != sensor_frames_.size()) {
      throw std::runtime_error("obstacle_radii must be empty or match sensor_frames size");
    }
    if (!trigger_distances.empty() && trigger_distances.size() != sensor_frames_.size()) {
      throw std::runtime_error("trigger_distances must be empty or match sensor_frames size");
    }
    if (!sensor_enabled_.empty() && sensor_enabled_.size() != sensor_frames_.size()) {
      throw std::runtime_error("sensor_enabled must be empty or match sensor_frames size");
    }
    if (sensor_enabled_.empty()) {
      sensor_enabled_.assign(sensor_frames_.size(), true);
    }

    latest_ranges_.resize(range_topics_.size());
    obstacle_radii_.assign(sensor_frames_.size(), obstacle_radius_);
    trigger_distances_.assign(sensor_frames_.size(), trigger_distance_);
    for (std::size_t index = 0; index < obstacle_radii.size(); ++index) {
      obstacle_radii_[index] = obstacle_radii[index];
    }
    for (std::size_t index = 0; index < trigger_distances.size(); ++index) {
      trigger_distances_[index] = trigger_distances[index];
    }

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

    range_subs_.resize(range_topics_.size());
    refresh_range_subscriptions();
    parameters_callback_handle_ = add_on_set_parameters_callback(
      std::bind(
        &ProximityObstacleBridgeNode::on_set_parameters,
        this,
        std::placeholders::_1));

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
      marker.text = sensor_frames_[index];

      if (!range_topic_enabled(index)) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        msg.markers.push_back(marker);
        continue;
      }

      const auto & range_msg = latest_ranges_[index];
      if (!range_msg.has_value() || !range_is_usable(*range_msg)) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        msg.markers.push_back(marker);
        continue;
      }

      const auto sensor_transform = lookup_sensor_transform(sensor_frames_[index]);
      if (!sensor_transform.has_value()) {
        continue;
      }

      const double range_m = effective_range_m(*range_msg);
      if (range_m > trigger_distances_[index]) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        msg.markers.push_back(marker);
        continue;
      }
      const double obstacle_radius = obstacle_radii_[index];
      const Eigen::Vector3d direction = sensor_transform->second * Eigen::Vector3d::UnitX();
      const Eigen::Vector3d center =
        sensor_transform->first + direction * (range_m + obstacle_radius);

      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = center.x();
      marker.pose.position.y = center.y();
      marker.pose.position.z = center.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle_radius * 2.0;
      marker.scale.y = obstacle_radius * 2.0;
      marker.scale.z = obstacle_radius * 2.0;
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

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      if (
        parameter.get_name() != "enable_proximity_distance_1_4" &&
        parameter.get_name() != "sensor_enabled")
      {
        continue;
      }

      if (parameter.get_name() == "enable_proximity_distance_1_4") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "enable_proximity_distance_1_4 must be a bool";
          return result;
        }

        enable_proximity_distance_1_4_ = parameter.as_bool();
        refresh_range_subscriptions();
        RCLCPP_INFO(
          get_logger(),
          "proximity_distance1~4 input %s",
          enable_proximity_distance_1_4_ ? "enabled" : "disabled");
        continue;
      }

      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
        result.successful = false;
        result.reason = "sensor_enabled must be a bool array";
        return result;
      }

      const auto next_sensor_enabled = parameter.as_bool_array();
      if (next_sensor_enabled.size() != range_topics_.size()) {
        result.successful = false;
        result.reason = "sensor_enabled size must match range_topics size";
        return result;
      }

      sensor_enabled_ = next_sensor_enabled;
      refresh_range_subscriptions();
      RCLCPP_INFO(get_logger(), "updated per-sensor proximity input enable list");
    }

    return result;
  }

  void refresh_range_subscriptions()
  {
    for (std::size_t index = 0; index < range_topics_.size(); ++index) {
      if (!range_topic_enabled(index)) {
        latest_ranges_[index].reset();
        range_subs_[index].reset();
        continue;
      }

      if (range_subs_[index]) {
        continue;
      }

      range_subs_[index] = create_subscription<sensor_msgs::msg::Range>(
        range_topics_[index],
        10,
        [this, index](const sensor_msgs::msg::Range::SharedPtr msg) {
          latest_ranges_[index] = *msg;
        });
    }
  }

  bool range_topic_enabled(std::size_t index) const
  {
    if (index >= range_topics_.size()) {
      return false;
    }
    if (index >= sensor_enabled_.size() || !sensor_enabled_[index]) {
      return false;
    }
    if (!enable_proximity_distance_1_4_ && is_proximity_distance_1_4(range_topics_[index])) {
      return false;
    }
    return true;
  }

  static bool is_proximity_distance_1_4(const std::string & topic)
  {
    std::string normalized_topic = topic;
    if (!normalized_topic.empty() && normalized_topic.front() == '/') {
      normalized_topic.erase(normalized_topic.begin());
    }

    return
      normalized_topic == "proximity_distance1" ||
      normalized_topic == "proximity_distance2" ||
      normalized_topic == "proximity_distance3" ||
      normalized_topic == "proximity_distance4";
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

  bool range_is_usable(const sensor_msgs::msg::Range & msg) const
  {
    if (!std::isfinite(msg.range)) {
      return false;
    }
    if (msg.range < 0.0) {
      return false;
    }
    return msg.range < (msg.max_range - valid_margin_);
  }

  double effective_range_m(const sensor_msgs::msg::Range & msg) const
  {
    return std::max(msg.range * range_scale_, minimum_hold_distance_);
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
  double minimum_hold_distance_{0.05};
  double trigger_distance_{0.3};
  bool rmp_flag_gate_enabled_{false};
  bool rmp_active_{true};
  bool enable_proximity_distance_1_4_{true};
  bool obstacles_cleared_for_inactive_{false};
  int rmp_active_flag_value_{1};
  std::vector<std::string> range_topics_;
  std::vector<std::string> sensor_frames_;
  std::vector<bool> sensor_enabled_;
  std::vector<double> obstacle_radii_;
  std::vector<double> trigger_distances_;
  std::vector<std::optional<sensor_msgs::msg::Range>> latest_ranges_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flag_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_subs_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
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
