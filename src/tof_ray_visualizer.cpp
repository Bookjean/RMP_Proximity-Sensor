#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rb10_rmpflow_rviz/rb10_model.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/color_rgba.hpp"
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

std::vector<std::string> default_sensor_frames()
{
  std::vector<std::string> frames;
  frames.reserve(RB10Model::sensor_control_points.size());
  for (const auto & sensor : RB10Model::sensor_control_points) {
    frames.emplace_back(sensor.frame_name);
  }
  return frames;
}

std::vector<std::string> default_range_topics()
{
  std::vector<std::string> topics;
  topics.reserve(RB10Model::sensor_control_points.size());
  for (std::size_t index = 0; index < RB10Model::sensor_control_points.size(); ++index) {
    topics.emplace_back("prox_distance" + std::to_string(index + 1));
  }
  return topics;
}

std::vector<std::string> default_ignored_marker_namespaces()
{
  return {"proximity_obstacles", "body_obstacles"};
}

struct SphereObstacle
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
};

class TofRayVisualizerNode : public rclcpp::Node
{
public:
  TofRayVisualizerNode()
  : Node("tof_ray_visualizer"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("fixed_frame", "base_link");
    declare_parameter("publish_rate", 20.0);
    declare_parameter("max_range", 0.2);
    declare_parameter("min_range", 0.02);
    declare_parameter("sensor_face_width", 0.25);
    declare_parameter("sensor_face_height", 0.25);
    declare_parameter("sensor_grid_resolution", 7);
    declare_parameter("edge_range_ratio", 0.6);
    declare_parameter("edge_falloff_power", 2.0);
    declare_parameter("marker_scale", 0.003);
    declare_parameter("marker_alpha", 0.5);
    declare_parameter("marker_topic", "tof_ray_markers");
    declare_parameter("obstacle_topic", "obstacles");
    declare_parameter("sensor_frames", default_sensor_frames());
    declare_parameter("range_topics", default_range_topics());
    declare_parameter("ignored_marker_namespaces", default_ignored_marker_namespaces());

    fixed_frame_ = get_parameter("fixed_frame").as_string();
    max_range_ = get_parameter("max_range").as_double();
    min_range_ = get_parameter("min_range").as_double();
    sensor_face_width_ = get_parameter("sensor_face_width").as_double();
    sensor_face_height_ = get_parameter("sensor_face_height").as_double();
    sensor_grid_resolution_ = get_parameter("sensor_grid_resolution").as_int();
    edge_range_ratio_ = get_parameter("edge_range_ratio").as_double();
    edge_falloff_power_ = get_parameter("edge_falloff_power").as_double();
    marker_scale_ = get_parameter("marker_scale").as_double();
    marker_alpha_ = get_parameter("marker_alpha").as_double();
    sensor_frames_ = get_parameter("sensor_frames").as_string_array();
    range_topics_ = get_parameter("range_topics").as_string_array();
    ignored_marker_namespaces_ = get_parameter("ignored_marker_namespaces").as_string_array();
    if (sensor_frames_.size() != range_topics_.size()) {
      throw std::runtime_error("sensor_frames and range_topics must have the same size");
    }

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    const auto marker_topic = get_parameter("marker_topic").as_string();
    const auto obstacle_topic = get_parameter("obstacle_topic").as_string();
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);

    for (const auto & topic : range_topics_) {
      range_pubs_.push_back(
        create_publisher<sensor_msgs::msg::Range>(topic, 10));
    }

    obstacle_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      obstacle_topic,
      10,
      std::bind(&TofRayVisualizerNode::on_obstacles, this, std::placeholders::_1));

    build_samples();

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("publish_rate").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TofRayVisualizerNode::publish_rays, this));
  }

private:
  struct RaySample
  {
    Eigen::Vector3d local_origin{Eigen::Vector3d::Zero()};
    double range_scale{1.0};
  };

  void build_samples()
  {
    samples_.clear();

    const int resolution = std::max<int64_t>(1, sensor_grid_resolution_);
    const double half_width = sensor_face_width_ * 0.5;
    const double half_height = sensor_face_height_ * 0.5;

    std::vector<double> coords;
    if (resolution == 1) {
      coords.push_back(0.0);
    } else {
      coords.reserve(static_cast<std::size_t>(resolution));
      for (int index = 0; index < resolution; ++index) {
        const double t = static_cast<double>(index) / static_cast<double>(resolution - 1);
        coords.push_back(-1.0 + 2.0 * t);
      }
    }

    for (const double y_norm : coords) {
      for (const double z_norm : coords) {
        const double y = y_norm * half_width;
        const double z = z_norm * half_height;
        const double radial = std::clamp(std::hypot(y_norm, z_norm) / std::sqrt(2.0), 0.0, 1.0);
        const double range_scale =
          1.0 - (1.0 - edge_range_ratio_) * std::pow(radial, edge_falloff_power_);
        samples_.push_back({Eigen::Vector3d(0.0, y, z), range_scale});
      }
    }
  }

  void on_obstacles(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    obstacles_.clear();
    for (const auto & marker : msg->markers) {
      if (marker.action != visualization_msgs::msg::Marker::ADD) {
        continue;
      }
      if (marker.type != visualization_msgs::msg::Marker::SPHERE) {
        continue;
      }
      if (std::find(
          ignored_marker_namespaces_.begin(),
          ignored_marker_namespaces_.end(),
          marker.ns) != ignored_marker_namespaces_.end())
      {
        continue;
      }

      SphereObstacle obstacle;
      obstacle.center = Eigen::Vector3d(
        marker.pose.position.x,
        marker.pose.position.y,
        marker.pose.position.z);
      obstacle.radius = marker.scale.x * 0.5;
      if (obstacle.radius > 0.0) {
        obstacles_.push_back(obstacle);
      }
    }
  }

  void publish_rays()
  {
    const auto stamp = now();
    visualization_msgs::msg::MarkerArray marker_array;

    for (std::size_t sensor_index = 0; sensor_index < sensor_frames_.size(); ++sensor_index) {
      const auto transform = lookup_sensor_transform(sensor_frames_[sensor_index]);
      if (!transform.has_value()) {
        continue;
      }

      const Eigen::Vector3d origin = transform->first;
      const Eigen::Matrix3d rotation = transform->second;
      const Eigen::Vector3d direction = rotation * Eigen::Vector3d::UnitX();

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = fixed_frame_;
      marker.header.stamp = stamp;
      marker.ns = "tof_rays";
      marker.id = static_cast<int32_t>(sensor_index);
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = marker_scale_;
      marker.pose.orientation.w = 1.0;
      marker.color = sensor_color(sensor_index);
      marker.color.a = marker_alpha_;

      double representative_range = max_range_;
      for (const auto & sample : samples_) {
        const Eigen::Vector3d ray_origin = origin + rotation * sample.local_origin;
        const double ray_limit = max_range_ * sample.range_scale;
        const double hit = nearest_hit_distance(ray_origin, direction, ray_limit);
        representative_range = std::min(representative_range, hit);

        const Eigen::Vector3d ray_end = ray_origin + direction * hit;
        marker.points.push_back(to_point(ray_origin));
        marker.points.push_back(to_point(ray_end));
      }

      marker_array.markers.push_back(marker);
      range_pubs_[sensor_index]->publish(make_range_msg(stamp, sensor_frames_[sensor_index], representative_range));
    }

    marker_pub_->publish(marker_array);
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
        get_logger(), *get_clock(), 2000, "TF lookup failed for %s: %s",
        sensor_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  double nearest_hit_distance(
    const Eigen::Vector3d & ray_origin,
    const Eigen::Vector3d & ray_direction,
    double ray_limit) const
  {
    double best = ray_limit;
    bool hit_found = false;

    for (const auto & obstacle : obstacles_) {
      const auto hit = intersect_sphere(ray_origin, ray_direction, obstacle, ray_limit);
      if (hit.has_value()) {
        best = std::min(best, *hit);
        hit_found = true;
      }
    }

    return hit_found ? best : ray_limit;
  }

  std::optional<double> intersect_sphere(
    const Eigen::Vector3d & ray_origin,
    const Eigen::Vector3d & ray_direction,
    const SphereObstacle & obstacle,
    double ray_limit) const
  {
    const Eigen::Vector3d oc = obstacle.center - ray_origin;
    const double forward = oc.dot(ray_direction);
    if (forward < min_range_) {
      return std::nullopt;
    }

    const double closest_sq = oc.squaredNorm() - forward * forward;
    const double radius_sq = obstacle.radius * obstacle.radius;
    if (closest_sq > radius_sq) {
      return std::nullopt;
    }

    const double thc = std::sqrt(std::max(radius_sq - closest_sq, 0.0));
    double hit_dist = forward - thc;
    if (hit_dist < min_range_) {
      hit_dist = forward + thc;
    }
    if (hit_dist < min_range_ || hit_dist > ray_limit) {
      return std::nullopt;
    }
    return hit_dist;
  }

  sensor_msgs::msg::Range make_range_msg(
    const rclcpp::Time & stamp,
    const std::string & frame_id,
    double range) const
  {
    sensor_msgs::msg::Range msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = static_cast<float>(M_PI);
    msg.min_range = static_cast<float>(min_range_);
    msg.max_range = static_cast<float>(max_range_);
    msg.range = static_cast<float>(std::clamp(range, min_range_, max_range_));
    return msg;
  }

  std_msgs::msg::ColorRGBA sensor_color(std::size_t index) const
  {
    static const std::array<std::array<float, 3>, 12> palette{{
      {{0.0F, 0.9F, 0.9F}},
      {{1.0F, 0.6F, 0.1F}},
      {{0.4F, 0.8F, 0.2F}},
      {{0.9F, 0.2F, 0.6F}},
      {{0.2F, 0.5F, 1.0F}},
      {{1.0F, 0.3F, 0.3F}},
      {{0.7F, 0.7F, 0.1F}},
      {{0.5F, 0.3F, 1.0F}},
      {{0.1F, 0.8F, 0.6F}},
      {{1.0F, 0.4F, 0.8F}},
      {{0.8F, 0.5F, 0.2F}},
      {{0.6F, 0.8F, 1.0F}},
    }};

    std_msgs::msg::ColorRGBA color;
    const auto & rgb = palette[index % palette.size()];
    color.r = rgb[0];
    color.g = rgb[1];
    color.b = rgb[2];
    color.a = 1.0F;
    return color;
  }

  geometry_msgs::msg::Point to_point(const Eigen::Vector3d & point) const
  {
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = point.z();
    return msg;
  }

  std::string fixed_frame_;
  double max_range_{0.2};
  double min_range_{0.02};
  double sensor_face_width_{0.25};
  double sensor_face_height_{0.25};
  int64_t sensor_grid_resolution_{7};
  double edge_range_ratio_{0.6};
  double edge_falloff_power_{2.0};
  double marker_scale_{0.003};
  double marker_alpha_{0.5};

  std::vector<std::string> sensor_frames_;
  std::vector<std::string> range_topics_;
  std::vector<std::string> ignored_marker_namespaces_;
  std::vector<RaySample> samples_;
  std::vector<SphereObstacle> obstacles_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> range_pubs_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

}  // namespace rb10_rmpflow_rviz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rb10_rmpflow_rviz::TofRayVisualizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
