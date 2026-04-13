#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rb10_rmpflow_rviz
{

struct ObstacleData
{
  geometry_msgs::msg::Point position;
  double radius{0.1};
};

class ObstacleManagerNode : public rclcpp::Node
{
public:
  ObstacleManagerNode()
  : Node("obstacle_manager")
  {
    declare_parameter("num_obstacles", 1);
    declare_parameter("default_radius", 0.1);
    declare_parameter("default_positions", std::vector<double>{});

    obstacle_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", 10);
    add_obstacle_sub_ = create_subscription<visualization_msgs::msg::Marker>(
      "add_obstacle",
      10,
      std::bind(&ObstacleManagerNode::on_add_obstacle, this, std::placeholders::_1));
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "obstacle_marker_server",
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_topics_interface(),
      get_node_services_interface());

    const std::vector<geometry_msgs::msg::Point> defaults{
      make_point(0.4, -0.2, 0.5),
      make_point(0.3, 0.3, 0.4),
      make_point(0.5, 0.0, 0.3),
    };
    const auto configured_positions = get_parameter("default_positions").as_double_array();
    const auto radius = get_parameter("default_radius").as_double();
    const auto num_obstacles = get_parameter("num_obstacles").as_int();
    for (int index = 0; index < num_obstacles; ++index) {
      const std::size_t base = static_cast<std::size_t>(index) * 3;
      if (base + 2 < configured_positions.size()) {
        add_obstacle(
          index,
          make_point(
            configured_positions[base],
            configured_positions[base + 1],
            configured_positions[base + 2]),
          radius);
        continue;
      }
      if (static_cast<std::size_t>(index) >= defaults.size()) {
        add_obstacle(index, make_point(0.0, 0.0, 0.0), radius);
        continue;
      }
      add_obstacle(index, defaults[static_cast<std::size_t>(index)], radius);
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ObstacleManagerNode::publish_obstacles, this));
  }

private:
  static geometry_msgs::msg::Point make_point(double x, double y, double z)
  {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  visualization_msgs::msg::InteractiveMarkerControl axis_control(
    const std::string & name,
    double x,
    double y,
    double z) const
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = x;
    control.orientation.y = y;
    control.orientation.z = z;
    control.name = name;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    return control;
  }

  void add_obstacle(int id, const geometry_msgs::msg::Point & position, double radius)
  {
    obstacles_[id] = ObstacleData{position, radius};

    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "base_link";
    marker.name = "obstacle_" + std::to_string(id);
    marker.description = "Obstacle " + std::to_string(id);
    marker.pose.position = position;
    marker.scale = radius * 3.0;

    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = radius * 2.0;
    sphere.scale.y = radius * 2.0;
    sphere.scale.z = radius * 2.0;
    sphere.color.r = 1.0F;
    sphere.color.g = 0.2F;
    sphere.color.b = 0.2F;
    sphere.color.a = 0.7F;

    visualization_msgs::msg::InteractiveMarkerControl visible;
    visible.always_visible = true;
    visible.markers.push_back(sphere);
    marker.controls.push_back(visible);

    visualization_msgs::msg::InteractiveMarkerControl move_3d;
    move_3d.orientation.w = 1.0;
    move_3d.name = "move_3d";
    move_3d.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
    marker.controls.push_back(move_3d);

    marker.controls.push_back(axis_control("move_x", 1.0, 0.0, 0.0));
    marker.controls.push_back(axis_control("move_y", 0.0, 0.0, 1.0));
    marker.controls.push_back(axis_control("move_z", 0.0, 1.0, 0.0));

    server_->insert(
      marker,
      std::bind(&ObstacleManagerNode::feedback, this, std::placeholders::_1));
    server_->applyChanges();
  }

  void feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback_msg)
  {
    const auto name = feedback_msg->marker_name;
    const auto prefix = std::string("obstacle_");
    if (name.rfind(prefix, 0) != 0) {
      return;
    }

    const int id = std::stoi(name.substr(prefix.size()));
    auto it = obstacles_.find(id);
    if (it == obstacles_.end()) {
      return;
    }
    it->second.position = feedback_msg->pose.position;
    publish_obstacles();
  }

  void on_add_obstacle(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    if (msg->action == visualization_msgs::msg::Marker::ADD) {
      add_obstacle(msg->id, msg->pose.position, msg->scale.x * 0.5);
      publish_obstacles();
      return;
    }

    if (msg->action == visualization_msgs::msg::Marker::DELETE) {
      obstacles_.erase(msg->id);
      server_->erase("obstacle_" + std::to_string(msg->id));
      server_->applyChanges();
      publish_obstacles();
    }
  }

  void publish_obstacles()
  {
    visualization_msgs::msg::MarkerArray msg;
    for (const auto & [id, obstacle] : obstacles_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = now();
      marker.ns = "obstacles";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = obstacle.position;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle.radius * 2.0;
      marker.scale.y = obstacle.radius * 2.0;
      marker.scale.z = obstacle.radius * 2.0;
      marker.color.r = 1.0F;
      marker.color.g = 0.2F;
      marker.color.b = 0.2F;
      marker.color.a = 0.7F;
      msg.markers.push_back(marker);
    }
    obstacle_pub_->publish(msg);
  }

  std::unordered_map<int, ObstacleData> obstacles_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr add_obstacle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rb10_rmpflow_rviz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rb10_rmpflow_rviz::ObstacleManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
