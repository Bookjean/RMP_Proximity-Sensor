#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rb10_rmpflow_rviz/rb10_model.hpp"

namespace rb10_rmpflow_rviz
{

class InteractiveGoalMarkerNode : public rclcpp::Node
{
public:
  InteractiveGoalMarkerNode()
  : Node("interactive_goal")
  {
    declare_parameter("initial_x", 0.6);
    declare_parameter("initial_y", -0.4);
    declare_parameter("initial_z", 0.6);
    declare_parameter("initial_q", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    goal_pub_ = create_publisher<geometry_msgs::msg::Point>("goal_position", 10);
    goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "goal_marker_server",
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_topics_interface(),
      get_node_services_interface());

    goal_pose_.position.x = get_parameter("initial_x").as_double();
    goal_pose_.position.y = get_parameter("initial_y").as_double();
    goal_pose_.position.z = get_parameter("initial_z").as_double();

    RB10Model::JointVector initial_q = RB10Model::JointVector::Zero();
    const auto initial_q_values = get_parameter("initial_q").as_double_array();
    for (std::size_t index = 0; index < std::min<std::size_t>(initial_q_values.size(), 6); ++index) {
      initial_q[static_cast<int>(index)] = initial_q_values[index];
    }
    const auto context = RB10Model::forward_context(initial_q);
    Eigen::Quaterniond initial_orientation(context.link_rotations[RB10Model::TCP_RMP]);
    initial_orientation.normalize();
    goal_pose_.orientation.x = initial_orientation.x();
    goal_pose_.orientation.y = initial_orientation.y();
    goal_pose_.orientation.z = initial_orientation.z();
    goal_pose_.orientation.w = initial_orientation.w();

    create_marker();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&InteractiveGoalMarkerNode::publish_goal, this));
  }

private:
  void create_marker()
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "base_link";
    marker.name = "goal_position";
    marker.description = "Goal Pose";
    marker.pose = goal_pose_;
    marker.scale = 0.15;

    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = 0.1;
    sphere.scale.y = 0.1;
    sphere.scale.z = 0.1;
    sphere.color.r = 0.0F;
    sphere.color.g = 0.8F;
    sphere.color.b = 0.0F;
    sphere.color.a = 0.9F;

    visualization_msgs::msg::InteractiveMarkerControl visible;
    visible.always_visible = true;
    visible.markers.push_back(sphere);
    marker.controls.push_back(visible);

    marker.controls.push_back(axis_control("rotate_x", 1.0, 0.0, 0.0,
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS));
    marker.controls.push_back(axis_control("move_x", 1.0, 0.0, 0.0,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS));
    marker.controls.push_back(axis_control("rotate_y", 0.0, 0.0, 1.0,
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS));
    marker.controls.push_back(axis_control("move_y", 0.0, 0.0, 1.0,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS));
    marker.controls.push_back(axis_control("rotate_z", 0.0, 1.0, 0.0,
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS));
    marker.controls.push_back(axis_control("move_z", 0.0, 1.0, 0.0,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS));

    visualization_msgs::msg::InteractiveMarkerControl move_3d;
    move_3d.orientation.w = 1.0;
    move_3d.name = "move_3d";
    move_3d.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
    marker.controls.push_back(move_3d);

    server_->insert(
      marker,
      std::bind(&InteractiveGoalMarkerNode::feedback, this, std::placeholders::_1));
    server_->applyChanges();
  }

  visualization_msgs::msg::InteractiveMarkerControl axis_control(
    const std::string & name,
    double x,
    double y,
    double z,
    uint8_t interaction_mode) const
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = x;
    control.orientation.y = y;
    control.orientation.z = z;
    control.name = name;
    control.interaction_mode = interaction_mode;
    return control;
  }

  void feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback_msg)
  {
    goal_pose_ = feedback_msg->pose;
    const double norm =
      std::sqrt(
      goal_pose_.orientation.x * goal_pose_.orientation.x +
      goal_pose_.orientation.y * goal_pose_.orientation.y +
      goal_pose_.orientation.z * goal_pose_.orientation.z +
      goal_pose_.orientation.w * goal_pose_.orientation.w);
    if (norm > 1e-9) {
      goal_pose_.orientation.x /= norm;
      goal_pose_.orientation.y /= norm;
      goal_pose_.orientation.z /= norm;
      goal_pose_.orientation.w /= norm;
    } else {
      goal_pose_.orientation.x = 0.0;
      goal_pose_.orientation.y = 0.0;
      goal_pose_.orientation.z = 0.0;
      goal_pose_.orientation.w = 1.0;
    }
  }

  void publish_goal()
  {
    geometry_msgs::msg::Point msg;
    msg = goal_pose_.position;
    goal_pub_->publish(msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "base_link";
    pose_msg.header.stamp = now();
    pose_msg.pose = goal_pose_;
    goal_pose_pub_->publish(pose_msg);
  }

  geometry_msgs::msg::Pose goal_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};

}  // namespace rb10_rmpflow_rviz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rb10_rmpflow_rviz::InteractiveGoalMarkerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
