#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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
    declare_parameter("initialize_from_joint_state", true);
    declare_parameter("lock_orientation_to_tcp", false);
    declare_parameter("joint_state_topic", "joint_states");
    declare_parameter("goal_command_point_topic", "goal_position_cmd");
    declare_parameter("goal_command_pose_topic", "goal_pose_cmd");

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

    initialize_from_joint_state_ = get_parameter("initialize_from_joint_state").as_bool();
    lock_orientation_to_tcp_ = get_parameter("lock_orientation_to_tcp").as_bool();
    goal_initialized_ = !initialize_from_joint_state_;

    create_marker();
    if (initialize_from_joint_state_) {
      joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("joint_state_topic").as_string(),
        10,
        std::bind(&InteractiveGoalMarkerNode::on_joint_state, this, std::placeholders::_1));
    }
    goal_point_command_sub_ = create_subscription<geometry_msgs::msg::Point>(
      get_parameter("goal_command_point_topic").as_string(),
      10,
      std::bind(&InteractiveGoalMarkerNode::on_goal_point_command, this, std::placeholders::_1));
    goal_pose_command_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_command_pose_topic").as_string(),
      10,
      std::bind(&InteractiveGoalMarkerNode::on_goal_pose_command, this, std::placeholders::_1));
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

    if (!lock_orientation_to_tcp_) {
      marker.controls.push_back(axis_control("rotate_x", 1.0, 0.0, 0.0,
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS));
    }
    marker.controls.push_back(axis_control("move_x", 1.0, 0.0, 0.0,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS));
    if (!lock_orientation_to_tcp_) {
      marker.controls.push_back(axis_control("rotate_y", 0.0, 1.0, 0.0,
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS));
    }
    marker.controls.push_back(axis_control("move_y", 0.0, 1.0, 0.0,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS));
    if (!lock_orientation_to_tcp_) {
      marker.controls.push_back(axis_control("rotate_z", 0.0, 0.0, 1.0,
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS));
    }
    marker.controls.push_back(axis_control("move_z", 0.0, 0.0, 1.0,
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
    normalize_goal_orientation();
    goal_initialized_ = true;
    publish_goal();
  }

  void on_goal_point_command(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    goal_pose_.position = *msg;
    goal_initialized_ = true;
    sync_marker_pose();
    publish_goal();
  }

  void on_goal_pose_command(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_.position = msg->pose.position;
    if (!lock_orientation_to_tcp_) {
      goal_pose_.orientation = msg->pose.orientation;
      normalize_goal_orientation();
    }
    goal_initialized_ = true;
    sync_marker_pose();
    publish_goal();
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (goal_initialized_ && !lock_orientation_to_tcp_) {
      return;
    }

    RB10Model::JointVector q = RB10Model::JointVector::Zero();
    if (msg->position.size() < RB10Model::joint_names.size()) {
      return;
    }

    if (msg->name.size() >= RB10Model::joint_names.size()) {
      bool matched_all = true;
      for (std::size_t joint = 0; joint < RB10Model::joint_names.size(); ++joint) {
        bool matched = false;
        for (std::size_t index = 0; index < msg->name.size(); ++index) {
          if (msg->name[index] == RB10Model::joint_names[joint]) {
            q[static_cast<int>(joint)] = msg->position[index];
            matched = true;
            break;
          }
        }
        if (!matched) {
          matched_all = false;
          break;
        }
      }
      if (!matched_all) {
        return;
      }
    } else {
      for (std::size_t joint = 0; joint < RB10Model::joint_names.size(); ++joint) {
        q[static_cast<int>(joint)] = msg->position[joint];
      }
    }

    const auto context = RB10Model::forward_context(q);
    Eigen::Quaterniond orientation(context.link_rotations[RB10Model::TCP_RMP]);
    orientation.normalize();
    const bool was_initialized = goal_initialized_;
    bool pose_changed = false;

    if (!was_initialized) {
      const auto & tcp = context.link_positions[RB10Model::TCP_RMP];
      goal_pose_.position.x = tcp.x();
      goal_pose_.position.y = tcp.y();
      goal_pose_.position.z = tcp.z();
      goal_initialized_ = true;
      pose_changed = true;
    }

    if (lock_orientation_to_tcp_) {
      const Eigen::Quaterniond current_goal_orientation(
        goal_pose_.orientation.w,
        goal_pose_.orientation.x,
        goal_pose_.orientation.y,
        goal_pose_.orientation.z);
      if (!current_goal_orientation.coeffs().allFinite() ||
        current_goal_orientation.norm() < 1e-9 ||
        std::abs(current_goal_orientation.normalized().dot(orientation)) < 1.0 - 1e-9)
      {
        goal_pose_.orientation.x = orientation.x();
        goal_pose_.orientation.y = orientation.y();
        goal_pose_.orientation.z = orientation.z();
        goal_pose_.orientation.w = orientation.w();
        pose_changed = true;
      }
    } else if (!was_initialized) {
      goal_pose_.orientation.x = orientation.x();
      goal_pose_.orientation.y = orientation.y();
      goal_pose_.orientation.z = orientation.z();
      goal_pose_.orientation.w = orientation.w();
      pose_changed = true;
    }

    if (!pose_changed) {
      return;
    }

    sync_marker_pose();
    publish_goal();
    if (!startup_log_emitted_) {
      startup_log_emitted_ = true;
      RCLCPP_INFO(
        get_logger(),
        lock_orientation_to_tcp_ ?
        "Initialized goal marker from tcp_rmp and locking goal orientation to the live tcp_rmp orientation" :
        "Initialized startup goal marker from the current tcp_rmp pose to avoid an initial jump");
    }
  }

  void normalize_goal_orientation()
  {
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

  void sync_marker_pose()
  {
    server_->setPose("goal_position", goal_pose_);
    server_->applyChanges();
  }

  void publish_goal()
  {
    if (!goal_initialized_) {
      return;
    }

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
  bool initialize_from_joint_state_{true};
  bool goal_initialized_{false};
  bool lock_orientation_to_tcp_{false};
  bool startup_log_emitted_{false};
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_point_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_command_sub_;
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
