#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace rb10_rmpflow_rviz
{

namespace
{

class RmpflowBridgeNode : public rclcpp::Node
{
public:
  RmpflowBridgeNode()
  : Node("rmpflow_bridge")
  {
    declare_parameter("flag_topic", std::string("/RMP_flag"));
    declare_parameter("goal_topic", std::string("/RMP_goal"));
    declare_parameter("controller_goal_topic", std::string("/goal_pose"));
    declare_parameter("controller_command_topic", std::string("/position_controllers/commands"));
    declare_parameter("target_q_topic", std::string("/target_q"));
    declare_parameter("forward_target_q", true);
    declare_parameter("goal_frame_id", std::string("base_link"));
    declare_parameter("active_flag_value", 1);
    declare_parameter("command_forward_delay_ms", 100);

    const auto flag_topic = get_parameter("flag_topic").as_string();
    const auto goal_topic = get_parameter("goal_topic").as_string();
    const auto controller_goal_topic = get_parameter("controller_goal_topic").as_string();
    const auto controller_command_topic = get_parameter("controller_command_topic").as_string();
    const auto target_q_topic = get_parameter("target_q_topic").as_string();
    forward_target_q_ = get_parameter("forward_target_q").as_bool();
    goal_frame_id_ = get_parameter("goal_frame_id").as_string();
    active_flag_value_ = static_cast<int>(get_parameter("active_flag_value").as_int());
    command_forward_delay_ = std::chrono::milliseconds(static_cast<int>(
      std::max<int64_t>(0, get_parameter("command_forward_delay_ms").as_int())));

    controller_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      controller_goal_topic,
      10);
    if (forward_target_q_) {
      target_q_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(target_q_topic, 10);
    }

    flag_sub_ = create_subscription<std_msgs::msg::UInt8>(
      flag_topic,
      10,
      std::bind(&RmpflowBridgeNode::on_flag, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::Pose>(
      goal_topic,
      10,
      std::bind(&RmpflowBridgeNode::on_goal, this, std::placeholders::_1));
    if (forward_target_q_) {
      controller_command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        controller_command_topic,
        10,
        std::bind(&RmpflowBridgeNode::on_controller_command, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "rmpflow_bridge ready: %s + %s -> %s (%s)",
      flag_topic.c_str(),
      goal_topic.c_str(),
      target_q_topic.c_str(),
      forward_target_q_ ? "forwarding target_q" : "goal forwarding only");
  }

private:
  void on_flag(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    std::scoped_lock lock(mutex_);

    const int flag_value = static_cast<int>(msg->data);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "RMP_flag: %d", flag_value);
    if (!last_flag_value_seen_ || flag_value != last_flag_value_) {
      if (flag_value == active_flag_value_) {
        RCLCPP_INFO(get_logger(), "RMP Init!!!!!");
      } else if (flag_value == 0) {
        RCLCPP_INFO(get_logger(), "RMP Standby!!!!");
      }
      last_flag_value_seen_ = true;
      last_flag_value_ = flag_value;
    }

    const bool requested_active = flag_value == active_flag_value_;
    if (requested_active == active_) {
      return;
    }
    active_ = requested_active;

    if (!active_) {
      command_forwarding_enabled_ = false;
      RCLCPP_INFO(
        get_logger(),
        forward_target_q_ ? "RMP bridge deactivated; target_q forwarding paused" :
        "RMP bridge deactivated; goal forwarding paused");
      return;
    }

    if (!goal_received_) {
      command_forwarding_enabled_ = false;
      RCLCPP_WARN(
        get_logger(),
        forward_target_q_ ?
        "RMP bridge activated but no /RMP_goal has been received yet; waiting before forwarding target_q" :
        "RMP bridge activated but no /RMP_goal has been received yet; waiting before forwarding goal_pose");
      return;
    }

    publish_external_goal_locked();
    RCLCPP_INFO(
      get_logger(),
      forward_target_q_ ?
      "RMP bridge activated; forwarding target_q once the new goal settles" :
      "RMP bridge activated; forwarding /RMP_goal to /goal_pose");
  }

  void on_goal(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    std::scoped_lock lock(mutex_);

    latest_goal_ = *msg;
    goal_received_ = true;
    if (!active_) {
      return;
    }

    publish_external_goal_locked();
  }
  void on_controller_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::scoped_lock lock(mutex_);

    if (!forward_target_q_ || !target_q_pub_) {
      return;
    }
    if (!command_forwarding_enabled_) {
      return;
    }
    if (std::chrono::steady_clock::now() < command_forwarding_allowed_after_) {
      return;
    }
    target_q_pub_->publish(*msg);
  }

  void publish_external_goal_locked()
  {
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = now();
    goal_msg.header.frame_id = goal_frame_id_;
    goal_msg.pose = latest_goal_;
    controller_goal_pub_->publish(goal_msg);

    command_forwarding_enabled_ = true;
    command_forwarding_allowed_after_ = std::chrono::steady_clock::now() + command_forward_delay_;
  }

  std::mutex mutex_;
  std::string goal_frame_id_{"base_link"};
  bool forward_target_q_{true};
  int active_flag_value_{1};
  std::chrono::milliseconds command_forward_delay_{100};
  std::chrono::steady_clock::time_point command_forwarding_allowed_after_{};

  bool active_{false};
  bool goal_received_{false};
  bool command_forwarding_enabled_{false};
  bool last_flag_value_seen_{false};
  int last_flag_value_{0};

  geometry_msgs::msg::Pose latest_goal_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr controller_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_q_pub_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flag_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr controller_command_sub_;
};

}  // namespace

}  // namespace rb10_rmpflow_rviz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rb10_rmpflow_rviz::RmpflowBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
