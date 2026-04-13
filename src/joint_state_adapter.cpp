#include <algorithm>
#include <cstddef>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace rb10_rmpflow_rviz
{

class JointStateAdapterNode : public rclcpp::Node
{
public:
  JointStateAdapterNode()
  : Node("joint_state_adapter")
  {
    declare_parameter("input_topic", std::string("/joint_states"));
    declare_parameter("output_topic", std::string("/rb10/joint_states"));
    declare_parameter(
      "source_joint_names",
      std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    declare_parameter(
      "target_joint_names",
      std::vector<std::string>{"base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"});

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    source_joint_names_ = get_parameter("source_joint_names").as_string_array();
    target_joint_names_ = get_parameter("target_joint_names").as_string_array();

    if (source_joint_names_.size() != target_joint_names_.size()) {
      throw std::runtime_error("source_joint_names and target_joint_names must have the same length");
    }

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&JointStateAdapterNode::on_joint_state, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "joint_state_adapter ready: %s -> %s",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  struct OrderedJointState
  {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
  };

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    OrderedJointState ordered_state;
    std::string mapping_source;
    if (!try_build_ordered_state(*msg, ordered_state, mapping_source)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignoring %s because the joint names do not match either the source or target map",
        input_topic_.c_str());
      return;
    }

    if (last_mapping_source_ != mapping_source) {
      RCLCPP_INFO(
        get_logger(),
        "joint_state_adapter using %s ordering for %s",
        mapping_source.c_str(),
        input_topic_.c_str());
      last_mapping_source_ = mapping_source;
    }

    sensor_msgs::msg::JointState out_msg;
    out_msg.header = msg->header;
    out_msg.name = target_joint_names_;
    out_msg.position = std::move(ordered_state.position);
    out_msg.velocity = std::move(ordered_state.velocity);
    out_msg.effort = std::move(ordered_state.effort);
    joint_state_pub_->publish(out_msg);
  }

  bool try_build_ordered_state(
    const sensor_msgs::msg::JointState & msg,
    OrderedJointState & ordered_state,
    std::string & mapping_source) const
  {
    if (msg.position.size() < target_joint_names_.size()) {
      return false;
    }

    if (try_reorder_by_names(msg, target_joint_names_, ordered_state)) {
      mapping_source = "target joint names";
      return true;
    }

    if (try_reorder_by_names(msg, source_joint_names_, ordered_state)) {
      mapping_source = "source joint names";
      return true;
    }

    ordered_state.position.assign(
      msg.position.begin(),
      msg.position.begin() + static_cast<std::ptrdiff_t>(target_joint_names_.size()));

    if (msg.velocity.size() >= target_joint_names_.size()) {
      ordered_state.velocity.assign(
        msg.velocity.begin(),
        msg.velocity.begin() + static_cast<std::ptrdiff_t>(target_joint_names_.size()));
    } else {
      ordered_state.velocity.clear();
    }

    if (msg.effort.size() >= target_joint_names_.size()) {
      ordered_state.effort.assign(
        msg.effort.begin(),
        msg.effort.begin() + static_cast<std::ptrdiff_t>(target_joint_names_.size()));
    } else {
      ordered_state.effort.clear();
    }

    mapping_source = "positional fallback";
    return true;
  }

  bool try_reorder_by_names(
    const sensor_msgs::msg::JointState & msg,
    const std::vector<std::string> & names_to_match,
    OrderedJointState & ordered_state) const
  {
    if (msg.name.size() < names_to_match.size()) {
      return false;
    }

    std::unordered_map<std::string, std::size_t> name_to_index;
    name_to_index.reserve(msg.name.size());
    for (std::size_t index = 0; index < msg.name.size(); ++index) {
      name_to_index.emplace(msg.name[index], index);
    }

    std::vector<std::size_t> ordered_indices;
    ordered_indices.reserve(names_to_match.size());
    for (const auto & name : names_to_match) {
      const auto it = name_to_index.find(name);
      if (it == name_to_index.end() || it->second >= msg.position.size()) {
        return false;
      }
      ordered_indices.push_back(it->second);
    }

    ordered_state.position.clear();
    ordered_state.position.reserve(ordered_indices.size());
    for (const auto index : ordered_indices) {
      ordered_state.position.push_back(msg.position[index]);
    }

    ordered_state.velocity.clear();
    if (msg.velocity.size() >= msg.position.size()) {
      ordered_state.velocity.reserve(ordered_indices.size());
      for (const auto index : ordered_indices) {
        ordered_state.velocity.push_back(msg.velocity[index]);
      }
    }

    ordered_state.effort.clear();
    if (msg.effort.size() >= msg.position.size()) {
      ordered_state.effort.reserve(ordered_indices.size());
      for (const auto index : ordered_indices) {
        ordered_state.effort.push_back(msg.effort[index]);
      }
    }

    return true;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string last_mapping_source_;
  std::vector<std::string> source_joint_names_;
  std::vector<std::string> target_joint_names_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

}  // namespace rb10_rmpflow_rviz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rb10_rmpflow_rviz::JointStateAdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
