#include "rb10_rmpflow_rviz/rb10_socket_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace rb10_rmpflow_rviz
{

namespace
{

constexpr std::array<const char *, 6> kJointNames = {
  "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"
};

constexpr double kPi = 3.14159265358979323846;

double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

double radians_to_degrees(double radians)
{
  return radians * 180.0 / kPi;
}

bool command_blocked_by_robot_safety(const Rb10SystemState & state)
{
  return state.op_stat_soft_estop_occur != 0 ||
         state.op_stat_ems_flag != 0 ||
         state.op_stat_collision_occur != 0 ||
         state.op_stat_self_collision != 0 ||
         state.op_stat_sos_flag != 0;
}

std::string robot_safety_summary(const Rb10SystemState & state)
{
  std::string summary;
  const auto append_flag = [&summary](const char * label, std::int32_t value) {
    if (value == 0) {
      return;
    }
    if (!summary.empty()) {
      summary += ", ";
    }
    summary += label;
  };

  append_flag("soft_estop", state.op_stat_soft_estop_occur);
  append_flag("ems", state.op_stat_ems_flag);
  append_flag("collision", state.op_stat_collision_occur);
  append_flag("self_collision", state.op_stat_self_collision);
  append_flag("sos", state.op_stat_sos_flag);

  if (summary.empty()) {
    summary = "none";
  }
  return summary;
}

}  // namespace

class AlphaBetaVelocityFilter
{
public:
  AlphaBetaVelocityFilter(double alpha, double beta, double fallback_dt_sec)
  : alpha_(alpha), beta_(beta), fallback_dt_sec_(fallback_dt_sec)
  {
  }

  void reset(
    const std::array<double, 6> & measured_position,
    const std::chrono::steady_clock::time_point & current_time)
  {
    filtered_position_ = measured_position;
    filtered_velocity_.fill(0.0);
    previous_time_ = current_time;
    initialized_ = true;
  }

  std::array<double, 6> update(
    const std::array<double, 6> & measured_position,
    const std::chrono::steady_clock::time_point & current_time)
  {
    if (!initialized_) {
      filtered_position_ = measured_position;
      filtered_velocity_.fill(0.0);
      previous_time_ = current_time;
      initialized_ = true;
      return filtered_velocity_;
    }

    const std::chrono::duration<double> dt_duration = current_time - previous_time_;
    double dt_sec = dt_duration.count();
    if (dt_sec <= 0.0 || dt_sec > 0.1) {
      dt_sec = fallback_dt_sec_;
    }

    for (std::size_t index = 0; index < 6U; ++index) {
      const double predicted_position = filtered_position_[index] + filtered_velocity_[index] * dt_sec;
      const double predicted_velocity = filtered_velocity_[index];
      const double residual = measured_position[index] - predicted_position;
      filtered_position_[index] = predicted_position + alpha_ * residual;
      filtered_velocity_[index] = predicted_velocity + (beta_ * residual) / dt_sec;
    }

    previous_time_ = current_time;
    return filtered_velocity_;
  }

private:
  double alpha_{0.5};
  double beta_{0.015};
  double fallback_dt_sec_{0.01};
  bool initialized_{false};
  std::array<double, 6> filtered_position_{};
  std::array<double, 6> filtered_velocity_{};
  std::chrono::steady_clock::time_point previous_time_{};
};

class Rb10DirectBridgeNode : public rclcpp::Node
{
public:
  Rb10DirectBridgeNode()
  : Node("rb10_direct_bridge")
  {
    declare_parameter<std::string>("robot_ip", "192.168.111.50");
    declare_parameter<bool>("simulation_mode", false);
    declare_parameter<std::string>("command_topic", "/position_controllers/commands");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>("real_joint_state_source", "measured");
    declare_parameter<bool>("publish_debug_joint_state_sources", false);
    declare_parameter<double>("publish_rate", 100.0);
    declare_parameter<double>("servo_t1", 0.002);
    declare_parameter<double>("servo_t2", 0.06);
    declare_parameter<double>("servo_gain", 0.035);
    declare_parameter<double>("servo_alpha", 0.3);
    declare_parameter<bool>("stop_on_shutdown", true);
    declare_parameter<std::string>("shutdown_action", "halt");
    declare_parameter<bool>("use_velocity_filter", true);
    declare_parameter<double>("velocity_filter_alpha", 0.5);
    declare_parameter<double>("velocity_filter_beta", 0.015);
    declare_parameter<bool>("startup_move_to_default_pose", true);
    declare_parameter<std::vector<double>>(
      "startup_home_joints_deg",
      std::vector<double>{88.82315826, 1.57005262, -108.45492554, 16.88487434, -89.99609375, 1.24207485});
    declare_parameter<double>("startup_movej_speed", 60.0);
    declare_parameter<double>("startup_movej_accel", 80.0);
    declare_parameter<double>("startup_release_tolerance_deg", 2.0);
    declare_parameter<double>("startup_release_timeout_sec", 6.0);

    robot_ip_ = get_parameter("robot_ip").as_string();
    simulation_mode_ = get_parameter("simulation_mode").as_bool();
    command_topic_ = get_parameter("command_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    real_joint_state_source_ = get_parameter("real_joint_state_source").as_string();
    publish_debug_joint_state_sources_ =
      get_parameter("publish_debug_joint_state_sources").as_bool();
    publish_rate_hz_ = get_parameter("publish_rate").as_double();
    servo_t1_ = get_parameter("servo_t1").as_double();
    servo_t2_ = get_parameter("servo_t2").as_double();
    servo_gain_ = get_parameter("servo_gain").as_double();
    servo_alpha_ = get_parameter("servo_alpha").as_double();
    stop_on_shutdown_ = get_parameter("stop_on_shutdown").as_bool();
    shutdown_action_ = get_parameter("shutdown_action").as_string();
    use_velocity_filter_ = get_parameter("use_velocity_filter").as_bool();
    velocity_filter_alpha_ = get_parameter("velocity_filter_alpha").as_double();
    velocity_filter_beta_ = get_parameter("velocity_filter_beta").as_double();
    startup_move_to_default_pose_ = get_parameter("startup_move_to_default_pose").as_bool();
    const auto startup_home_joints_deg = get_parameter("startup_home_joints_deg").as_double_array();
    if (startup_home_joints_deg.size() != 6U) {
      throw std::runtime_error("startup_home_joints_deg must contain exactly 6 values");
    }
    for (std::size_t index = 0; index < 6U; ++index) {
      startup_home_joints_deg_[index] = startup_home_joints_deg[index];
    }
    startup_movej_speed_ = get_parameter("startup_movej_speed").as_double();
    startup_movej_accel_ = get_parameter("startup_movej_accel").as_double();
    startup_release_tolerance_deg_ = get_parameter("startup_release_tolerance_deg").as_double();
    startup_release_timeout_sec_ = get_parameter("startup_release_timeout_sec").as_double();

    const double fallback_dt = 1.0 / std::max(publish_rate_hz_, 1.0);
    if (use_velocity_filter_) {
      velocity_filter_ = std::make_unique<AlphaBetaVelocityFilter>(
        velocity_filter_alpha_, velocity_filter_beta_, fallback_dt);
    }

    joint_state_publisher_ =
      create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, rclcpp::QoS(10));
    if (publish_debug_joint_state_sources_) {
      reference_joint_state_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("/rb10/reference_joint_states", rclcpp::QoS(10));
      measured_joint_state_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("/rb10/measured_joint_states", rclcpp::QoS(10));
      tracking_error_publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("/rb10/joint_tracking_error_deg", rclcpp::QoS(10));
    }
    command_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_, rclcpp::QoS(10),
      std::bind(&Rb10DirectBridgeNode::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Connecting to RB10 at %s", robot_ip_.c_str());
    const bool connected = socket_client_.connect(
      robot_ip_,
      publish_rate_hz_,
      std::bind(&Rb10DirectBridgeNode::handle_robot_state, this, std::placeholders::_1),
      std::bind(&Rb10DirectBridgeNode::handle_cmd_log, this, std::placeholders::_1));

    if (!connected) {
      throw std::runtime_error("Failed to open RB10 command/data sockets");
    }

    if (!socket_client_.initialize_robot(simulation_mode_)) {
      socket_client_.disconnect();
      throw std::runtime_error("Failed to initialize RB10 and set program mode");
    }

    if (startup_move_to_default_pose_) {
      if (!socket_client_.send_movej_degrees(
          startup_home_joints_deg_, startup_movej_speed_, startup_movej_accel_))
      {
        socket_client_.disconnect();
        throw std::runtime_error("Failed to command RB10 startup move_j to default pose");
      }
      startup_waiting_for_home_.store(true);
      startup_release_deadline_ =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(startup_release_timeout_sec_));
      RCLCPP_INFO(
        get_logger(),
        "Commanded startup move_j to default pose; delaying joint-state publication until home is reached or timeout expires");
    }

    const char * mode_label = simulation_mode_ ? "SIMULATION" : "REAL";
    RCLCPP_INFO(
      get_logger(),
      "RB10 direct C++ bridge connected to %s in %s mode (%s -> %s)",
      robot_ip_.c_str(),
      mode_label,
      command_topic_.c_str(),
      joint_state_topic_.c_str());
  }

  ~Rb10DirectBridgeNode() override
  {
    shutdown_bridge();
  }

private:
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!socket_client_.is_connected()) {
      return;
    }
    if (command_blocked_due_to_safety_.load()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Blocking move_servo_j while an RB10 safety stop is active");
      return;
    }
    if (msg->data.size() < 6U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Received %zu joint commands, expected 6",
        msg->data.size());
      return;
    }

    std::array<double, 6> joint_deg{};
    for (std::size_t index = 0; index < 6U; ++index) {
      joint_deg[index] = radians_to_degrees(msg->data[index]);
    }

    if (!socket_client_.send_servoj_degrees(
        joint_deg, servo_t1_, servo_t2_, servo_gain_, servo_alpha_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Failed to send move_servo_j command to RB10");
    }
  }

  void handle_robot_state(const Rb10SystemState & state)
  {
    if (!rclcpp::ok()) {
      return;
    }

    const bool command_blocked = command_blocked_by_robot_safety(state);
    const bool was_command_blocked = command_blocked_due_to_safety_.exchange(command_blocked);
    if (command_blocked && !was_command_blocked) {
      RCLCPP_ERROR(
        get_logger(),
        "RB10 safety stop active; blocking move_servo_j commands (%s)",
        robot_safety_summary(state).c_str());
    } else if (!command_blocked && was_command_blocked) {
      RCLCPP_INFO(get_logger(), "RB10 safety stop cleared; servo commands may resume");
    }

    std::array<double, 6> reference_position_rad{};
    std::array<double, 6> measured_position_rad{};
    for (std::size_t index = 0; index < 6U; ++index) {
      reference_position_rad[index] = degrees_to_radians(state.joint_ref_deg[index]);
      measured_position_rad[index] = degrees_to_radians(state.joint_ang_deg[index]);
    }

    std::array<double, 6> selected_position_rad{};
    const auto & joint_source_deg = select_joint_source_deg(state);
    for (std::size_t index = 0; index < 6U; ++index) {
      selected_position_rad[index] = degrees_to_radians(joint_source_deg[index]);
    }

    const auto current_time = std::chrono::steady_clock::now();
    if (startup_waiting_for_home_.load()) {
      bool home_reached = true;
      const auto & startup_joint_deg =
        simulation_mode_ ? state.joint_ref_deg : state.joint_ang_deg;
      for (std::size_t index = 0; index < 6U; ++index) {
        if (
          std::fabs(startup_joint_deg[index] - startup_home_joints_deg_[index]) >
          startup_release_tolerance_deg_)
        {
          home_reached = false;
          break;
        }
      }

      if (home_reached || current_time >= startup_release_deadline_) {
        startup_waiting_for_home_.store(false);
        reset_velocity_estimator(selected_position_rad, current_time);
        if (home_reached) {
          RCLCPP_INFO(get_logger(), "RB10 startup home pose reached; enabling joint-state publication");
        } else {
          RCLCPP_WARN(get_logger(), "RB10 startup home pose release timeout expired; enabling joint-state publication");
        }
      } else {
        return;
      }
    }

    const std::array<double, 6> estimated_velocity =
      estimate_velocity(selected_position_rad, current_time);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.reserve(kJointNames.size());
    msg.position.reserve(kJointNames.size());
    msg.velocity.reserve(kJointNames.size());
    for (std::size_t index = 0; index < kJointNames.size(); ++index) {
      msg.name.emplace_back(kJointNames[index]);
      msg.position.emplace_back(selected_position_rad[index]);
      msg.velocity.emplace_back(estimated_velocity[index]);
    }

    joint_state_publisher_->publish(msg);

    if (publish_debug_joint_state_sources_) {
      publish_debug_joint_states(reference_position_rad, measured_position_rad, msg.header.stamp);
    }
  }

  void publish_debug_joint_states(
    const std::array<double, 6> & reference_position_rad,
    const std::array<double, 6> & measured_position_rad,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (reference_joint_state_publisher_) {
      sensor_msgs::msg::JointState ref_msg;
      ref_msg.header.stamp = stamp;
      ref_msg.name.assign(kJointNames.begin(), kJointNames.end());
      ref_msg.position.assign(reference_position_rad.begin(), reference_position_rad.end());
      reference_joint_state_publisher_->publish(ref_msg);
    }

    if (measured_joint_state_publisher_) {
      sensor_msgs::msg::JointState ang_msg;
      ang_msg.header.stamp = stamp;
      ang_msg.name.assign(kJointNames.begin(), kJointNames.end());
      ang_msg.position.assign(measured_position_rad.begin(), measured_position_rad.end());
      measured_joint_state_publisher_->publish(ang_msg);
    }

    if (tracking_error_publisher_) {
      std_msgs::msg::Float64MultiArray error_msg;
      error_msg.data.resize(6U);
      for (std::size_t index = 0; index < 6U; ++index) {
        error_msg.data[index] =
          radians_to_degrees(reference_position_rad[index] - measured_position_rad[index]);
      }
      tracking_error_publisher_->publish(error_msg);
    }
  }

  const std::array<float, 6> & select_joint_source_deg(const Rb10SystemState & state) const
  {
    if (simulation_mode_) {
      return state.joint_ref_deg;
    }

    if (real_joint_state_source_ == "reference") {
      return state.joint_ref_deg;
    }

    return state.joint_ang_deg;
  }

  void handle_cmd_log(const std::string & text)
  {
    if (text.empty()) {
      return;
    }
    if (text.rfind("[socket]", 0U) == 0U) {
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
      return;
    }
    if (text == "The command was executed") {
      RCLCPP_DEBUG(get_logger(), "RB10 CMD: %s", text.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "RB10 CMD: %s", text.c_str());
  }

  void reset_velocity_estimator(
    const std::array<double, 6> & measured_position_rad,
    const std::chrono::steady_clock::time_point & current_time)
  {
    if (use_velocity_filter_) {
      velocity_filter_->reset(measured_position_rad, current_time);
      return;
    }

    previous_velocity_position_rad_ = measured_position_rad;
    previous_velocity_time_ = current_time;
    previous_velocity_sample_initialized_ = true;
  }

  std::array<double, 6> estimate_velocity(
    const std::array<double, 6> & measured_position_rad,
    const std::chrono::steady_clock::time_point & current_time)
  {
    if (use_velocity_filter_) {
      return velocity_filter_->update(measured_position_rad, current_time);
    }

    std::array<double, 6> raw_velocity{};
    if (!previous_velocity_sample_initialized_) {
      previous_velocity_position_rad_ = measured_position_rad;
      previous_velocity_time_ = current_time;
      previous_velocity_sample_initialized_ = true;
      return raw_velocity;
    }

    const std::chrono::duration<double> dt_duration = current_time - previous_velocity_time_;
    const double dt_sec = dt_duration.count();
    if (dt_sec <= 0.0 || dt_sec > 0.1) {
      previous_velocity_position_rad_ = measured_position_rad;
      previous_velocity_time_ = current_time;
      return raw_velocity;
    }

    for (std::size_t index = 0; index < 6U; ++index) {
      raw_velocity[index] =
        (measured_position_rad[index] - previous_velocity_position_rad_[index]) / dt_sec;
    }

    previous_velocity_position_rad_ = measured_position_rad;
    previous_velocity_time_ = current_time;
    return raw_velocity;
  }

  void shutdown_bridge()
  {
    if (shutdown_started_.exchange(true)) {
      return;
    }

    if (stop_on_shutdown_ && socket_client_.is_connected()) {
      const bool halt_first = shutdown_action_ != "pause";
      const bool stopped = socket_client_.send_shutdown_sequence(halt_first);
      if (stopped) {
        RCLCPP_INFO(get_logger(), "Sent RB10 shutdown stop/clear sequence");
      } else {
        RCLCPP_WARN(get_logger(), "RB10 shutdown stop/clear sequence did not complete cleanly");
      }
    }

    socket_client_.disconnect();
  }

  std::string robot_ip_;
  bool simulation_mode_{false};
  std::string command_topic_;
  std::string joint_state_topic_;
  std::string real_joint_state_source_{"measured"};
  bool publish_debug_joint_state_sources_{false};
  double publish_rate_hz_{100.0};
  double servo_t1_{0.002};
  double servo_t2_{0.06};
  double servo_gain_{0.035};
  double servo_alpha_{0.3};
  bool stop_on_shutdown_{true};
  std::string shutdown_action_{"halt"};
  bool use_velocity_filter_{true};
  double velocity_filter_alpha_{0.5};
  double velocity_filter_beta_{0.015};
  bool startup_move_to_default_pose_{true};
  std::array<double, 6> startup_home_joints_deg_{};
  double startup_movej_speed_{60.0};
  double startup_movej_accel_{80.0};
  double startup_release_tolerance_deg_{2.0};
  double startup_release_timeout_sec_{6.0};
  std::atomic<bool> startup_waiting_for_home_{false};
  std::chrono::steady_clock::time_point startup_release_deadline_{};
  std::atomic<bool> shutdown_started_{false};
  std::atomic<bool> command_blocked_due_to_safety_{false};
  Rb10SocketClient socket_client_;
  std::unique_ptr<AlphaBetaVelocityFilter> velocity_filter_;
  std::array<double, 6> previous_velocity_position_rad_{};
  std::chrono::steady_clock::time_point previous_velocity_time_{};
  bool previous_velocity_sample_initialized_{false};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr measured_joint_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tracking_error_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscription_;
};

}  // namespace rb10_rmpflow_rviz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<rb10_rmpflow_rviz::Rb10DirectBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "rb10_direct_bridge fatal error: %s\n", exception.what());
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
