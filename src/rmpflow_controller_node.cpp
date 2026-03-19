#include <pthread.h>
#include <sys/mman.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rb10_rmpflow_rviz/pinocchio_direct_solver.hpp"
#include "rb10_rmpflow_rviz/rb10_model.hpp"
#include "rb10_rmpflow_rviz/rmp_solver_interface.hpp"

namespace rb10_rmpflow_rviz
{

namespace
{

using JointVector = RB10Model::JointVector;

struct RobotState
{
  JointVector q{JointVector::Zero()};
  JointVector qd{JointVector::Zero()};
};

struct ExternalRmpBuffer
{
  int dim{0};
  Eigen::MatrixXd metric_sqrt{Eigen::MatrixXd::Zero(0, 0)};
  Eigen::VectorXd acceleration{Eigen::VectorXd::Zero(0)};
  bool has_metric{false};
  bool has_acceleration{false};
};

class ControllerBackend
{
public:
  virtual ~ControllerBackend() = default;

  virtual RobotState read_state() = 0;
  virtual void apply_acceleration(
    const JointVector & qdd,
    double dt,
    const std::array<const char *, 6> & joint_names) = 0;
};

class SimulationBackend : public ControllerBackend
{
public:
  explicit SimulationBackend(const JointVector & initial_q)
  {
    state_.q = initial_q;
  }

  RobotState read_state() override
  {
    std::scoped_lock lock(mutex_);
    return state_;
  }

  void apply_acceleration(
    const JointVector & qdd,
    double dt,
    const std::array<const char *, 6> &) override
  {
    std::scoped_lock lock(mutex_);
    state_.qd += qdd * dt;
    state_.q += state_.qd * dt;
    state_.q = RB10Model::clamp_positions(state_.q);
    for (int index = 0; index < state_.q.size(); ++index) {
      if (
        (state_.q[index] <= RB10Model::joint_lower_limits[index] + 0.1 && state_.qd[index] < 0.0) ||
        (state_.q[index] >= RB10Model::joint_upper_limits[index] - 0.1 && state_.qd[index] > 0.0))
      {
        state_.qd[index] *= 0.5;
      }
    }
  }

private:
  std::mutex mutex_;
  RobotState state_;
};

class HardwareBridgeBackend : public ControllerBackend
{
public:
  HardwareBridgeBackend(
    rclcpp::Node * node,
    const JointVector & initial_q,
    const std::string & state_topic,
    const std::string & command_topic)
  : node_(node)
  {
    state_.q = initial_q;
    command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(command_topic, 10);
    state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      state_topic,
      10,
      std::bind(&HardwareBridgeBackend::on_state, this, std::placeholders::_1));
  }

  RobotState read_state() override
  {
    std::scoped_lock lock(mutex_);
    return state_;
  }

  void apply_acceleration(
    const JointVector & qdd,
    double dt,
    const std::array<const char *, 6> & joint_names) override
  {
    sensor_msgs::msg::JointState command;
    command.header.stamp = node_->now();
    command.name.assign(joint_names.begin(), joint_names.end());

    {
      std::scoped_lock lock(mutex_);
      state_.qd += qdd * dt;
      state_.q += state_.qd * dt;
      state_.q = RB10Model::clamp_positions(state_.q);
      command.position.assign(state_.q.data(), state_.q.data() + state_.q.size());
      command.velocity.assign(state_.qd.data(), state_.qd.data() + state_.qd.size());
    }

    command_pub_->publish(command);
  }

private:
  void on_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < RB10Model::joint_names.size()) {
      return;
    }

    std::scoped_lock lock(mutex_);
    for (std::size_t index = 0; index < RB10Model::joint_names.size(); ++index) {
      state_.q[static_cast<int>(index)] = msg->position[index];
      state_.qd[static_cast<int>(index)] =
        index < msg->velocity.size() ? msg->velocity[index] : 0.0;
    }
  }

  rclcpp::Node * node_;
  std::mutex mutex_;
  RobotState state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
};

bool enable_realtime(
  rclcpp::Logger logger,
  bool enabled,
  int priority,
  bool lock_memory)
{
  if (!enabled) {
    return false;
  }

  if (lock_memory && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_WARN(logger, "mlockall failed; continuing without memory locking");
  }

  sched_param params{};
  params.sched_priority = priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &params) != 0) {
    RCLCPP_WARN(logger, "Failed to switch control thread to SCHED_FIFO priority %d", priority);
    return false;
  }

  RCLCPP_INFO(logger, "Control thread running with SCHED_FIFO priority %d", priority);
  return true;
}

class RmpflowControllerNode : public rclcpp::Node
{
public:
  RmpflowControllerNode()
  : Node("rmpflow_controller"), running_(true)
  {
    declare_parameter("control_rate", 100.0);
    declare_parameter("visualization_rate", 20.0);
    declare_parameter("goal_x", 0.6);
    declare_parameter("goal_y", -0.4);
    declare_parameter("goal_z", 0.6);
    declare_parameter("min_goal_z", 0.05);
    declare_parameter("safety_stop_on_min_z", true);
    declare_parameter("workspace_floor_z", 0.0);
    declare_parameter("min_link6_z", 0.03);
    declare_parameter("min_tcp_z", 0.05);
    declare_parameter("body_goal", std::vector<double>{0.45, 0.0, 0.9});
    declare_parameter("orientation_goal", std::vector<double>{0.0, 0.0, 1.0});
    declare_parameter("initial_q", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter("default_q", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter("joint_limit_buffers", std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
    declare_parameter("backend_mode", "simulation");
    declare_parameter("hardware_state_topic", "hardware_joint_states");
    declare_parameter("hardware_command_topic", "hardware_joint_command");
    declare_parameter("enable_realtime", false);
    declare_parameter("realtime_priority", 80);
    declare_parameter("lock_memory", false);
    declare_parameter("graph.node_names", default_rmp_graph_node_names());
    declare_parameter("root_solve_offset", 1e-3);
    declare_parameter("solve_method", "rmp2");
    declare_parameter("rmp_type", "canonical");
    declare_parameter("pinocchio_urdf_path", "");
    declare_parameter(
      "external_rmp.topic_prefix",
      std::string("external_rmp"));
    declare_parameter(
      "body_obstacles.names",
      rclcpp::ParameterValue(std::vector<std::string>{}));
    declare_parameter("cspace_target_metric_scalar", 0.005);
    declare_parameter("cspace_target_position_gain", 100.0);
    declare_parameter("cspace_target_damping_gain", 50.0);
    declare_parameter("cspace_target_robust_position_term_thresh", 0.5);
    declare_parameter("cspace_target_inertia", 0.0001);
    declare_parameter("joint_limit_metric_scalar", 0.1);
    declare_parameter("joint_limit_metric_length_scale", 0.01);
    declare_parameter("joint_limit_metric_exploder_eps", 0.001);
    declare_parameter("joint_limit_metric_velocity_gate_length_scale", 0.01);
    declare_parameter("joint_limit_accel_damper_gain", 200.0);
    declare_parameter("joint_limit_accel_potential_gain", 1.0);
    declare_parameter("joint_limit_accel_potential_exploder_eps", 0.01);
    declare_parameter("joint_limit_accel_potential_exploder_length_scale", 0.1);
    declare_parameter("joint_velocity_cap_max_velocity", 1.7);
    declare_parameter("joint_velocity_cap_velocity_damping_region", 0.15);
    declare_parameter("joint_velocity_cap_damping_gain", 5.0);
    declare_parameter("joint_velocity_cap_metric_weight", 0.05);
    declare_parameter("max_joint_accel", 20.0);
    declare_parameter("target_rmp_accel_p_gain", 50.0);
    declare_parameter("target_rmp_accel_d_gain", 70.0);
    declare_parameter("target_rmp_accel_norm_eps", 0.075);
    declare_parameter("target_rmp_metric_alpha_length_scale", 0.05);
    declare_parameter("target_rmp_min_metric_alpha", 0.03);
    declare_parameter("target_rmp_max_metric_scalar", 1.0);
    declare_parameter("target_rmp_min_metric_scalar", 0.5);
    declare_parameter("target_rmp_proximity_metric_boost_scalar", 3.0);
    declare_parameter("target_rmp_proximity_metric_boost_length_scale", 0.02);
    declare_parameter("orientation_rmp_accel_p_gain", 6.0);
    declare_parameter("orientation_rmp_accel_d_gain", 10.0);
    declare_parameter("orientation_rmp_metric_scalar", 0.08);
    declare_parameter("collision_rmp_margin", 0.0);
    declare_parameter("collision_rmp_damping_gain", 50.0);
    declare_parameter("collision_rmp_damping_std_dev", 0.04);
    declare_parameter("collision_rmp_damping_robustness_eps", 0.01);
    declare_parameter("collision_rmp_damping_velocity_gate_length_scale", 0.01);
    declare_parameter("collision_rmp_repulsion_gain", 800.0);
    declare_parameter("collision_rmp_repulsion_std_dev", 0.01);
    declare_parameter("collision_rmp_metric_modulation_radius", 0.5);
    declare_parameter("collision_rmp_metric_scalar", 1.0);
    declare_parameter("collision_rmp_metric_exploder_std_dev", 0.02);
    declare_parameter("collision_rmp_metric_exploder_eps", 0.001);
    declare_parameter("damping_rmp_accel_d_gain", 30.0);
    declare_parameter("damping_rmp_metric_scalar", 0.005);
    declare_parameter("damping_rmp_inertia", 0.3);

    const auto initial_q = get_parameter("initial_q").as_double_array();
    state_.q = JointVector::Zero();
    state_.qd = JointVector::Zero();
    for (std::size_t index = 0; index < std::min<std::size_t>(initial_q.size(), 6); ++index) {
      state_.q[static_cast<int>(index)] = initial_q[index];
    }
    const auto initial_context = RB10Model::forward_context(state_.q);
    goal_ = Eigen::Vector3d(
      get_parameter("goal_x").as_double(),
      get_parameter("goal_y").as_double(),
      get_parameter("goal_z").as_double());
    body_goal_ = parse_vector3_parameter("body_goal", Eigen::Vector3d(0.45, 0.0, 0.9));
    goal_orientation_ = Eigen::Quaterniond(initial_context.link_rotations[RB10Model::TCP_RMP]);
    goal_orientation_.normalize();
    declare_graph_parameters();
    declare_body_obstacle_parameters();
    obstacles_.push_back(ObstacleSphere{});
    const auto solver_config = build_solver_config();
    configure_external_rmp_inputs(solver_config.graph_nodes);
    solver_ = build_solver(solver_config);
    body_obstacles_visual_ = solver_config.body_obstacles;

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
    control_point_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("control_points", 10);
    body_obstacle_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("body_obstacle_markers", 10);
    eef_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("end_effector_pose", 10);
    debug_state_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("rmp_debug_state", 10);

    goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "goal_position",
      10,
      std::bind(&RmpflowControllerNode::on_goal, this, std::placeholders::_1));
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose",
      10,
      std::bind(&RmpflowControllerNode::on_goal_pose, this, std::placeholders::_1));
    obstacle_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "obstacles",
      10,
      std::bind(&RmpflowControllerNode::on_obstacles, this, std::placeholders::_1));

    const auto backend_mode = get_parameter("backend_mode").as_string();
    if (backend_mode == "hardware_bridge") {
      backend_ = std::make_unique<HardwareBridgeBackend>(
        this,
        state_.q,
        get_parameter("hardware_state_topic").as_string(),
        get_parameter("hardware_command_topic").as_string());
      RCLCPP_INFO(get_logger(), "Using hardware bridge backend");
    } else {
      backend_ = std::make_unique<SimulationBackend>(state_.q);
      RCLCPP_INFO(get_logger(), "Using simulation backend");
    }

    publish_joint_states(backend_->read_state());

    const auto visualization_period = std::chrono::duration<double>(
      1.0 / get_parameter("visualization_rate").as_double());
    visualization_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(visualization_period),
      std::bind(&RmpflowControllerNode::publish_visualization, this));

    control_thread_ = std::thread(&RmpflowControllerNode::control_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "C++ RMPflow controller started at %.1f Hz",
      get_parameter("control_rate").as_double());
  }

  ~RmpflowControllerNode() override
  {
    running_.store(false);
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
  }

private:
  void on_goal(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    std::scoped_lock lock(goal_mutex_);
    goal_ = Eigen::Vector3d(
      msg->x,
      msg->y,
      std::max(msg->z, get_parameter("min_goal_z").as_double()));
  }

  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::scoped_lock lock(goal_mutex_);
    goal_ = Eigen::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y,
      std::max(msg->pose.position.z, get_parameter("min_goal_z").as_double()));
    Eigen::Quaterniond goal_orientation(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z);
    if (!goal_orientation.coeffs().allFinite() || goal_orientation.norm() < 1e-9) {
      goal_orientation_ = Eigen::Quaterniond::Identity();
    } else {
      goal_orientation_ = goal_orientation.normalized();
    }
  }

  void on_obstacles(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    std::vector<ObstacleSphere> obstacles;
    for (const auto & marker : msg->markers) {
      if (marker.action != visualization_msgs::msg::Marker::ADD) {
        continue;
      }
      obstacles.push_back(ObstacleSphere{
        Eigen::Vector3d(
          marker.pose.position.x,
          marker.pose.position.y,
          marker.pose.position.z),
        marker.scale.x * 0.5
      });
    }
    if (obstacles.empty()) {
      obstacles.push_back(ObstacleSphere{});
    }
    std::scoped_lock lock(obstacles_mutex_);
    obstacles_ = obstacles;
  }

  EigenRmpConfig build_solver_config() const
  {
    EigenRmpConfig config;
    config.graph_nodes = parse_graph_nodes();
    config.solve_offset = get_parameter("root_solve_offset").as_double();
    config.solve_method = get_parameter("solve_method").as_string();
    config.rmp_type = get_parameter("rmp_type").as_string();
    config.body_obstacles = parse_body_obstacles();

    const auto default_q = get_parameter("default_q").as_double_array();
    for (std::size_t index = 0; index < std::min<std::size_t>(default_q.size(), 6); ++index) {
      config.default_q[index] = default_q[index];
    }

    const auto joint_limit_buffers = get_parameter("joint_limit_buffers").as_double_array();
    for (std::size_t index = 0; index < std::min<std::size_t>(joint_limit_buffers.size(), 6); ++index) {
      config.joint_limit_buffers[index] = joint_limit_buffers[index];
    }

    config.cspace_target.metric_scalar = get_parameter("cspace_target_metric_scalar").as_double();
    config.cspace_target.position_gain = get_parameter("cspace_target_position_gain").as_double();
    config.cspace_target.damping_gain = get_parameter("cspace_target_damping_gain").as_double();
    config.cspace_target.robust_position_term_thresh =
      get_parameter("cspace_target_robust_position_term_thresh").as_double();
    config.cspace_target.inertia = get_parameter("cspace_target_inertia").as_double();

    config.joint_limit.metric_scalar = get_parameter("joint_limit_metric_scalar").as_double();
    config.joint_limit.metric_length_scale =
      get_parameter("joint_limit_metric_length_scale").as_double();
    config.joint_limit.metric_exploder_eps =
      get_parameter("joint_limit_metric_exploder_eps").as_double();
    config.joint_limit.metric_velocity_gate_length_scale =
      get_parameter("joint_limit_metric_velocity_gate_length_scale").as_double();
    config.joint_limit.accel_damper_gain =
      get_parameter("joint_limit_accel_damper_gain").as_double();
    config.joint_limit.accel_potential_gain =
      get_parameter("joint_limit_accel_potential_gain").as_double();
    config.joint_limit.accel_potential_exploder_eps =
      get_parameter("joint_limit_accel_potential_exploder_eps").as_double();
    config.joint_limit.accel_potential_exploder_length_scale =
      get_parameter("joint_limit_accel_potential_exploder_length_scale").as_double();

    config.joint_velocity_cap.max_velocity =
      get_parameter("joint_velocity_cap_max_velocity").as_double();
    config.joint_velocity_cap.velocity_damping_region =
      get_parameter("joint_velocity_cap_velocity_damping_region").as_double();
    config.joint_velocity_cap.damping_gain =
      get_parameter("joint_velocity_cap_damping_gain").as_double();
    config.joint_velocity_cap.metric_weight =
      get_parameter("joint_velocity_cap_metric_weight").as_double();

    config.target.accel_p_gain = get_parameter("target_rmp_accel_p_gain").as_double();
    config.target.accel_d_gain = get_parameter("target_rmp_accel_d_gain").as_double();
    config.target.accel_norm_eps = get_parameter("target_rmp_accel_norm_eps").as_double();
    config.target.metric_alpha_length_scale =
      get_parameter("target_rmp_metric_alpha_length_scale").as_double();
    config.target.min_metric_alpha =
      get_parameter("target_rmp_min_metric_alpha").as_double();
    config.target.max_metric_scalar =
      get_parameter("target_rmp_max_metric_scalar").as_double();
    config.target.min_metric_scalar =
      get_parameter("target_rmp_min_metric_scalar").as_double();
    config.target.proximity_metric_boost_scalar =
      get_parameter("target_rmp_proximity_metric_boost_scalar").as_double();
    config.target.proximity_metric_boost_length_scale =
      get_parameter("target_rmp_proximity_metric_boost_length_scale").as_double();
    config.orientation.accel_p_gain =
      get_parameter("orientation_rmp_accel_p_gain").as_double();
    config.orientation.accel_d_gain =
      get_parameter("orientation_rmp_accel_d_gain").as_double();
    config.orientation.metric_scalar =
      get_parameter("orientation_rmp_metric_scalar").as_double();

    config.collision.margin = get_parameter("collision_rmp_margin").as_double();
    config.collision.damping_gain = get_parameter("collision_rmp_damping_gain").as_double();
    config.collision.damping_std_dev = get_parameter("collision_rmp_damping_std_dev").as_double();
    config.collision.damping_robustness_eps =
      get_parameter("collision_rmp_damping_robustness_eps").as_double();
    config.collision.damping_velocity_gate_length_scale =
      get_parameter("collision_rmp_damping_velocity_gate_length_scale").as_double();
    config.collision.repulsion_gain =
      get_parameter("collision_rmp_repulsion_gain").as_double();
    config.collision.repulsion_std_dev =
      get_parameter("collision_rmp_repulsion_std_dev").as_double();
    config.collision.metric_modulation_radius =
      get_parameter("collision_rmp_metric_modulation_radius").as_double();
    config.collision.metric_scalar =
      get_parameter("collision_rmp_metric_scalar").as_double();
    config.collision.metric_exploder_std_dev =
      get_parameter("collision_rmp_metric_exploder_std_dev").as_double();
    config.collision.metric_exploder_eps =
      get_parameter("collision_rmp_metric_exploder_eps").as_double();

    config.damping.accel_d_gain = get_parameter("damping_rmp_accel_d_gain").as_double();
    config.damping.metric_scalar = get_parameter("damping_rmp_metric_scalar").as_double();
    config.damping.inertia = get_parameter("damping_rmp_inertia").as_double();
    return config;
  }

  std::unique_ptr<RmpSolverInterface> build_solver(const EigenRmpConfig & config) const
  {
    return std::make_unique<PinocchioDirectRmpSolver>(config, resolve_pinocchio_urdf_path());
  }

  std::string resolve_pinocchio_urdf_path() const
  {
    const auto configured = get_parameter("pinocchio_urdf_path").as_string();
    if (!configured.empty()) {
      return configured;
    }
    return ament_index_cpp::get_package_share_directory("rb10_rmpflow_rviz") +
           "/urdf/rb10_1300e.urdf";
  }

  void declare_graph_parameters()
  {
    const auto node_names = get_parameter("graph.node_names").as_string_array();
    const auto defaults = default_rmp_graph_nodes();
    for (const auto & name : node_names) {
      auto match_it = std::find_if(
        defaults.begin(),
        defaults.end(),
        [&name](const RmpNodeConfig & node) {return node.name == name;});
      const RmpNodeConfig fallback =
        match_it != defaults.end() ? *match_it : make_rmp_node_config(name, "root", "identity", "none", true);
      const std::string prefix = "graph." + name + ".";
      declare_parameter(prefix + "parent", fallback.parents.front());
      declare_parameter(prefix + "parents", fallback.parents);
      declare_parameter(prefix + "task_map", fallback.task_map_type);
      declare_parameter(prefix + "leaf", fallback.leaf_rmp_type);
      declare_parameter(prefix + "enabled", fallback.enabled);
      declare_parameter(prefix + "target_key", fallback.target_key);
      declare_parameter(prefix + "link_name", fallback.link_name);
      declare_parameter(prefix + "axis", fallback.axis);
      declare_parameter(prefix + "handcrafted_leaf", fallback.handcrafted_leaf_rmp_type);
      declare_parameter(prefix + "parent_weights", fallback.parent_weights);
      declare_parameter(prefix + "bias", fallback.bias);
      declare_parameter(prefix + "matrix", fallback.matrix);
      declare_parameter(prefix + "slice_start", fallback.slice_start);
      declare_parameter(prefix + "slice_length", fallback.slice_length);
      declare_parameter(prefix + "scale", fallback.scale);
      declare_parameter(prefix + "identity_multiplier", fallback.identity_multiplier);
      declare_parameter(prefix + "epsilon", fallback.epsilon);
    }
  }

  std::vector<RmpNodeConfig> parse_graph_nodes() const
  {
    const auto node_names = get_parameter("graph.node_names").as_string_array();
    std::vector<RmpNodeConfig> nodes;
    nodes.reserve(node_names.size());

    for (const auto & name : node_names) {
      const std::string prefix = "graph." + name + ".";
      auto parents = get_parameter(prefix + "parents").as_string_array();
      if (parents.empty()) {
        parents.push_back(get_parameter(prefix + "parent").as_string());
      }
      nodes.push_back(make_rmp_node_config(
        name,
        parents,
        get_parameter(prefix + "task_map").as_string(),
        get_parameter(prefix + "leaf").as_string(),
        get_parameter(prefix + "enabled").as_bool(),
        get_parameter(prefix + "target_key").as_string(),
        get_parameter(prefix + "link_name").as_string(),
        get_parameter(prefix + "axis").as_string(),
        get_parameter(prefix + "handcrafted_leaf").as_string(),
        get_parameter(prefix + "parent_weights").as_double_array(),
        get_parameter(prefix + "bias").as_double_array(),
        get_parameter(prefix + "matrix").as_double_array(),
        static_cast<int>(get_parameter(prefix + "slice_start").as_int()),
        static_cast<int>(get_parameter(prefix + "slice_length").as_int()),
        get_parameter(prefix + "scale").as_double(),
        get_parameter(prefix + "identity_multiplier").as_double(),
        get_parameter(prefix + "epsilon").as_double()));
    }

    return nodes;
  }

  std::vector<BodyObstacle> parse_body_obstacles() const
  {
    std::vector<std::string> names;
    get_parameter_or("body_obstacles.names", names, std::vector<std::string>{});
    std::vector<BodyObstacle> obstacles;
    obstacles.reserve(names.size());
    for (const auto & name : names) {
      const std::string prefix = "body_obstacles." + name + ".";
      BodyObstacle obstacle;
      obstacle.type = get_parameter(prefix + "type").as_string();
      obstacle.link_name = get_parameter(prefix + "link_name").as_string();
      obstacle.mins = parse_vector3_parameter(prefix + "mins", Eigen::Vector3d::Zero());
      obstacle.maxs = parse_vector3_parameter(prefix + "maxs", Eigen::Vector3d::Zero());
      obstacle.center = parse_vector3_parameter(prefix + "center", Eigen::Vector3d::Zero());
      obstacle.radius = get_parameter(prefix + "radius").as_double();
      obstacles.push_back(obstacle);
    }
    return obstacles;
  }

  int control_point_count() const
  {
    return static_cast<int>(RB10Model::sensor_control_points.size());
  }

  std::vector<std::size_t> build_topological_order(
    const std::vector<RmpNodeConfig> & nodes) const
  {
    std::unordered_map<std::string, std::size_t> enabled_nodes;
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      if (!nodes[index].enabled) {
        continue;
      }
      enabled_nodes.emplace(nodes[index].name, index);
    }

    std::unordered_map<std::string, int> indegree;
    std::unordered_map<std::string, std::vector<std::string>> outgoing;
    for (const auto & entry : enabled_nodes) {
      indegree.emplace(entry.first, 0);
    }

    for (const auto & entry : enabled_nodes) {
      const auto & node = nodes[entry.second];
      for (const auto & parent : node.parents) {
        if (parent == "root") {
          continue;
        }
        if (!enabled_nodes.count(parent)) {
          throw std::runtime_error(
                  "Graph node " + node.name + " references missing parent " + parent);
        }
        ++indegree[node.name];
        outgoing[parent].push_back(node.name);
      }
    }

    std::vector<std::string> ready;
    for (const auto & entry : indegree) {
      if (entry.second == 0) {
        ready.push_back(entry.first);
      }
    }
    std::sort(ready.begin(), ready.end());

    std::vector<std::size_t> order;
    while (!ready.empty()) {
      const auto name = ready.front();
      ready.erase(ready.begin());
      order.push_back(enabled_nodes.at(name));
      for (const auto & child : outgoing[name]) {
        auto & child_indegree = indegree.at(child);
        --child_indegree;
        if (child_indegree == 0) {
          ready.push_back(child);
        }
      }
      std::sort(ready.begin(), ready.end());
    }

    if (order.size() != enabled_nodes.size()) {
      throw std::runtime_error("Cycle detected in graph configuration");
    }
    return order;
  }

  std::unordered_map<std::string, int> infer_node_dims(
    const std::vector<RmpNodeConfig> & nodes) const
  {
    std::unordered_map<std::string, int> dims;
    dims.emplace("root", 6);
    for (const auto index : build_topological_order(nodes)) {
      const auto & node = nodes[index];
      if (node.task_map_type == "tcp_position" ||
        node.task_map_type == "link_position" ||
        node.task_map_type == "link_orientation_axis")
      {
        dims[node.name] = 3;
      } else if (node.task_map_type == "joint_limit") {
        dims[node.name] = 12;
      } else if (node.task_map_type == "control_points") {
        dims[node.name] = 3 * control_point_count();
      } else if (node.task_map_type == "collision_distance") {
        dims[node.name] = control_point_count();
      } else if (node.task_map_type == "norm") {
        dims[node.name] = 1;
      } else if (node.task_map_type == "affine") {
        if (!node.bias.empty()) {
          dims[node.name] = static_cast<int>(node.bias.size());
        } else if (!node.matrix.empty()) {
          int input_dim = 0;
          for (const auto & parent : node.parents) {
            input_dim += dims.at(parent);
          }
          if (input_dim == 0 || static_cast<int>(node.matrix.size()) % input_dim != 0) {
            throw std::runtime_error("Invalid affine matrix size for node " + node.name);
          }
          dims[node.name] = static_cast<int>(node.matrix.size()) / input_dim;
        } else {
          dims[node.name] = dims.at(node.parents.front());
        }
      } else if (node.task_map_type == "concat") {
        int dim = 0;
        for (const auto & parent : node.parents) {
          dim += dims.at(parent);
        }
        dims[node.name] = dim;
      } else if (
        node.task_map_type == "elem_multiply" ||
        node.task_map_type == "elem_divide" ||
        node.task_map_type == "sin" ||
        node.task_map_type == "cos" ||
        node.task_map_type == "tanh" ||
        node.task_map_type == "square" ||
        node.task_map_type == "abs")
      {
        dims[node.name] = dims.at(node.parents.front());
      } else if (node.task_map_type == "slice") {
        dims[node.name] = node.slice_length > 0 ? node.slice_length :
          dims.at(node.parents.front()) - node.slice_start;
      } else {
        dims[node.name] = dims.at(node.parents.front());
      }
    }
    return dims;
  }

  void declare_external_rmp_feature_parameters(const std::vector<RmpNodeConfig> & nodes)
  {
    std::vector<std::string> feature_keys;
    for (const auto & node : nodes) {
      if (node.leaf_rmp_type == "external" || node.handcrafted_leaf_rmp_type == "external") {
        feature_keys.push_back(node.target_key);
      }
    }
    std::sort(feature_keys.begin(), feature_keys.end());
    feature_keys.erase(std::unique(feature_keys.begin(), feature_keys.end()), feature_keys.end());
    for (const auto & key : feature_keys) {
      const std::string prefix = "external_rmp." + key + ".";
      declare_parameter(prefix + "enabled", true);
      declare_parameter(prefix + "topic_prefix", key);
    }
  }

  void configure_external_rmp_inputs(const std::vector<RmpNodeConfig> & nodes)
  {
    declare_external_rmp_feature_parameters(nodes);
    const auto dims = infer_node_dims(nodes);
    const std::string root_prefix = get_parameter("external_rmp.topic_prefix").as_string();
    for (const auto & node : nodes) {
      const bool uses_external =
        node.leaf_rmp_type == "external" || node.handcrafted_leaf_rmp_type == "external";
      if (!uses_external) {
        continue;
      }
      const std::string key = node.target_key;
      const std::string prefix = "external_rmp." + key + ".";
      if (!get_parameter(prefix + "enabled").as_bool()) {
        continue;
      }
      if (external_rmp_buffers_.count(key)) {
        continue;
      }
      const int dim = dims.at(node.name);
      external_rmp_buffers_.emplace(
        key,
        ExternalRmpBuffer{
          dim,
          Eigen::MatrixXd::Zero(dim, dim),
          Eigen::VectorXd::Zero(dim),
          false,
          false
        });
      const std::string topic_prefix =
        root_prefix + "/" + get_parameter(prefix + "topic_prefix").as_string();
      external_metric_subs_.push_back(create_subscription<std_msgs::msg::Float64MultiArray>(
          topic_prefix + "/metric_sqrt",
          10,
          [this, key, dim](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (static_cast<int>(msg->data.size()) != dim * dim) {
              RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "External RMP metric_sqrt size mismatch for %s: expected %d, got %zu",
                key.c_str(),
                dim * dim,
                msg->data.size());
              return;
            }
            Eigen::MatrixXd matrix(dim, dim);
            for (int row = 0; row < dim; ++row) {
              for (int col = 0; col < dim; ++col) {
                matrix(row, col) = msg->data[static_cast<std::size_t>(row * dim + col)];
              }
            }
            std::scoped_lock lock(external_rmp_mutex_);
            auto & buffer = external_rmp_buffers_.at(key);
            buffer.metric_sqrt = matrix;
            buffer.has_metric = true;
          }));
      external_accel_subs_.push_back(create_subscription<std_msgs::msg::Float64MultiArray>(
          topic_prefix + "/acceleration",
          10,
          [this, key, dim](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (static_cast<int>(msg->data.size()) != dim) {
              RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "External RMP acceleration size mismatch for %s: expected %d, got %zu",
                key.c_str(),
                dim,
                msg->data.size());
              return;
            }
            Eigen::VectorXd acceleration(dim);
            for (int index = 0; index < dim; ++index) {
              acceleration[index] = msg->data[static_cast<std::size_t>(index)];
            }
            std::scoped_lock lock(external_rmp_mutex_);
            auto & buffer = external_rmp_buffers_.at(key);
            buffer.acceleration = acceleration;
            buffer.has_acceleration = true;
          }));
      RCLCPP_INFO(
        get_logger(),
        "External RMP input enabled for key '%s' on topics %s/metric_sqrt and %s/acceleration",
        key.c_str(),
        topic_prefix.c_str(),
        topic_prefix.c_str());
    }
  }

  void declare_body_obstacle_parameters()
  {
    std::vector<std::string> names;
    get_parameter_or("body_obstacles.names", names, std::vector<std::string>{});
    for (const auto & name : names) {
      const std::string prefix = "body_obstacles." + name + ".";
      declare_parameter(prefix + "type", "ball");
      declare_parameter(prefix + "link_name", "");
      declare_parameter(prefix + "mins", std::vector<double>{0.0, 0.0, 0.0});
      declare_parameter(prefix + "maxs", std::vector<double>{0.0, 0.0, 0.0});
      declare_parameter(prefix + "center", std::vector<double>{0.0, 0.0, 0.0});
      declare_parameter(prefix + "radius", 0.0);
    }
  }

  Eigen::Vector3d parse_vector3_parameter(
    const std::string & name,
    const Eigen::Vector3d & fallback) const
  {
    const auto values = get_parameter(name).as_double_array();
    Eigen::Vector3d out = fallback;
    for (std::size_t index = 0; index < std::min<std::size_t>(values.size(), 3); ++index) {
      out[static_cast<int>(index)] = values[index];
    }
    return out;
  }

  std::size_t link_index_from_name(const std::string & link_name) const
  {
    for (std::size_t index = 0; index < RB10Model::link_names.size(); ++index) {
      if (link_name == RB10Model::link_names[index]) {
        return index;
      }
    }
    throw std::runtime_error("Unknown RB10 link name: " + link_name);
  }

  Eigen::Vector3d body_obstacle_center_world(
    const BodyObstacle & obstacle,
    const KinematicsContext & context) const
  {
    if (!obstacle.link_name.empty()) {
      const auto link_index = link_index_from_name(obstacle.link_name);
      const auto & rotation = context.link_rotations[link_index];
      const auto & origin = context.link_positions[link_index];
      if (obstacle.type == "box") {
        return origin + rotation * (0.5 * (obstacle.mins + obstacle.maxs));
      }
      return origin + rotation * obstacle.center;
    }

    if (obstacle.type == "box") {
      return 0.5 * (obstacle.mins + obstacle.maxs);
    }
    return obstacle.center;
  }

  double body_obstacle_min_z_world(
    const BodyObstacle & obstacle,
    const KinematicsContext & context) const
  {
    if (obstacle.type == "ball") {
      return body_obstacle_center_world(obstacle, context).z() - obstacle.radius;
    }

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    if (!obstacle.link_name.empty()) {
      const auto link_index = link_index_from_name(obstacle.link_name);
      rotation = context.link_rotations[link_index];
      origin = context.link_positions[link_index];
    }

    double min_z = std::numeric_limits<double>::infinity();
    for (int x_sel = 0; x_sel < 2; ++x_sel) {
      for (int y_sel = 0; y_sel < 2; ++y_sel) {
        for (int z_sel = 0; z_sel < 2; ++z_sel) {
          const Eigen::Vector3d local_corner(
            x_sel == 0 ? obstacle.mins.x() : obstacle.maxs.x(),
            y_sel == 0 ? obstacle.mins.y() : obstacle.maxs.y(),
            z_sel == 0 ? obstacle.mins.z() : obstacle.maxs.z());
          min_z = std::min(min_z, (origin + rotation * local_corner).z());
        }
      }
    }
    return min_z;
  }

  bool body_obstacle_interacts_with_sensor_control_point(
    std::size_t /*point_index*/,
    const BodyObstacle & obstacle) const
  {
    return obstacle.link_name != "link3" && obstacle.link_name != "link3_5";
  }

  struct FloorSafetyMetrics
  {
    double min_link_z{std::numeric_limits<double>::infinity()};
    double min_joint_z{std::numeric_limits<double>::infinity()};
    double min_control_point_z{std::numeric_limits<double>::infinity()};
    double min_body_obstacle_z{std::numeric_limits<double>::infinity()};
  };

  FloorSafetyMetrics compute_floor_safety_metrics(const KinematicsContext & context) const
  {
    FloorSafetyMetrics metrics;
    for (const auto & position : context.link_positions) {
      metrics.min_link_z = std::min(metrics.min_link_z, position.z());
    }
    for (const auto & origin : context.joint_origins) {
      metrics.min_joint_z = std::min(metrics.min_joint_z, origin.z());
    }
    for (const auto & control_point : context.control_points) {
      metrics.min_control_point_z = std::min(
        metrics.min_control_point_z,
        control_point.position.z() - control_point.radius);
    }
    for (const auto & obstacle : body_obstacles_visual_) {
      metrics.min_body_obstacle_z = std::min(
        metrics.min_body_obstacle_z,
        body_obstacle_min_z_world(obstacle, context));
    }
    return metrics;
  }

  void publish_debug_state(const KinematicsContext & context)
  {
    Eigen::Vector3d goal;
    std::vector<ObstacleSphere> obstacles;
    {
      std::scoped_lock goal_lock(goal_mutex_);
      goal = goal_;
    }
    {
      std::scoped_lock obstacle_lock(obstacles_mutex_);
      obstacles = obstacles_;
    }

    double min_external_clearance = std::numeric_limits<double>::infinity();
    for (const auto & control_point : context.control_points) {
      for (const auto & obstacle : obstacles) {
        const double clearance =
          (control_point.position - obstacle.center).norm() -
          (control_point.radius + obstacle.radius);
        min_external_clearance = std::min(min_external_clearance, clearance);
      }
    }

    double min_body_clearance = std::numeric_limits<double>::infinity();
    for (std::size_t point_index = 0; point_index < context.control_points.size(); ++point_index) {
      const auto & control_point = context.control_points[point_index];
      for (const auto & obstacle : body_obstacles_visual_) {
        if (!body_obstacle_interacts_with_sensor_control_point(point_index, obstacle)) {
          continue;
        }
        if (obstacle.type != "ball") {
          continue;
        }
        const auto center = body_obstacle_center_world(obstacle, context);
        const double clearance =
          (control_point.position - center).norm() -
          (control_point.radius + obstacle.radius);
        min_body_clearance = std::min(min_body_clearance, clearance);
      }
    }

    if (!std::isfinite(min_external_clearance)) {
      min_external_clearance = 1e6;
    }
    if (!std::isfinite(min_body_clearance)) {
      min_body_clearance = 1e6;
    }

    const auto floor_metrics = compute_floor_safety_metrics(context);

    std_msgs::msg::Float64MultiArray debug;
    debug.data = {
      (context.tcp_position - goal).norm(),
      context.tcp_position.z(),
      context.link_positions[RB10Model::LINK6].z(),
      min_external_clearance,
      min_body_clearance,
      state_.qd.norm(),
      last_min_z_safety_triggered_.load() ? 1.0 : 0.0,
      floor_metrics.min_link_z,
      floor_metrics.min_joint_z,
      floor_metrics.min_control_point_z,
      floor_metrics.min_body_obstacle_z,
    };
    debug_state_pub_->publish(debug);
  }

  JointVector compute_command(
    const RobotState & state,
    const Eigen::Vector3d & goal,
    const Eigen::Quaterniond & goal_orientation,
    const std::vector<ObstacleSphere> & obstacles) const
  {
    std::unordered_map<std::string, Eigen::Vector3d> vector_targets;
    vector_targets.emplace("goal", goal);
    vector_targets.emplace("body_goal", body_goal_);
    const Eigen::Matrix3d goal_rotation = goal_orientation.normalized().toRotationMatrix();
    vector_targets.emplace("orientation_goal_x", goal_rotation.col(0));
    vector_targets.emplace("orientation_goal_y", goal_rotation.col(1));
    vector_targets.emplace("orientation_goal_z", goal_rotation.col(2));
    std::unordered_map<std::string, ExternalRmpFeature> external_rmps;
    {
      std::scoped_lock lock(external_rmp_mutex_);
      for (const auto & entry : external_rmp_buffers_) {
        const auto & buffer = entry.second;
        if (!buffer.has_metric || !buffer.has_acceleration) {
          continue;
        }
        external_rmps.emplace(
          entry.first,
          ExternalRmpFeature{buffer.metric_sqrt, buffer.acceleration});
      }
    }
    const auto solution = solver_->solve(state.q, state.qd, vector_targets, obstacles, external_rmps);
    JointVector qdd = solution.qdd;
    const double max_joint_accel = get_parameter("max_joint_accel").as_double();
    for (int index = 0; index < qdd.size(); ++index) {
      qdd[index] = std::clamp(qdd[index], -max_joint_accel, max_joint_accel);
    }
    return qdd;
  }

  bool violates_min_z_safety(const RobotState & state) const
  {
    if (!get_parameter("safety_stop_on_min_z").as_bool()) {
      return false;
    }

    const auto context = RB10Model::forward_context(state.q);
    const auto floor_metrics = compute_floor_safety_metrics(context);
    const double workspace_floor_z = get_parameter("workspace_floor_z").as_double();
    const double min_link6_z = get_parameter("min_link6_z").as_double();
    const double min_tcp_z = get_parameter("min_tcp_z").as_double();
    return
      floor_metrics.min_link_z < workspace_floor_z ||
      floor_metrics.min_joint_z < workspace_floor_z ||
      floor_metrics.min_control_point_z < workspace_floor_z ||
      context.link_positions[RB10Model::LINK6].z() < min_link6_z ||
      context.link_positions[RB10Model::TCP_RMP].z() < min_tcp_z;
  }

  void control_loop()
  {
    enable_realtime(
      get_logger(),
      get_parameter("enable_realtime").as_bool(),
      get_parameter("realtime_priority").as_int(),
      get_parameter("lock_memory").as_bool());

    const double control_rate = get_parameter("control_rate").as_double();
    const auto period = std::chrono::duration<double>(1.0 / control_rate);
    auto next_tick = std::chrono::steady_clock::now();

    while (rclcpp::ok() && running_.load()) {
      next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

      RobotState state = backend_->read_state();
      Eigen::Vector3d goal;
      Eigen::Quaterniond goal_orientation;
      std::vector<ObstacleSphere> obstacles;
      {
        std::scoped_lock goal_lock(goal_mutex_);
        goal = goal_;
        goal_orientation = goal_orientation_;
      }
      {
        std::scoped_lock obstacle_lock(obstacles_mutex_);
        obstacles = obstacles_;
      }

      JointVector qdd = JointVector::Zero();
      last_min_z_safety_triggered_.store(false);
      try {
        qdd = compute_command(state, goal, goal_orientation, obstacles);
      } catch (const std::exception & error) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "RMP solve failed, holding command at zero acceleration: %s",
          error.what());
      }
      RobotState predicted_state = state;
      predicted_state.qd += qdd * period.count();
      predicted_state.q += predicted_state.qd * period.count();
      predicted_state.q = RB10Model::clamp_positions(predicted_state.q);
      if (violates_min_z_safety(state) || violates_min_z_safety(predicted_state)) {
        qdd.setZero();
        last_min_z_safety_triggered_.store(true);
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Min-Z safety triggered. Holding zero acceleration to avoid driving below workspace.");
      }
      backend_->apply_acceleration(qdd, period.count(), RB10Model::joint_names);
      state_ = backend_->read_state();
      publish_joint_states(state_);

      std::this_thread::sleep_until(next_tick);
    }
  }

  void publish_joint_states(const RobotState & state)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.assign(RB10Model::joint_names.begin(), RB10Model::joint_names.end());
    msg.position.assign(state.q.data(), state.q.data() + state.q.size());
    msg.velocity.assign(state.qd.data(), state.qd.data() + state.qd.size());
    joint_state_pub_->publish(msg);
  }

  void publish_visualization()
  {
    const auto state = backend_->read_state();
    const auto context = RB10Model::forward_context(state.q);
    publish_debug_state(context);
    Eigen::Vector3d goal;
    {
      std::scoped_lock lock(goal_mutex_);
      goal = goal_;
    }

    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = "base_link";
    goal_marker.header.stamp = now();
    goal_marker.ns = "goal";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.position.x = goal.x();
    goal_marker.pose.position.y = goal.y();
    goal_marker.pose.position.z = goal.z();
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.08;
    goal_marker.scale.y = 0.08;
    goal_marker.scale.z = 0.08;
    goal_marker.color.r = 0.0F;
    goal_marker.color.g = 1.0F;
    goal_marker.color.b = 0.0F;
    goal_marker.color.a = 0.8F;
    goal_marker_pub_->publish(goal_marker);

    visualization_msgs::msg::MarkerArray points;
    for (std::size_t index = 0; index < context.control_points.size(); ++index) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = now();
      marker.ns = "control_points";
      marker.id = static_cast<int>(index);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = context.control_points[index].position.x();
      marker.pose.position.y = context.control_points[index].position.y();
      marker.pose.position.z = context.control_points[index].position.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = context.control_points[index].radius * 2.0;
      marker.scale.y = context.control_points[index].radius * 2.0;
      marker.scale.z = context.control_points[index].radius * 2.0;
      marker.color.r = 0.0F;
      marker.color.g = 0.5F;
      marker.color.b = 1.0F;
      marker.color.a = 0.3F;
      points.markers.push_back(marker);
    }
    control_point_pub_->publish(points);

    visualization_msgs::msg::MarkerArray body_obstacles;
    for (std::size_t index = 0; index < body_obstacles_visual_.size(); ++index) {
      const auto & obstacle = body_obstacles_visual_[index];
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = now();
      marker.ns = "body_obstacles";
      marker.id = static_cast<int>(index);
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.color.r = 1.0F;
      marker.color.g = 0.6F;
      marker.color.b = 0.0F;
      marker.color.a = 0.22F;

      Eigen::Vector3d center = obstacle.center;
      Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
      if (!obstacle.link_name.empty()) {
        const auto link_index = link_index_from_name(obstacle.link_name);
        rotation = context.link_rotations[link_index];
        if (obstacle.type == "box") {
          center =
            context.link_positions[link_index] +
            rotation * (0.5 * (obstacle.mins + obstacle.maxs));
        } else {
          center =
            context.link_positions[link_index] +
            rotation * obstacle.center;
        }
      } else if (obstacle.type == "box") {
        center = 0.5 * (obstacle.mins + obstacle.maxs);
      }

      marker.pose.position.x = center.x();
      marker.pose.position.y = center.y();
      marker.pose.position.z = center.z();

      if (obstacle.type == "ball") {
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = obstacle.radius * 2.0;
        marker.scale.y = obstacle.radius * 2.0;
        marker.scale.z = obstacle.radius * 2.0;
      } else if (obstacle.type == "box") {
        marker.type = visualization_msgs::msg::Marker::CUBE;
        Eigen::Quaterniond q(rotation);
        q.normalize();
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        const Eigen::Vector3d size = obstacle.maxs - obstacle.mins;
        marker.scale.x = size.x();
        marker.scale.y = size.y();
        marker.scale.z = size.z();
      } else {
        continue;
      }

      body_obstacles.markers.push_back(marker);
    }
    body_obstacle_pub_->publish(body_obstacles);

    geometry_msgs::msg::PoseStamped eef_pose;
    const Eigen::Vector3d & eef = context.tcp_position;
    eef_pose.header.frame_id = "base_link";
    eef_pose.header.stamp = now();
    eef_pose.pose.position.x = eef.x();
    eef_pose.pose.position.y = eef.y();
    eef_pose.pose.position.z = eef.z();
    Eigen::Quaterniond tcp_orientation(context.link_rotations[RB10Model::TCP_RMP]);
    tcp_orientation.normalize();
    eef_pose.pose.orientation.x = tcp_orientation.x();
    eef_pose.pose.orientation.y = tcp_orientation.y();
    eef_pose.pose.orientation.z = tcp_orientation.z();
    eef_pose.pose.orientation.w = tcp_orientation.w();
    eef_pose_pub_->publish(eef_pose);
  }

  std::atomic<bool> running_;
  std::thread control_thread_;
  std::unique_ptr<ControllerBackend> backend_;
  std::unique_ptr<RmpSolverInterface> solver_;
  RobotState state_;
  Eigen::Vector3d goal_;
  Eigen::Vector3d body_goal_;
  Eigen::Quaterniond goal_orientation_{Eigen::Quaterniond::Identity()};
  std::vector<ObstacleSphere> obstacles_;
  std::vector<BodyObstacle> body_obstacles_visual_;
  std::atomic<bool> last_min_z_safety_triggered_{false};
  std::mutex goal_mutex_;
  std::mutex obstacles_mutex_;
  mutable std::mutex external_rmp_mutex_;
  std::unordered_map<std::string, ExternalRmpBuffer> external_rmp_buffers_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr control_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr body_obstacle_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr eef_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr> external_metric_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr> external_accel_subs_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
};

}  // namespace

}  // namespace rb10_rmpflow_rviz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rb10_rmpflow_rviz::RmpflowControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
