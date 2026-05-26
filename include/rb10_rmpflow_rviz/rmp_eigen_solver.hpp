#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "rb10_rmpflow_rviz/rb10_model.hpp"

namespace rb10_rmpflow_rviz
{

struct ObstacleSphere
{
  Eigen::Vector3d center{100.0, 100.0, 100.0};
  double radius{0.01};
};

struct BodyObstacle
{
  std::string type{"ball"};
  std::string link_name{""};
  Eigen::Vector3d mins{Eigen::Vector3d::Zero()};
  Eigen::Vector3d maxs{Eigen::Vector3d::Zero()};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
};

struct ExternalRmpFeature
{
  Eigen::MatrixXd metric_sqrt;
  Eigen::VectorXd acceleration;
};

struct CSpaceTargetParams
{
  double metric_scalar{0.005};
  double position_gain{100.0};
  double damping_gain{50.0};
  double robust_position_term_thresh{0.5};
  double inertia{0.0001};
};

struct JointLimitParams
{
  double metric_scalar{0.1};
  double metric_length_scale{0.01};
  double metric_exploder_eps{0.001};
  double metric_velocity_gate_length_scale{0.01};
  double accel_damper_gain{200.0};
  double accel_potential_gain{1.0};
  double accel_potential_exploder_eps{0.01};
  double accel_potential_exploder_length_scale{0.1};
};

struct JointVelocityCapParams
{
  double max_velocity{1.7};
  double velocity_damping_region{0.15};
  double damping_gain{5.0};
  double metric_weight{0.05};
  double eps{1e-6};
};

struct TargetRmpParams
{
  double accel_p_gain{50.0};
  double accel_d_gain{70.0};
  double accel_norm_eps{0.075};
  double metric_alpha_length_scale{0.05};
  double min_metric_alpha{0.03};
  double max_metric_scalar{1.0};
  double min_metric_scalar{0.5};
  double proximity_metric_boost_scalar{3.0};
  double proximity_metric_boost_length_scale{0.02};
};

struct AxisTargetParams
{
  double accel_p_gain{1000.0};
  double accel_d_gain{500.0};
  double metric_scalar{50.0};
  double proximity_metric_boost_scalar{10.0};
  double proximity_metric_boost_length_scale{0.1};
};

struct CollisionRmpParams
{
  double margin{0.0};
  double damping_gain{50.0};
  double damping_std_dev{0.04};
  double damping_robustness_eps{0.01};
  double damping_velocity_gate_length_scale{0.01};
  double repulsion_gain{800.0};
  double repulsion_std_dev{0.01};
  double metric_modulation_radius{0.5};
  double metric_scalar{1.0};
  double metric_exploder_std_dev{0.02};
  double metric_exploder_eps{0.001};
};

struct DampingRmpParams
{
  double accel_d_gain{30.0};
  double metric_scalar{0.005};
  double inertia{0.3};
};

struct RmpNodeConfig
{
  std::string name;
  std::vector<std::string> parents{"root"};
  std::string task_map_type;
  std::string leaf_rmp_type{"none"};
  std::string target_key{"goal"};
  std::string link_name{"tcp"};
  std::string axis{"z"};
  std::string handcrafted_leaf_rmp_type{"none"};
  std::vector<double> parent_weights{};
  std::vector<double> bias{};
  std::vector<double> matrix{};
  int slice_start{0};
  int slice_length{0};
  double scale{1.0};
  double identity_multiplier{0.0};
  double epsilon{1e-9};
  bool enabled{true};
};

inline RmpNodeConfig make_rmp_node_config(
  const std::string & name,
  const std::string & parent,
  const std::string & task_map_type,
  const std::string & leaf_rmp_type,
  bool enabled,
  const std::string & target_key = "goal",
  const std::string & link_name = "tcp",
  const std::string & axis = "z",
  const std::string & handcrafted_leaf_rmp_type = "none",
  const std::vector<double> & parent_weights = {},
  const std::vector<double> & bias = {},
  const std::vector<double> & matrix = {},
  int slice_start = 0,
  int slice_length = 0,
  double scale = 1.0,
  double identity_multiplier = 0.0,
  double epsilon = 1e-9)
{
  RmpNodeConfig node;
  node.name = name;
  node.parents = {parent};
  node.task_map_type = task_map_type;
  node.leaf_rmp_type = leaf_rmp_type;
  node.target_key = target_key;
  node.link_name = link_name;
  node.axis = axis;
  node.handcrafted_leaf_rmp_type = handcrafted_leaf_rmp_type;
  node.parent_weights = parent_weights;
  node.bias = bias;
  node.matrix = matrix;
  node.slice_start = slice_start;
  node.slice_length = slice_length;
  node.scale = scale;
  node.identity_multiplier = identity_multiplier;
  node.epsilon = epsilon;
  node.enabled = enabled;
  return node;
}

inline RmpNodeConfig make_rmp_node_config(
  const std::string & name,
  const std::vector<std::string> & parents,
  const std::string & task_map_type,
  const std::string & leaf_rmp_type,
  bool enabled,
  const std::string & target_key = "goal",
  const std::string & link_name = "tcp",
  const std::string & axis = "z",
  const std::string & handcrafted_leaf_rmp_type = "none",
  const std::vector<double> & parent_weights = {},
  const std::vector<double> & bias = {},
  const std::vector<double> & matrix = {},
  int slice_start = 0,
  int slice_length = 0,
  double scale = 1.0,
  double identity_multiplier = 0.0,
  double epsilon = 1e-9)
{
  RmpNodeConfig node;
  node.name = name;
  node.parents = parents.empty() ? std::vector<std::string>{"root"} : parents;
  node.task_map_type = task_map_type;
  node.leaf_rmp_type = leaf_rmp_type;
  node.target_key = target_key;
  node.link_name = link_name;
  node.axis = axis;
  node.handcrafted_leaf_rmp_type = handcrafted_leaf_rmp_type;
  node.parent_weights = parent_weights;
  node.bias = bias;
  node.matrix = matrix;
  node.slice_start = slice_start;
  node.slice_length = slice_length;
  node.scale = scale;
  node.identity_multiplier = identity_multiplier;
  node.epsilon = epsilon;
  node.enabled = enabled;
  return node;
}

inline std::vector<RmpNodeConfig> default_rmp_graph_nodes()
{
  return {
    make_rmp_node_config("cspace_target", "root", "cspace_target", "cspace_target", true),
    make_rmp_node_config("joint_limits", "root", "joint_limit", "joint_limit", true),
    make_rmp_node_config("joint_velocity_cap", "root", "identity", "joint_velocity_cap", true),
    make_rmp_node_config("tcp_position", "root", "tcp_position", "none", true),
    make_rmp_node_config("target", "tcp_position", "identity", "target", true, "goal"),
    make_rmp_node_config("control_points", "root", "control_points", "none", true),
    make_rmp_node_config("collision", "control_points", "collision_distance", "collision", true),
    make_rmp_node_config("damping", "root", "identity", "damping", true),
    make_rmp_node_config("body_link4", "root", "link_position", "target", false, "body_goal", "link4"),
    make_rmp_node_config("tcp_orientation", "root", "link_orientation_axis", "target", false, "orientation_goal", "tcp", "z"),
  };
}

inline std::vector<std::string> default_rmp_graph_node_names()
{
  std::vector<std::string> names;
  for (const auto & node : default_rmp_graph_nodes()) {
    names.push_back(node.name);
  }
  return names;
}

struct EigenRmpConfig
{
  std::array<double, 6> default_q{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> joint_limit_buffers{0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
  double solve_offset{1e-3};
  std::string solve_method{"rmp2"};
  std::string rmp_type{"canonical"};
  CSpaceTargetParams cspace_target{};
  JointLimitParams joint_limit{};
  JointVelocityCapParams joint_velocity_cap{};
  TargetRmpParams target{};
  AxisTargetParams axis_target{};
  AxisTargetParams wrist_axis_target{};
  CollisionRmpParams collision{};
  DampingRmpParams damping{};
  std::vector<BodyObstacle> body_obstacles;
  std::vector<RmpNodeConfig> graph_nodes{default_rmp_graph_nodes()};
};

struct RmpSolveResult
{
  RB10Model::JointVector qdd{RB10Model::JointVector::Zero()};
  Eigen::Matrix<double, 6, 6> metric{Eigen::Matrix<double, 6, 6>::Zero()};
  RB10Model::JointVector force{RB10Model::JointVector::Zero()};
};

class EigenRmpSolver
{
public:
  using JointVector = RB10Model::JointVector;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  using RowVector6 = Eigen::Matrix<double, 1, 6>;

  explicit EigenRmpSolver(EigenRmpConfig config)
  : config_(std::move(config))
  {}

  RmpSolveResult solve(
    const JointVector & q,
    const JointVector & qd,
    const Eigen::Vector3d & goal,
    const std::vector<ObstacleSphere> & obstacles) const
  {
    const auto context = RB10Model::forward_context(q);
    Matrix6 metric = Matrix6::Zero();
    JointVector force = JointVector::Zero();

    accumulate_cspace_target(q, qd, metric, force);
    accumulate_joint_limits(q, qd, metric, force);
    accumulate_joint_velocity_cap(q, qd, metric, force);
    accumulate_target(context, qd, goal, metric, force);
    accumulate_collision(context, qd, obstacles, metric, force);
    accumulate_joint_damping(q, qd, metric, force);

    const double max_abs = std::max(metric.cwiseAbs().maxCoeff() * 0.01, 1.0);
    Matrix6 scaled_metric = metric / max_abs;
    JointVector scaled_force = force / max_abs;
    scaled_metric += config_.solve_offset * Matrix6::Identity();

    JointVector qdd = scaled_metric.ldlt().solve(scaled_force);
    if (!qdd.allFinite()) {
      qdd = scaled_metric.completeOrthogonalDecomposition().solve(scaled_force);
    }
    if (!qdd.allFinite()) {
      qdd.setZero();
    }

    return RmpSolveResult{qdd, metric, force};
  }

private:
  static double sigmoid(double value)
  {
    return 1.0 / (1.0 + std::exp(-value));
  }

  static void accumulate_scalar_leaf(
    const RowVector6 & jacobian,
    double metric_scalar,
    double acceleration,
    double curvature,
    Matrix6 & metric,
    JointVector & force)
  {
    metric += jacobian.transpose() * metric_scalar * jacobian;
    force += jacobian.transpose() * (metric_scalar * (acceleration - curvature));
  }

  static void accumulate_vector_leaf(
    const Eigen::MatrixXd & jacobian,
    const Eigen::MatrixXd & leaf_metric,
    const Eigen::VectorXd & acceleration,
    const Eigen::VectorXd & curvature,
    Matrix6 & metric,
    JointVector & force)
  {
    metric += jacobian.transpose() * leaf_metric * jacobian;
    force += jacobian.transpose() * leaf_metric * (acceleration - curvature);
  }

  void accumulate_cspace_target(
    const JointVector & q,
    const JointVector & qd,
    Matrix6 & metric,
    JointVector & force) const
  {
    JointVector delta = q;
    for (int index = 0; index < delta.size(); ++index) {
      delta[index] -= config_.default_q[static_cast<std::size_t>(index)];
    }

    const double norm = delta.norm();
    JointVector position_term = -config_.cspace_target.position_gain * delta;
    if (norm > config_.cspace_target.robust_position_term_thresh && norm > 1e-9) {
      position_term =
        -config_.cspace_target.robust_position_term_thresh *
        config_.cspace_target.position_gain *
        (delta / norm);
    }

    const JointVector acceleration =
      position_term - config_.cspace_target.damping_gain * qd;
    const Matrix6 leaf_metric =
      (config_.cspace_target.metric_scalar + config_.cspace_target.inertia) *
      Matrix6::Identity();
    accumulate_vector_leaf(
      Matrix6::Identity(),
      leaf_metric,
      acceleration,
      JointVector::Zero(),
      metric,
      force);
  }

  void accumulate_joint_limits(
    const JointVector & q,
    const JointVector & qd,
    Matrix6 & metric,
    JointVector & force) const
  {
    for (int joint = 0; joint < q.size(); ++joint) {
      const double lower =
        RB10Model::joint_lower_limits[static_cast<std::size_t>(joint)] +
        config_.joint_limit_buffers[static_cast<std::size_t>(joint)];
      const double upper =
        RB10Model::joint_upper_limits[static_cast<std::size_t>(joint)] -
        config_.joint_limit_buffers[static_cast<std::size_t>(joint)];

      const std::array<std::pair<double, double>, 2> leaves{{
        {upper - q[joint], -qd[joint]},
        {q[joint] - lower, qd[joint]},
      }};

      for (int sign_index = 0; sign_index < 2; ++sign_index) {
        const double x = std::max(leaves[static_cast<std::size_t>(sign_index)].first, 0.0);
        const double xd = leaves[static_cast<std::size_t>(sign_index)].second;

        const double metric_before_gate =
          config_.joint_limit.metric_scalar /
          (x / config_.joint_limit.metric_length_scale +
          config_.joint_limit.metric_exploder_eps);
        const double metric_scalar =
          (1.0 - sigmoid(xd / config_.joint_limit.metric_velocity_gate_length_scale)) *
          metric_before_gate;
        const double scaled_x =
          x / config_.joint_limit.accel_potential_exploder_length_scale;
        const double acceleration =
          config_.joint_limit.accel_potential_gain /
          (scaled_x * scaled_x + config_.joint_limit.accel_potential_exploder_eps) -
          config_.joint_limit.accel_damper_gain * xd;

        RowVector6 jacobian = RowVector6::Zero();
        jacobian[joint] = sign_index == 0 ? -1.0 : 1.0;
        accumulate_scalar_leaf(jacobian, metric_scalar, acceleration, 0.0, metric, force);
      }
    }
  }

  void accumulate_joint_velocity_cap(
    const JointVector &,
    const JointVector & qd,
    Matrix6 & metric,
    JointVector & force) const
  {
    for (int joint = 0; joint < qd.size(); ++joint) {
      const double delta_velocity =
        std::abs(qd[joint]) - config_.joint_velocity_cap.max_velocity +
        config_.joint_velocity_cap.velocity_damping_region;
      if (std::abs(qd[joint]) < (
          config_.joint_velocity_cap.max_velocity -
          config_.joint_velocity_cap.velocity_damping_region))
      {
        continue;
      }

      const double xdd =
        -std::abs(config_.joint_velocity_cap.damping_gain * delta_velocity) *
        ((qd[joint] >= 0.0) ? 1.0 : -1.0);
      const double clipped_relative_velocity = std::min(
        delta_velocity,
        config_.joint_velocity_cap.velocity_damping_region - config_.joint_velocity_cap.eps);
      const double velocity_ratio =
        clipped_relative_velocity / config_.joint_velocity_cap.velocity_damping_region;
      const double metric_scalar =
        config_.joint_velocity_cap.metric_weight /
        (1.0 - velocity_ratio * velocity_ratio);

      RowVector6 jacobian = RowVector6::Zero();
      jacobian[joint] = 1.0;
      accumulate_scalar_leaf(jacobian, metric_scalar, xdd, 0.0, metric, force);
    }
  }

  void accumulate_target(
    const KinematicsContext & context,
    const JointVector & qd,
    const Eigen::Vector3d & goal,
    Matrix6 & metric,
    JointVector & force) const
  {
    const Eigen::Vector3d x = context.tcp_position;
    const Eigen::Matrix<double, 3, 6> & jacobian = context.tcp_jacobian;
    const Eigen::Vector3d xd = jacobian * qd;
    const Eigen::Vector3d curvature = Eigen::Vector3d::Zero();

    const Eigen::Vector3d delta = goal - x;
    const double delta_norm = delta.norm();
    const double soft_delta_norm =
      std::max(delta_norm, config_.target.accel_norm_eps / 10.0);
    const Eigen::Vector3d delta_hat = delta / soft_delta_norm;

    const Eigen::Vector3d acceleration =
      config_.target.accel_p_gain * delta / (delta_norm + config_.target.accel_norm_eps) -
      config_.target.accel_d_gain * xd;

    const Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d shape = delta_hat * delta_hat.transpose();
    const double scaled_dist = delta_norm / config_.target.metric_alpha_length_scale;
    const double alpha =
      (1.0 - config_.target.min_metric_alpha) * std::exp(-0.5 * scaled_dist * scaled_dist) +
      config_.target.min_metric_alpha;
    Eigen::Matrix3d leaf_metric =
      alpha * config_.target.max_metric_scalar * eye +
      (1.0 - alpha) * config_.target.min_metric_scalar * shape;

    const double boost_scaled_dist =
      delta_norm / config_.target.proximity_metric_boost_length_scale;
    const double boost_alpha = std::exp(-0.5 * boost_scaled_dist * boost_scaled_dist);
    const double metric_boost_scalar =
      boost_alpha * config_.target.proximity_metric_boost_scalar + (1.0 - boost_alpha);
    leaf_metric *= metric_boost_scalar;

    accumulate_vector_leaf(jacobian, leaf_metric, acceleration, curvature, metric, force);
  }

  void accumulate_collision(
    const KinematicsContext & context,
    const JointVector & qd,
    const std::vector<ObstacleSphere> & obstacles,
    Matrix6 & metric,
    JointVector & force) const
  {
    if (obstacles.empty()) {
      return;
    }

    for (std::size_t cp_index = 0; cp_index < context.control_points.size(); ++cp_index) {
      const auto & control_point = context.control_points[cp_index];
      const auto & point_jacobian = context.control_point_jacobians[cp_index];
      for (const auto & obstacle : obstacles) {
        const Eigen::Vector3d delta = control_point.position - obstacle.center;
        const double center_distance = std::max(delta.norm(), 1e-9);
        const double x = std::max(
          center_distance - (control_point.radius + obstacle.radius) - config_.collision.margin,
          0.0);
        const Eigen::Vector3d delta_hat = delta / center_distance;
        const RowVector6 jacobian = delta_hat.transpose() * point_jacobian;
        const double xd = (jacobian * qd)[0];
        const double curvature = 0.0;

        double metric_scalar =
          config_.collision.metric_scalar /
          (x / config_.collision.metric_exploder_std_dev +
          config_.collision.metric_exploder_eps);
        const double radius = config_.collision.metric_modulation_radius;
        double gate = x * x / (radius * radius) - 2.0 * x / radius + 1.0;
        if (x > radius) {
          gate = 0.0;
        }
        metric_scalar *= gate;

        const double repel =
          config_.collision.repulsion_gain *
          std::exp(-(x / config_.collision.repulsion_std_dev));
        const double sigma =
          sigmoid(xd / config_.collision.damping_velocity_gate_length_scale);
        const double damping =
          -(1.0 - sigma) * config_.collision.damping_gain * xd /
          (x / config_.collision.damping_std_dev + config_.collision.damping_robustness_eps);
        if (x > radius) {
          metric_scalar = 0.0;
        } else {
          metric_scalar *= (1.0 - sigma);
        }

        accumulate_scalar_leaf(
          jacobian,
          metric_scalar,
          repel + damping,
          curvature,
          metric,
          force);
      }
    }
  }

  void accumulate_joint_damping(
    const JointVector &,
    const JointVector & qd,
    Matrix6 & metric,
    JointVector & force) const
  {
    const double velocity_norm = qd.norm();
    const double nonlinear_gain = config_.damping.accel_d_gain * velocity_norm;
    const JointVector acceleration = -nonlinear_gain * qd;
    const double metric_scalar =
      config_.damping.metric_scalar * velocity_norm + config_.damping.inertia;
    const Matrix6 leaf_metric = metric_scalar * Matrix6::Identity();
    accumulate_vector_leaf(
      Matrix6::Identity(),
      leaf_metric,
      acceleration,
      JointVector::Zero(),
      metric,
      force);
  }

  EigenRmpConfig config_;
};

}  // namespace rb10_rmpflow_rviz
