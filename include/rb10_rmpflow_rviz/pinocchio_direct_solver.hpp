#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "rb10_rmpflow_rviz/pinocchio_model.hpp"
#include "rb10_rmpflow_rviz/rmp_solver_interface.hpp"

namespace rb10_rmpflow_rviz
{

class PinocchioDirectRmpSolver : public RmpSolverInterface
{
public:
  using JointVector = RB10Model::JointVector;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  using RowVector6 = Eigen::Matrix<double, 1, 6>;

  PinocchioDirectRmpSolver(EigenRmpConfig config, std::string urdf_path);

  RmpSolveResult solve(
    const JointVector & q,
    const JointVector & qd,
    const std::unordered_map<std::string, Eigen::Vector3d> & vector_targets,
    const std::vector<ObstacleSphere> & obstacles,
    const std::unordered_map<std::string, ExternalRmpFeature> & external_rmps = {}) const override;

  std::vector<RmpSolveResult> solve_batch(
    const std::vector<RmpBatchInput> & batch_inputs) const override;

private:
  struct NodeGeometry
  {
    Eigen::VectorXd x;
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd velocity;
    Eigen::VectorXd curvature;
  };

  static double sigmoid(double value);

  static void accumulate_scalar_leaf(
    bool use_natural_rmp,
    const RowVector6 & jacobian,
    double metric_scalar,
    double acceleration,
    double curvature,
    Matrix6 & metric,
    JointVector & force);

  static void accumulate_vector_leaf(
    bool use_natural_rmp,
    const Eigen::MatrixXd & jacobian,
    const Eigen::MatrixXd & leaf_metric,
    const Eigen::VectorXd & acceleration,
    const Eigen::VectorXd & curvature,
    Matrix6 & metric,
    JointVector & force);

  static Eigen::VectorXd make_default_q(const EigenRmpConfig & config);
  static Eigen::VectorXd velocity_of(const NodeGeometry & geometry, const JointVector & qd);
  static int control_point_count();
  static bool uses_casadi_task_map(const std::string & task_map_type);
  bool uses_rmp2_solve() const;
  bool uses_natural_rmp() const;
  static Eigen::Vector3d axis_unit_vector(const std::string & axis_name);
  static Eigen::VectorXd flatten_control_points(const KinematicsContext & context);
  static Eigen::MatrixXd stack_control_point_jacobians(const KinematicsContext & context);

  static int infer_node_dim(
    const RmpNodeConfig & node,
    const std::unordered_map<std::string, int> & node_dims);
  bool node_enabled(const std::string & name) const;
  NodeGeometry evaluate_node(
    const RmpNodeConfig & node,
    const JointVector & q,
    const KinematicsContext & context,
    const std::vector<ObstacleSphere> & obstacles,
    std::unordered_map<std::string, NodeGeometry> & cache) const;
  NodeGeometry evaluate_native_node(
    const RmpNodeConfig & node,
    const JointVector & q,
    const KinematicsContext & context,
    const std::vector<ObstacleSphere> & obstacles,
    const std::vector<NodeGeometry> & parents) const;
  NodeGeometry evaluate_casadi_node(
    const RmpNodeConfig & node,
    const std::vector<NodeGeometry> & parents) const;

  void accumulate_cspace_target(
    const NodeGeometry & geometry,
    const JointVector & qd,
    Matrix6 & metric,
    JointVector & force) const;

  void accumulate_joint_limits(
    const NodeGeometry & geometry,
    const JointVector & qd,
    Matrix6 & metric,
    JointVector & force) const;

  void accumulate_joint_velocity_cap(
    const NodeGeometry & geometry,
    const JointVector & qd,
    Matrix6 & metric,
    JointVector & force) const;

  void accumulate_target(
    const NodeGeometry & geometry,
    const JointVector & qd,
    const Eigen::Vector3d & goal,
    Matrix6 & metric,
    JointVector & force) const;

  void accumulate_orientation_target(
    const NodeGeometry & geometry,
    const JointVector & qd,
    const Eigen::Vector3d & goal,
    Matrix6 & metric,
    JointVector & force) const;

  void accumulate_collision(
    const NodeGeometry & geometry,
    const JointVector & qd,
    Matrix6 & metric,
    JointVector & force) const;

  void accumulate_joint_damping(
    const NodeGeometry & geometry,
    const JointVector & qd,
    Matrix6 & metric,
    JointVector & force) const;
  void accumulate_external(
    const RmpNodeConfig & node,
    const NodeGeometry & geometry,
    const std::unordered_map<std::string, ExternalRmpFeature> & external_rmps,
    Matrix6 & metric,
    JointVector & force) const;
  void accumulate_leaf_type(
    const std::string & leaf_type,
    const RmpNodeConfig & node,
    const NodeGeometry & geometry,
    const JointVector & qd,
    const std::unordered_map<std::string, Eigen::Vector3d> & vector_targets,
    const std::unordered_map<std::string, ExternalRmpFeature> & external_rmps,
    Matrix6 & metric,
    JointVector & force) const;

  EigenRmpConfig config_;
  std::shared_ptr<const PinocchioModel> model_;
  std::shared_ptr<const void> compiled_state_;
};

}  // namespace rb10_rmpflow_rviz
