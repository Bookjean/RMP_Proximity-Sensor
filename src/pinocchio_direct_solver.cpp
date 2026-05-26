#include "rb10_rmpflow_rviz/pinocchio_direct_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "rb10_rmpflow_rviz/casadi_task_graph.hpp"

namespace rb10_rmpflow_rviz
{

namespace
{

struct SolverImpl
{
  std::unordered_map<std::string, int> node_dims;
  std::unordered_map<std::string, CompiledCasadiTaskMap> compiled_task_maps;
  std::vector<std::size_t> topo_indices;
};

const SolverImpl & get_impl(const std::shared_ptr<const void> & state)
{
  return *std::static_pointer_cast<const SolverImpl>(state);
}

std::vector<std::size_t> build_topological_order(const EigenRmpConfig & config)
{
  std::unordered_map<std::string, std::size_t> enabled_nodes;
  for (std::size_t index = 0; index < config.graph_nodes.size(); ++index) {
    const auto & node = config.graph_nodes[index];
    if (!node.enabled) {
      continue;
    }
    const auto inserted = enabled_nodes.emplace(node.name, index);
    if (!inserted.second) {
      throw std::runtime_error("Duplicate graph node name: " + node.name);
    }
  }

  std::unordered_map<std::string, int> indegree;
  std::unordered_map<std::string, std::vector<std::string>> outgoing;
  for (const auto & entry : enabled_nodes) {
    indegree.emplace(entry.first, 0);
  }

  for (const auto & entry : enabled_nodes) {
    const auto & node = config.graph_nodes[entry.second];
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
  ready.reserve(enabled_nodes.size());
  for (const auto & entry : indegree) {
    if (entry.second == 0) {
      ready.push_back(entry.first);
    }
  }
  std::sort(ready.begin(), ready.end());

  std::vector<std::size_t> order;
  order.reserve(enabled_nodes.size());
  while (!ready.empty()) {
    const auto name = ready.front();
    ready.erase(ready.begin());
    order.push_back(enabled_nodes.at(name));
    auto out_it = outgoing.find(name);
    if (out_it == outgoing.end()) {
      continue;
    }
    for (const auto & child : out_it->second) {
      auto & child_indegree = indegree.at(child);
      --child_indegree;
      if (child_indegree == 0) {
        ready.push_back(child);
      }
    }
    std::sort(ready.begin(), ready.end());
  }

  if (order.size() != enabled_nodes.size()) {
    throw std::runtime_error("Cycle detected in RMP graph configuration");
  }
  return order;
}

Eigen::VectorXd to_eigen_vector(const RB10Model::JointVector & value)
{
  return Eigen::VectorXd(value);
}

casadi::DM to_dm(const Eigen::VectorXd & value)
{
  auto out = casadi::DM::zeros(value.size(), 1);
  auto & nonzeros = out.nonzeros();
  nonzeros.resize(static_cast<std::size_t>(value.size()));
  for (Eigen::Index index = 0; index < value.size(); ++index) {
    nonzeros[static_cast<std::size_t>(index)] = value[index];
  }
  return out;
}

Eigen::VectorXd dm_to_vector(const casadi::DM & value)
{
  const auto dense = casadi::DM::densify(value);
  const auto elements = dense.get_elements();
  Eigen::VectorXd out(static_cast<Eigen::Index>(elements.size()));
  for (std::size_t index = 0; index < elements.size(); ++index) {
    out[static_cast<Eigen::Index>(index)] = elements[index];
  }
  return out;
}

Eigen::MatrixXd dm_to_matrix(const casadi::DM & value)
{
  const auto dense = casadi::DM::densify(value);
  const auto elements = dense.get_elements();
  Eigen::MatrixXd out(dense.size1(), dense.size2());
  for (casadi_int col = 0; col < dense.size2(); ++col) {
    for (casadi_int row = 0; row < dense.size1(); ++row) {
      const std::size_t flat = static_cast<std::size_t>(col * dense.size1() + row);
      out(row, col) = elements[flat];
    }
  }
  return out;
}

int link_index_from_name(const std::string & link_name)
{
  for (std::size_t index = 0; index < RB10Model::link_names.size(); ++index) {
    if (link_name == RB10Model::link_names[index]) {
      return static_cast<int>(index);
    }
  }
  throw std::runtime_error("Unsupported RB10 link name: " + link_name);
}

bool body_obstacle_interacts_with_sensor_control_point(
  int,
  const BodyObstacle & obstacle)
{
  (void)obstacle;
  return false;
}

PinocchioDirectRmpSolver::JointVector resolve_root_direct(
  const PinocchioDirectRmpSolver::Matrix6 & metric,
  const PinocchioDirectRmpSolver::JointVector & force,
  double solve_offset)
{
  const double max_abs = std::max(metric.cwiseAbs().maxCoeff() * 0.01, 1.0);
  PinocchioDirectRmpSolver::Matrix6 scaled_metric = metric / max_abs;
  PinocchioDirectRmpSolver::JointVector scaled_force = force / max_abs;
  scaled_metric += solve_offset * PinocchioDirectRmpSolver::Matrix6::Identity();

  PinocchioDirectRmpSolver::JointVector qdd =
    scaled_metric.ldlt().solve(scaled_force).eval();
  if (!qdd.allFinite()) {
    qdd = scaled_metric.completeOrthogonalDecomposition().solve(scaled_force).eval();
  }
  if (!qdd.allFinite()) {
    qdd.setZero();
  }
  return qdd;
}

PinocchioDirectRmpSolver::JointVector resolve_root_rmp2(
  const PinocchioDirectRmpSolver::Matrix6 & metric,
  const PinocchioDirectRmpSolver::JointVector & force,
  double solve_offset)
{
  const double max_abs = std::max(metric.cwiseAbs().maxCoeff() * 0.01, 1.0);
  PinocchioDirectRmpSolver::Matrix6 scaled_metric = metric / max_abs;
  PinocchioDirectRmpSolver::JointVector scaled_force = force / max_abs;
  scaled_metric += solve_offset * PinocchioDirectRmpSolver::Matrix6::Identity();

  PinocchioDirectRmpSolver::JointVector qdd =
    scaled_metric.ldlt().solve(scaled_force).eval();
  if (!qdd.allFinite()) {
    qdd = scaled_metric.completeOrthogonalDecomposition().solve(scaled_force).eval();
  }
  if (!qdd.allFinite()) {
    qdd.setZero();
  }
  return qdd;
}

}  // namespace

PinocchioDirectRmpSolver::PinocchioDirectRmpSolver(
  EigenRmpConfig config,
  std::string urdf_path)
: config_(std::move(config)),
  model_(std::make_shared<PinocchioModel>(urdf_path))
{
  auto impl = std::make_shared<SolverImpl>();
  impl->topo_indices = build_topological_order(config_);
  impl->node_dims.emplace("root", 6);
  for (const auto index : impl->topo_indices) {
    const auto & node = config_.graph_nodes[index];
    impl->node_dims.emplace(node.name, infer_node_dim(node, impl->node_dims));
  }

  std::vector<RmpNodeConfig> casadi_nodes;
  for (const auto index : impl->topo_indices) {
    const auto & node = config_.graph_nodes[index];
    if (uses_casadi_task_map(node.task_map_type)) {
      casadi_nodes.push_back(node);
    }
  }
  impl->compiled_task_maps = CasadiTaskGraph(casadi_nodes).compile(impl->node_dims);
  compiled_state_ = impl;
}

RmpSolveResult PinocchioDirectRmpSolver::solve(
  const JointVector & q,
  const JointVector & qd,
  const std::unordered_map<std::string, Eigen::Vector3d> & vector_targets,
  const std::vector<ObstacleSphere> & obstacles,
  const std::unordered_map<std::string, ExternalRmpFeature> & external_rmps) const
{
  const auto context = model_->forward_context(q, qd);
  const auto & impl = get_impl(compiled_state_);
  const bool use_rmp2 = uses_rmp2_solve();
  const bool use_natural_rmp = uses_natural_rmp();
  if (!use_rmp2 && use_natural_rmp) {
    throw std::runtime_error("rmp_type=natural requires solve_method=rmp2");
  }
  Matrix6 metric = Matrix6::Zero();
  JointVector force = JointVector::Zero();

  std::unordered_map<std::string, NodeGeometry> cache;
  cache.emplace(
    "root",
    NodeGeometry{
      to_eigen_vector(q),
      Matrix6::Identity(),
      to_eigen_vector(qd),
      Eigen::VectorXd::Zero(6)
    });

  for (const auto index : impl.topo_indices) {
    const auto & node = config_.graph_nodes[index];

    const auto geometry = evaluate_node(node, q, context, obstacles, cache);
    cache[node.name] = geometry;

    accumulate_leaf_type(
      node.leaf_rmp_type, node, geometry, qd, cache, vector_targets, external_rmps, metric, force);
    accumulate_leaf_type(
      node.handcrafted_leaf_rmp_type,
      node,
      geometry,
      qd,
      cache,
      vector_targets,
      external_rmps,
      metric,
      force);
  }

  JointVector qdd = use_rmp2 ?
    resolve_root_rmp2(metric, force, config_.solve_offset) :
    resolve_root_direct(metric, force, config_.solve_offset);

  return RmpSolveResult{qdd, metric, force};
}

std::vector<RmpSolveResult> PinocchioDirectRmpSolver::solve_batch(
  const std::vector<RmpBatchInput> & batch_inputs) const
{
  std::vector<RmpSolveResult> results;
  results.reserve(batch_inputs.size());
  for (const auto & input : batch_inputs) {
    results.push_back(solve(
        input.q,
        input.qd,
        input.vector_targets,
        input.obstacles,
        input.external_rmps));
  }
  return results;
}

double PinocchioDirectRmpSolver::sigmoid(double value)
{
  return 1.0 / (1.0 + std::exp(-value));
}

Eigen::VectorXd PinocchioDirectRmpSolver::make_default_q(const EigenRmpConfig & config)
{
  Eigen::VectorXd out(6);
  for (int index = 0; index < 6; ++index) {
    out[index] = config.default_q[static_cast<std::size_t>(index)];
  }
  return out;
}

Eigen::VectorXd PinocchioDirectRmpSolver::velocity_of(
  const NodeGeometry & geometry,
  const JointVector &)
{
  return geometry.velocity;
}

int PinocchioDirectRmpSolver::control_point_count()
{
  return static_cast<int>(RB10Model::sensor_control_points.size());
}

bool PinocchioDirectRmpSolver::uses_casadi_task_map(const std::string & task_map_type)
{
  return task_map_type == "affine" ||
         task_map_type == "elem_multiply" ||
         task_map_type == "elem_divide" ||
         task_map_type == "sin" ||
         task_map_type == "cos" ||
         task_map_type == "tanh" ||
         task_map_type == "square" ||
         task_map_type == "abs" ||
         task_map_type == "sum" ||
         task_map_type == "weighted_sum" ||
         task_map_type == "difference" ||
         task_map_type == "concat" ||
         task_map_type == "slice" ||
         task_map_type == "norm" ||
         task_map_type == "normalize";
}

bool PinocchioDirectRmpSolver::uses_rmp2_solve() const
{
  if (config_.solve_method == "direct") {
    return false;
  }
  if (config_.solve_method == "rmp2") {
    return true;
  }
  throw std::runtime_error("Unsupported solve_method: " + config_.solve_method);
}

bool PinocchioDirectRmpSolver::uses_natural_rmp() const
{
  if (config_.rmp_type == "canonical") {
    return false;
  }
  if (config_.rmp_type == "natural") {
    return true;
  }
  throw std::runtime_error("Unsupported rmp_type: " + config_.rmp_type);
}

Eigen::Vector3d PinocchioDirectRmpSolver::axis_unit_vector(const std::string & axis_name)
{
  if (axis_name == "x") {
    return Eigen::Vector3d::UnitX();
  }
  if (axis_name == "y") {
    return Eigen::Vector3d::UnitY();
  }
  if (axis_name == "z") {
    return Eigen::Vector3d::UnitZ();
  }
  throw std::runtime_error("Unsupported axis name: " + axis_name);
}

Eigen::VectorXd PinocchioDirectRmpSolver::flatten_control_points(
  const KinematicsContext & context)
{
  Eigen::VectorXd out(3 * static_cast<Eigen::Index>(context.control_points.size()));
  for (std::size_t index = 0; index < context.control_points.size(); ++index) {
    out.segment<3>(static_cast<Eigen::Index>(3 * index)) = context.control_points[index].position;
  }
  return out;
}

Eigen::MatrixXd PinocchioDirectRmpSolver::stack_control_point_jacobians(
  const KinematicsContext & context)
{
  Eigen::MatrixXd out(3 * static_cast<Eigen::Index>(context.control_point_jacobians.size()), 6);
  for (std::size_t index = 0; index < context.control_point_jacobians.size(); ++index) {
    out.block<3, 6>(static_cast<Eigen::Index>(3 * index), 0) =
      context.control_point_jacobians[index];
  }
  return out;
}

int PinocchioDirectRmpSolver::infer_node_dim(
  const RmpNodeConfig & node,
  const std::unordered_map<std::string, int> & node_dims)
{
  if (node.task_map_type == "tcp_position" ||
    node.task_map_type == "link_position" ||
    node.task_map_type == "link_orientation_axis")
  {
    return 3;
  }
  if (node.task_map_type == "joint_limit") {
    return 12;
  }
  if (node.task_map_type == "control_points") {
    return 3 * control_point_count();
  }
  if (node.task_map_type == "collision_distance") {
    return control_point_count();
  }
  if (node.task_map_type == "norm") {
    return 1;
  }
  if (node.task_map_type == "affine") {
    if (!node.bias.empty()) {
      return static_cast<int>(node.bias.size());
    }
    if (!node.matrix.empty()) {
      int input_dim = 0;
      for (const auto & parent : node.parents) {
        input_dim += node_dims.at(parent);
      }
      if (input_dim <= 0 || static_cast<int>(node.matrix.size()) % input_dim != 0) {
        throw std::runtime_error("Invalid affine matrix size for node " + node.name);
      }
      return static_cast<int>(node.matrix.size()) / input_dim;
    }
    return node_dims.at(node.parents.front());
  }
  if (node.task_map_type == "concat") {
    int dim = 0;
    for (const auto & parent : node.parents) {
      dim += node_dims.at(parent);
    }
    return dim;
  }
  if (
    node.task_map_type == "elem_multiply" ||
    node.task_map_type == "elem_divide" ||
    node.task_map_type == "sin" ||
    node.task_map_type == "cos" ||
    node.task_map_type == "tanh" ||
    node.task_map_type == "square" ||
    node.task_map_type == "abs")
  {
    return node_dims.at(node.parents.front());
  }
  if (node.task_map_type == "slice") {
    return node.slice_length > 0 ? node.slice_length :
           node_dims.at(node.parents.front()) - node.slice_start;
  }
  return node_dims.at(node.parents.front());
}

bool PinocchioDirectRmpSolver::node_enabled(const std::string & name) const
{
  for (const auto & node : config_.graph_nodes) {
    if (node.name == name) {
      return node.enabled;
    }
  }
  return false;
}

PinocchioDirectRmpSolver::NodeGeometry PinocchioDirectRmpSolver::evaluate_node(
  const RmpNodeConfig & node,
  const JointVector & q,
  const KinematicsContext & context,
  const std::vector<ObstacleSphere> & obstacles,
  std::unordered_map<std::string, NodeGeometry> & cache) const
{
  std::vector<NodeGeometry> parents;
  parents.reserve(node.parents.size());
  for (const auto & parent_name : node.parents) {
    const auto it = cache.find(parent_name);
    if (it == cache.end()) {
      throw std::runtime_error("Missing parent node state for " + node.name + ": " + parent_name);
    }
    parents.push_back(it->second);
  }

  if (uses_casadi_task_map(node.task_map_type)) {
    return evaluate_casadi_node(node, parents);
  }
  return evaluate_native_node(node, q, context, obstacles, parents);
}

PinocchioDirectRmpSolver::NodeGeometry PinocchioDirectRmpSolver::evaluate_native_node(
  const RmpNodeConfig & node,
  const JointVector &,
  const KinematicsContext & context,
  const std::vector<ObstacleSphere> & obstacles,
  const std::vector<NodeGeometry> & parents) const
{
  if (node.task_map_type == "cspace_target" || node.task_map_type == "identity") {
    return parents.front();
  }

  if (node.task_map_type == "joint_limit") {
    NodeGeometry out;
    out.x.resize(12);
    out.jacobian = Eigen::MatrixXd::Zero(12, 6);
    out.velocity.resize(12);
    out.curvature = Eigen::VectorXd::Zero(12);
    const auto & lower_limits = model_->lower_limits();
    const auto & upper_limits = model_->upper_limits();
    for (int joint = 0; joint < 6; ++joint) {
      const double lower =
        lower_limits[static_cast<std::size_t>(joint)] +
        config_.joint_limit_buffers[static_cast<std::size_t>(joint)];
      const double upper =
        upper_limits[static_cast<std::size_t>(joint)] -
        config_.joint_limit_buffers[static_cast<std::size_t>(joint)];
      out.x[2 * joint] = upper - parents.front().x[joint];
      out.x[2 * joint + 1] = parents.front().x[joint] - lower;
      out.velocity[2 * joint] = -parents.front().velocity[joint];
      out.velocity[2 * joint + 1] = parents.front().velocity[joint];
      out.jacobian(2 * joint, joint) = -1.0;
      out.jacobian(2 * joint + 1, joint) = 1.0;
    }
    return out;
  }

  if (node.task_map_type == "tcp_position") {
    return NodeGeometry{
      context.tcp_position,
      context.tcp_jacobian,
      context.tcp_velocity,
      context.tcp_curvature
    };
  }

  if (node.task_map_type == "link_position") {
    const auto link_index = link_index_from_name(node.link_name);
    return NodeGeometry{
      context.link_positions[static_cast<std::size_t>(link_index)],
      context.link_jacobians[static_cast<std::size_t>(link_index)],
      context.link_velocities[static_cast<std::size_t>(link_index)],
      context.link_curvatures[static_cast<std::size_t>(link_index)]
    };
  }

  if (node.task_map_type == "link_orientation_axis") {
    const auto link_index = link_index_from_name(node.link_name);
    const Eigen::Vector3d axis_world =
      context.link_rotations[static_cast<std::size_t>(link_index)] * axis_unit_vector(node.axis);
    const Eigen::Vector3d angular_velocity =
      context.link_angular_velocities[static_cast<std::size_t>(link_index)];
    const Eigen::Vector3d angular_curvature =
      context.link_angular_curvatures[static_cast<std::size_t>(link_index)];
    const Eigen::Matrix<double, 3, 6> & angular_jacobian =
      context.link_angular_jacobians[static_cast<std::size_t>(link_index)];
    Eigen::Matrix<double, 3, 6> axis_jacobian;
    for (int column = 0; column < 6; ++column) {
      axis_jacobian.col(column) = angular_jacobian.col(column).cross(axis_world);
    }
    const Eigen::Vector3d axis_velocity = angular_velocity.cross(axis_world);
    const Eigen::Vector3d axis_curvature =
      angular_curvature.cross(axis_world) + angular_velocity.cross(axis_velocity);
    return NodeGeometry{axis_world, axis_jacobian, axis_velocity, axis_curvature};
  }

  if (node.task_map_type == "control_points") {
    return NodeGeometry{
      flatten_control_points(context),
      stack_control_point_jacobians(context),
      [&context]() {
        Eigen::VectorXd out(3 * static_cast<Eigen::Index>(context.control_point_velocities.size()));
        for (std::size_t index = 0; index < context.control_point_velocities.size(); ++index) {
          out.segment<3>(static_cast<Eigen::Index>(3 * index)) =
            context.control_point_velocities[index];
        }
        return out;
      }(),
      [&context]() {
        Eigen::VectorXd out(3 * static_cast<Eigen::Index>(context.control_point_curvatures.size()));
        for (std::size_t index = 0; index < context.control_point_curvatures.size(); ++index) {
          out.segment<3>(static_cast<Eigen::Index>(3 * index)) =
            context.control_point_curvatures[index];
        }
        return out;
      }()
    };
  }

  if (node.task_map_type == "collision_distance") {
    const auto & parent = parents.front();
    const int num_control_points = static_cast<int>(parent.x.size() / 3);
    if (num_control_points != control_point_count()) {
      throw std::runtime_error("collision_distance expects full RB10 control point vector");
    }

    NodeGeometry out;
    const int total_obstacles =
      static_cast<int>(obstacles.size() + config_.body_obstacles.size());
    if (total_obstacles == 0) {
      out.x = Eigen::VectorXd::Zero(0);
      out.jacobian = Eigen::MatrixXd::Zero(0, 6);
      out.velocity = Eigen::VectorXd::Zero(0);
      out.curvature = Eigen::VectorXd::Zero(0);
      return out;
    }

    std::vector<double> xs;
    std::vector<double> velocities;
    std::vector<double> curvatures;
    std::vector<Eigen::RowVectorXd> jacobians;
    xs.reserve(static_cast<std::size_t>(
      num_control_points * (static_cast<int>(obstacles.size()) + (config_.body_obstacles.empty() ? 0 : 1))));
    velocities.reserve(xs.capacity());
    curvatures.reserve(xs.capacity());
    jacobians.reserve(xs.capacity());

    const auto append_term =
      [&xs, &velocities, &curvatures, &jacobians](
        double x,
        const Eigen::RowVectorXd & jacobian,
        double velocity,
        double curvature)
      {
        xs.push_back(x);
        jacobians.push_back(jacobian);
        velocities.push_back(velocity);
        curvatures.push_back(curvature);
      };

    for (int point_index = 0; point_index < num_control_points; ++point_index) {
      const double point_radius =
        RB10Model::sensor_control_points[static_cast<std::size_t>(point_index)].radius;
      const auto position = parent.x.segment<3>(3 * point_index);
      const auto point_jacobian = parent.jacobian.block(3 * point_index, 0, 3, 6);
      const Eigen::Vector3d point_velocity = parent.velocity.segment<3>(3 * point_index);
      const Eigen::Vector3d point_curvature = parent.curvature.segment<3>(3 * point_index);
      for (const auto & obstacle : obstacles) {
        const Eigen::Vector3d delta = position - obstacle.center;
        const double center_distance = std::max(delta.norm(), 1e-9);
        const double signed_distance =
          center_distance - (point_radius + obstacle.radius) - config_.collision.margin;
        const double x = std::max(signed_distance, 0.0);
        const Eigen::RowVectorXd jacobian =
          (delta / center_distance).transpose() * point_jacobian;
        const double velocity = (delta / center_distance).dot(point_velocity);
        double curvature = 0.0;
        if (signed_distance > 0.0) {
          const Eigen::Matrix3d projector =
            (Eigen::Matrix3d::Identity() -
            (delta / center_distance) * (delta / center_distance).transpose()) /
            center_distance;
          curvature =
            (delta / center_distance).dot(point_curvature) +
            point_velocity.transpose() * projector * point_velocity;
        }
        append_term(
          x,
          jacobian,
          signed_distance > 0.0 ? velocity : 0.0,
          curvature);
      }

      bool has_best_body_term = false;
      double best_signed_distance = 0.0;
      double best_x = 0.0;
      Eigen::RowVectorXd best_jacobian = Eigen::RowVectorXd::Zero(6);
      double best_velocity = 0.0;
      double best_curvature = 0.0;

      for (const auto & obstacle : config_.body_obstacles) {
        if (!body_obstacle_interacts_with_sensor_control_point(point_index, obstacle)) {
          continue;
        }
        if (obstacle.type == "ball") {
          Eigen::Vector3d obstacle_center = obstacle.center;
          if (!obstacle.link_name.empty()) {
            const auto link_index =
              link_index_from_name(obstacle.link_name);
            obstacle_center =
              context.link_positions[static_cast<std::size_t>(link_index)] +
              context.link_rotations[static_cast<std::size_t>(link_index)] * obstacle.center;
          }
          const Eigen::Vector3d delta = position - obstacle_center;
          const double center_distance = std::max(delta.norm(), 1e-9);
          const double signed_distance =
            center_distance - (point_radius + obstacle.radius) - config_.collision.margin;
          const double x = std::max(signed_distance, 0.0);
          const Eigen::RowVectorXd jacobian =
            (delta / center_distance).transpose() * point_jacobian;
          const double velocity = (delta / center_distance).dot(point_velocity);
          double curvature = 0.0;
          if (signed_distance > 0.0) {
            const Eigen::Matrix3d projector =
              (Eigen::Matrix3d::Identity() -
              (delta / center_distance) * (delta / center_distance).transpose()) /
              center_distance;
            curvature =
              (delta / center_distance).dot(point_curvature) +
              point_velocity.transpose() * projector * point_velocity;
          }
          if (!has_best_body_term || signed_distance < best_signed_distance) {
            has_best_body_term = true;
            best_signed_distance = signed_distance;
            best_x = x;
            best_jacobian = jacobian;
            best_velocity = signed_distance > 0.0 ? velocity : 0.0;
            best_curvature = curvature;
          }
        } else if (obstacle.type == "box") {
          Eigen::Vector3d local_position = position;
          Eigen::Matrix3d obstacle_rotation = Eigen::Matrix3d::Identity();
          Eigen::Vector3d obstacle_origin = Eigen::Vector3d::Zero();
          if (!obstacle.link_name.empty()) {
            const auto link_index =
              link_index_from_name(obstacle.link_name);
            obstacle_rotation =
              context.link_rotations[static_cast<std::size_t>(link_index)];
            obstacle_origin =
              context.link_positions[static_cast<std::size_t>(link_index)];
            local_position =
              obstacle_rotation.transpose() * (position - obstacle_origin);
          }
          const Eigen::Vector3d clamped =
            local_position.cwiseMax(obstacle.mins).cwiseMin(obstacle.maxs);
          const Eigen::Vector3d delta = local_position - clamped;
          const double outside_distance = delta.norm();
          const double signed_distance =
            outside_distance - point_radius - config_.collision.margin;
          const double x = std::max(signed_distance, 0.0);
          Eigen::Vector3d grad_local = Eigen::Vector3d::Zero();
          if (outside_distance > 1e-9) {
            grad_local = delta / outside_distance;
          } else {
            const Eigen::Vector3d dist_to_min = local_position - obstacle.mins;
            const Eigen::Vector3d dist_to_max = obstacle.maxs - local_position;
            Eigen::Index axis = 0;
            double min_face = dist_to_min[0];
            for (Eigen::Index idx = 0; idx < 3; ++idx) {
              if (dist_to_min[idx] < min_face) {
                min_face = dist_to_min[idx];
                axis = idx;
              }
              if (dist_to_max[idx] < min_face) {
                min_face = dist_to_max[idx];
                axis = idx;
              }
            }
            grad_local[axis] = (dist_to_min[axis] < dist_to_max[axis]) ? -1.0 : 1.0;
          }
          const Eigen::Vector3d grad = obstacle_rotation * grad_local;
          const Eigen::RowVectorXd jacobian = grad.transpose() * point_jacobian;
          const double velocity = grad.dot(point_velocity);
          const double curvature = grad.dot(point_curvature);
          if (!has_best_body_term || signed_distance < best_signed_distance) {
            has_best_body_term = true;
            best_signed_distance = signed_distance;
            best_x = x;
            best_jacobian = jacobian;
            best_velocity = signed_distance > 0.0 ? velocity : 0.0;
            best_curvature = signed_distance > 0.0 ? curvature : 0.0;
          }
        } else {
          throw std::runtime_error("Unsupported body obstacle type: " + obstacle.type);
        }
      }

      if (has_best_body_term) {
        append_term(best_x, best_jacobian, best_velocity, best_curvature);
      }
    }

    out.x.resize(static_cast<Eigen::Index>(xs.size()));
    out.jacobian.resize(static_cast<Eigen::Index>(xs.size()), 6);
    out.velocity.resize(static_cast<Eigen::Index>(xs.size()));
    out.curvature.resize(static_cast<Eigen::Index>(xs.size()));
    for (std::size_t index = 0; index < xs.size(); ++index) {
      out.x[static_cast<Eigen::Index>(index)] = xs[index];
      out.jacobian.row(static_cast<Eigen::Index>(index)) = jacobians[index];
      out.velocity[static_cast<Eigen::Index>(index)] = velocities[index];
      out.curvature[static_cast<Eigen::Index>(index)] = curvatures[index];
    }
    return out;
  }

  throw std::runtime_error("Unsupported native task map for pinocchio_direct: " + node.task_map_type);
}

PinocchioDirectRmpSolver::NodeGeometry PinocchioDirectRmpSolver::evaluate_casadi_node(
  const RmpNodeConfig & node,
  const std::vector<NodeGeometry> & parents) const
{
  const auto & impl = get_impl(compiled_state_);
  const auto task_it = impl.compiled_task_maps.find(node.name);
  if (task_it == impl.compiled_task_maps.end()) {
    throw std::runtime_error("Missing compiled CasADi task map for node: " + node.name);
  }

  std::vector<casadi::DM> args;
  args.reserve(2 * parents.size());
  for (const auto & parent : parents) {
    args.push_back(to_dm(parent.x));
    args.push_back(to_dm(parent.velocity));
  }
  const auto outputs = task_it->second.function(args);

  NodeGeometry out;
  out.x = dm_to_vector(outputs.front());
  out.velocity = dm_to_vector(outputs[1]);
  out.curvature = dm_to_vector(outputs[2]);
  if (out.velocity.size() != out.x.size()) {
    throw std::runtime_error(
            "CasADi task map velocity dimension mismatch for node " + node.name);
  }
  if (out.curvature.size() != out.x.size()) {
    throw std::runtime_error(
            "CasADi task map curvature dimension mismatch for node " + node.name);
  }
  out.jacobian = Eigen::MatrixXd::Zero(out.x.size(), 6);
  for (std::size_t index = 0; index < parents.size(); ++index) {
    const auto local_jacobian = dm_to_matrix(outputs[index + 3]);
    const auto chained_jacobian = local_jacobian * parents[index].jacobian;
    const auto chained_curvature = local_jacobian * parents[index].curvature;
    if (chained_curvature.size() != out.curvature.size()) {
      throw std::runtime_error(
              "CasADi chained curvature dimension mismatch for node " + node.name);
    }
    out.jacobian += chained_jacobian;
    out.curvature += chained_curvature;
  }
  return out;
}

void PinocchioDirectRmpSolver::accumulate_scalar_leaf(
  bool use_natural_rmp,
  const RowVector6 & jacobian,
  double metric_scalar,
  double acceleration,
  double curvature,
  Matrix6 & metric,
  JointVector & force)
{
  metric += jacobian.transpose() * metric_scalar * jacobian;
  if (use_natural_rmp) {
    const double natural_force = metric_scalar * acceleration;
    force += jacobian.transpose() * (natural_force - metric_scalar * curvature);
  } else {
    force += jacobian.transpose() * (metric_scalar * (acceleration - curvature));
  }
}

void PinocchioDirectRmpSolver::accumulate_vector_leaf(
  bool use_natural_rmp,
  const Eigen::MatrixXd & jacobian,
  const Eigen::MatrixXd & leaf_metric,
  const Eigen::VectorXd & acceleration,
  const Eigen::VectorXd & curvature,
  Matrix6 & metric,
  JointVector & force)
{
  metric += jacobian.transpose() * leaf_metric * jacobian;
  if (use_natural_rmp) {
    const Eigen::VectorXd natural_force = leaf_metric * acceleration;
    force += jacobian.transpose() * (natural_force - leaf_metric * curvature);
  } else {
    force += jacobian.transpose() * leaf_metric * (acceleration - curvature);
  }
}

void PinocchioDirectRmpSolver::accumulate_external(
  const RmpNodeConfig & node,
  const NodeGeometry & geometry,
  const std::unordered_map<std::string, ExternalRmpFeature> & external_rmps,
  Matrix6 & metric,
  JointVector & force) const
{
  const auto feature_it = external_rmps.find(node.target_key);
  if (feature_it == external_rmps.end()) {
    return;
  }
  const auto & feature = feature_it->second;
  if (feature.acceleration.size() != geometry.x.size()) {
    throw std::runtime_error("External RMP acceleration dimension mismatch for node " + node.name);
  }
  if (
    feature.metric_sqrt.rows() != geometry.x.size() ||
    feature.metric_sqrt.cols() != geometry.x.size())
  {
    throw std::runtime_error("External RMP metric_sqrt dimension mismatch for node " + node.name);
  }
  Eigen::MatrixXd metric_sqrt = feature.metric_sqrt;
  if (node.identity_multiplier != 0.0) {
    metric_sqrt += node.identity_multiplier *
      Eigen::MatrixXd::Identity(metric_sqrt.rows(), metric_sqrt.cols());
  }
  const Eigen::MatrixXd leaf_metric = metric_sqrt * metric_sqrt.transpose();
  accumulate_vector_leaf(
    uses_rmp2_solve() && uses_natural_rmp(),
    geometry.jacobian,
    leaf_metric,
    feature.acceleration,
    geometry.curvature,
    metric,
    force);
}

void PinocchioDirectRmpSolver::accumulate_leaf_type(
  const std::string & leaf_type,
  const RmpNodeConfig & node,
  const NodeGeometry & geometry,
  const JointVector & qd,
  const std::unordered_map<std::string, NodeGeometry> & cache,
  const std::unordered_map<std::string, Eigen::Vector3d> & vector_targets,
  const std::unordered_map<std::string, ExternalRmpFeature> & external_rmps,
  Matrix6 & metric,
  JointVector & force) const
{
  if (leaf_type == "none" || leaf_type.empty()) {
    return;
  }

  if (leaf_type == "cspace_target") {
    accumulate_cspace_target(geometry, qd, metric, force);
    return;
  }
  if (leaf_type == "joint_limit") {
    accumulate_joint_limits(geometry, qd, metric, force);
    return;
  }
  if (leaf_type == "joint_velocity_cap") {
    accumulate_joint_velocity_cap(geometry, qd, metric, force);
    return;
  }
  if (leaf_type == "target") {
    const auto target_it = vector_targets.find(node.target_key);
    if (target_it != vector_targets.end()) {
      accumulate_target(geometry, qd, target_it->second, metric, force);
    }
    return;
  }
  if (leaf_type == "axis_target") {
    const auto target_it = vector_targets.find(node.target_key);
    if (target_it != vector_targets.end()) {
      Eigen::Vector3d current_position_value = Eigen::Vector3d::Zero();
      const Eigen::Vector3d * current_position = nullptr;
      const Eigen::Vector3d * position_goal = nullptr;
      if (node_enabled("target")) {
        const auto position_goal_it = vector_targets.find("goal");
        if (position_goal_it != vector_targets.end()) {
          position_goal = &position_goal_it->second;
        }
        const auto tcp_position_it = cache.find("tcp_position");
        if (
          tcp_position_it != cache.end() &&
          tcp_position_it->second.x.size() == 3)
        {
          current_position_value = tcp_position_it->second.x.head<3>();
          current_position = &current_position_value;
        }
      }
      accumulate_axis_target(
        geometry,
        qd,
        target_it->second,
        current_position,
        position_goal,
        metric,
        force);
    }
    return;
  }
  if (leaf_type == "wrist_axis_target") {
    const auto target_it = vector_targets.find(node.target_key);
    if (target_it != vector_targets.end()) {
      Eigen::Vector3d current_position_value = Eigen::Vector3d::Zero();
      const Eigen::Vector3d * current_position = nullptr;
      const Eigen::Vector3d * position_goal = nullptr;
      if (node_enabled("target")) {
        const auto position_goal_it = vector_targets.find("goal");
        if (position_goal_it != vector_targets.end()) {
          position_goal = &position_goal_it->second;
        }
        const auto tcp_position_it = cache.find("tcp_position");
        if (
          tcp_position_it != cache.end() &&
          tcp_position_it->second.x.size() == 3)
        {
          current_position_value = tcp_position_it->second.x.head<3>();
          current_position = &current_position_value;
        }
      }
      accumulate_wrist_axis_target(
        geometry,
        qd,
        target_it->second,
        current_position,
        position_goal,
        metric,
        force);
    }
    return;
  }
  if (leaf_type == "collision") {
    accumulate_collision(geometry, qd, metric, force);
    return;
  }
  if (leaf_type == "damping") {
    accumulate_joint_damping(geometry, qd, metric, force);
    return;
  }
  if (leaf_type == "external") {
    accumulate_external(node, geometry, external_rmps, metric, force);
    return;
  }
  throw std::runtime_error("Unsupported pinocchio_direct leaf RMP: " + leaf_type);
}

void PinocchioDirectRmpSolver::accumulate_cspace_target(
  const NodeGeometry & geometry,
  const JointVector & qd,
  Matrix6 & metric,
  JointVector & force) const
{
  const bool use_natural_rmp = uses_rmp2_solve() && uses_natural_rmp();
  const auto velocity = velocity_of(geometry, qd);
  Eigen::VectorXd delta = geometry.x - make_default_q(config_);

  const double norm = delta.norm();
  Eigen::VectorXd position_term = -config_.cspace_target.position_gain * delta;
  if (norm > config_.cspace_target.robust_position_term_thresh && norm > 1e-9) {
    position_term =
      -config_.cspace_target.robust_position_term_thresh *
      config_.cspace_target.position_gain *
      (delta / norm);
  }

  const Eigen::VectorXd acceleration = position_term - config_.cspace_target.damping_gain * velocity;
  const Eigen::MatrixXd leaf_metric =
    (config_.cspace_target.metric_scalar + config_.cspace_target.inertia) *
    Eigen::MatrixXd::Identity(geometry.x.size(), geometry.x.size());
  accumulate_vector_leaf(
    use_natural_rmp,
    geometry.jacobian,
    leaf_metric,
    acceleration,
    geometry.curvature,
    metric,
    force);
}

void PinocchioDirectRmpSolver::accumulate_joint_limits(
  const NodeGeometry & geometry,
  const JointVector & qd,
  Matrix6 & metric,
  JointVector & force) const
{
  const bool use_natural_rmp = uses_rmp2_solve() && uses_natural_rmp();
  const auto velocity = velocity_of(geometry, qd);
  for (Eigen::Index row = 0; row < geometry.x.size(); ++row) {
    const double x = std::max(geometry.x[row], 0.0);
    const double xd = velocity[row];
    const double metric_before_gate =
      config_.joint_limit.metric_scalar /
      (x / config_.joint_limit.metric_length_scale + config_.joint_limit.metric_exploder_eps);
    const double metric_scalar =
      (1.0 - sigmoid(xd / config_.joint_limit.metric_velocity_gate_length_scale)) *
      metric_before_gate;
    const double scaled_x =
      x / config_.joint_limit.accel_potential_exploder_length_scale;
    const double acceleration =
      config_.joint_limit.accel_potential_gain /
      (scaled_x * scaled_x + config_.joint_limit.accel_potential_exploder_eps) -
      config_.joint_limit.accel_damper_gain * xd;
    accumulate_scalar_leaf(
      use_natural_rmp,
      geometry.jacobian.row(row),
      metric_scalar,
      acceleration,
      geometry.curvature[row],
      metric,
      force);
  }
}

void PinocchioDirectRmpSolver::accumulate_joint_velocity_cap(
  const NodeGeometry & geometry,
  const JointVector & qd,
  Matrix6 & metric,
  JointVector & force) const
{
  const bool use_natural_rmp = uses_rmp2_solve() && uses_natural_rmp();
  const auto velocity = velocity_of(geometry, qd);
  for (Eigen::Index joint = 0; joint < velocity.size(); ++joint) {
    const double delta_velocity =
      std::abs(velocity[joint]) - config_.joint_velocity_cap.max_velocity +
      config_.joint_velocity_cap.velocity_damping_region;
    if (std::abs(velocity[joint]) < (
        config_.joint_velocity_cap.max_velocity -
        config_.joint_velocity_cap.velocity_damping_region))
    {
      continue;
    }

    const double xdd =
      -std::abs(config_.joint_velocity_cap.damping_gain * delta_velocity) *
      ((velocity[joint] >= 0.0) ? 1.0 : -1.0);
    const double clipped_relative_velocity = std::min(
      delta_velocity,
      config_.joint_velocity_cap.velocity_damping_region - config_.joint_velocity_cap.eps);
    const double velocity_ratio =
      clipped_relative_velocity / config_.joint_velocity_cap.velocity_damping_region;
    const double metric_scalar =
      config_.joint_velocity_cap.metric_weight /
      (1.0 - velocity_ratio * velocity_ratio);

    accumulate_scalar_leaf(
      use_natural_rmp,
      geometry.jacobian.row(joint),
      metric_scalar,
      xdd,
      geometry.curvature[joint],
      metric,
      force);
  }
}

void PinocchioDirectRmpSolver::accumulate_target(
  const NodeGeometry & geometry,
  const JointVector & qd,
  const Eigen::Vector3d & goal,
  Matrix6 & metric,
  JointVector & force) const
{
  const bool use_natural_rmp = uses_rmp2_solve() && uses_natural_rmp();
  if (geometry.x.size() != 3) {
    throw std::runtime_error("target leaf expects 3D task map output");
  }

  const auto velocity = velocity_of(geometry, qd);
  const Eigen::Vector3d x = geometry.x;
  const Eigen::Vector3d xd = velocity;
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

  accumulate_vector_leaf(
    use_natural_rmp,
    geometry.jacobian,
    leaf_metric,
    acceleration,
    geometry.curvature,
    metric,
    force);
}

void PinocchioDirectRmpSolver::accumulate_axis_target(
  const NodeGeometry & geometry,
  const JointVector & qd,
  const Eigen::Vector3d & goal,
  const Eigen::Vector3d * current_position,
  const Eigen::Vector3d * position_goal,
  Matrix6 & metric,
  JointVector & force) const
{
  const bool use_natural_rmp = uses_rmp2_solve() && uses_natural_rmp();
  if (geometry.x.size() != 3) {
    throw std::runtime_error("axis_target leaf expects 3D task map output");
  }

  const auto velocity = velocity_of(geometry, qd);
  const Eigen::Vector3d x = geometry.x.normalized();
  const Eigen::Vector3d xd = velocity;
  const Eigen::Vector3d goal_unit = goal.normalized();
  const Eigen::Vector3d delta = goal_unit - x;
  const Eigen::Vector3d acceleration =
    config_.axis_target.accel_p_gain * delta -
    config_.axis_target.accel_d_gain * xd;

  Eigen::Matrix3d leaf_metric =
    config_.axis_target.metric_scalar * Eigen::Matrix3d::Identity();
  if (current_position != nullptr && position_goal != nullptr) {
    const double delta_norm = (*position_goal - *current_position).norm();
    const double boost_length_scale =
      std::max(config_.axis_target.proximity_metric_boost_length_scale, 1e-9);
    const double boost_scaled_dist =
      delta_norm / boost_length_scale;
    const double boost_alpha = std::exp(-0.5 * boost_scaled_dist * boost_scaled_dist);
    const double metric_boost_scalar =
      boost_alpha * config_.axis_target.proximity_metric_boost_scalar + (1.0 - boost_alpha);
    leaf_metric *= metric_boost_scalar;
  }

  accumulate_vector_leaf(
    use_natural_rmp,
    geometry.jacobian,
    leaf_metric,
    acceleration,
    geometry.curvature,
    metric,
    force);
}

void PinocchioDirectRmpSolver::accumulate_wrist_axis_target(
  const NodeGeometry & geometry,
  const JointVector & qd,
  const Eigen::Vector3d & goal,
  const Eigen::Vector3d * current_position,
  const Eigen::Vector3d * position_goal,
  Matrix6 & metric,
  JointVector & force) const
{
  NodeGeometry wrist_geometry = geometry;
  if (wrist_geometry.jacobian.cols() != 6) {
    throw std::runtime_error("wrist_axis_target leaf expects a 6-column Jacobian");
  }
  // wrist_geometry.jacobian.leftCols(3).setZero();
  wrist_geometry.velocity = wrist_geometry.jacobian * qd;
  wrist_geometry.curvature = Eigen::VectorXd::Zero(wrist_geometry.x.size());

  const bool use_natural_rmp = uses_rmp2_solve() && uses_natural_rmp();
  if (wrist_geometry.x.size() != 3) {
    throw std::runtime_error("wrist_axis_target leaf expects 3D task map output");
  }

  const Eigen::Vector3d x = wrist_geometry.x.normalized();
  const Eigen::Vector3d xd = wrist_geometry.velocity;
  const Eigen::Vector3d goal_unit = goal.normalized();
  const Eigen::Vector3d delta = goal_unit - x;
  const Eigen::Vector3d acceleration =
    config_.wrist_axis_target.accel_p_gain * delta -
    config_.wrist_axis_target.accel_d_gain * xd;

  Eigen::Matrix3d leaf_metric =
    config_.wrist_axis_target.metric_scalar * Eigen::Matrix3d::Identity();
  if (current_position != nullptr && position_goal != nullptr) {
    const double delta_norm = (*position_goal - *current_position).norm();
    const double boost_length_scale =
      std::max(config_.wrist_axis_target.proximity_metric_boost_length_scale, 1e-9);
    const double boost_scaled_dist =
      delta_norm / boost_length_scale;
    const double boost_alpha = std::exp(-0.5 * boost_scaled_dist * boost_scaled_dist);
    const double metric_boost_scalar =
      boost_alpha * config_.wrist_axis_target.proximity_metric_boost_scalar +
      (1.0 - boost_alpha);
    leaf_metric *= metric_boost_scalar;
  }

  accumulate_vector_leaf(
    use_natural_rmp,
    wrist_geometry.jacobian,
    leaf_metric,
    acceleration,
    wrist_geometry.curvature,
    metric,
    force);
}

void PinocchioDirectRmpSolver::accumulate_collision(
  const NodeGeometry & geometry,
  const JointVector & qd,
  Matrix6 & metric,
  JointVector & force) const
{
  const bool use_natural_rmp = uses_rmp2_solve() && uses_natural_rmp();
  const auto velocity = velocity_of(geometry, qd);
  for (Eigen::Index row = 0; row < geometry.x.size(); ++row) {
    const double x = std::max(geometry.x[row], 0.0);
    const double xd = velocity[row];

    double metric_scalar =
      config_.collision.metric_scalar /
      (x / config_.collision.metric_exploder_std_dev + config_.collision.metric_exploder_eps);
    const double radius = config_.collision.metric_modulation_radius;
    double gate = x * x / (radius * radius) - 2.0 * x / radius + 1.0;
    if (x > radius) {
      gate = 0.0;
    }
    metric_scalar *= gate;

    const double repel =
      config_.collision.repulsion_gain * std::exp(-(x / config_.collision.repulsion_std_dev));
    const double sigma = sigmoid(xd / config_.collision.damping_velocity_gate_length_scale);
    const double damping =
      -(1.0 - sigma) * config_.collision.damping_gain * xd /
      (x / config_.collision.damping_std_dev + config_.collision.damping_robustness_eps);
    if (x > radius) {
      metric_scalar = 0.0;
    } else {
      metric_scalar *= (1.0 - sigma);
    }

    accumulate_scalar_leaf(
      use_natural_rmp,
      geometry.jacobian.row(row),
      metric_scalar,
      repel + damping,
      geometry.curvature[row],
      metric,
      force);
  }
}

void PinocchioDirectRmpSolver::accumulate_joint_damping(
  const NodeGeometry & geometry,
  const JointVector & qd,
  Matrix6 & metric,
  JointVector & force) const
{
  const bool use_natural_rmp = uses_rmp2_solve() && uses_natural_rmp();
  const auto velocity = velocity_of(geometry, qd);
  const double velocity_norm = velocity.norm();
  const double nonlinear_gain = config_.damping.accel_d_gain * velocity_norm;
  const Eigen::VectorXd acceleration = -nonlinear_gain * velocity;
  const double metric_scalar =
    config_.damping.metric_scalar * velocity_norm + config_.damping.inertia;
  const Eigen::MatrixXd leaf_metric =
    metric_scalar * Eigen::MatrixXd::Identity(velocity.size(), velocity.size());
  accumulate_vector_leaf(
    use_natural_rmp,
    geometry.jacobian,
    leaf_metric,
    acceleration,
    geometry.curvature,
    metric,
    force);
}

}  // namespace rb10_rmpflow_rviz
