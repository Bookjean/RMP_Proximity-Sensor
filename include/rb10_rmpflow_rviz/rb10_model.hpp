#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace rb10_rmpflow_rviz
{

struct ControlPointSpec
{
  int start_link;
  int end_link;
  int interpolation_points;
  double radius;
};

struct SensorControlPointSpec
{
  const char * frame_name;
  std::size_t parent_link;
  Eigen::Vector3d offset;
  double radius;
};

struct ControlPoint
{
  Eigen::Vector3d position;
  double radius;
};

struct KinematicsContext
{
  std::array<Eigen::Vector3d, 12> link_positions;
  std::array<Eigen::Matrix3d, 12> link_rotations;
  std::array<Eigen::Matrix<double, 3, 6>, 12> link_jacobians;
  std::array<Eigen::Matrix<double, 3, 6>, 12> link_angular_jacobians;
  std::array<Eigen::Vector3d, 12> link_velocities;
  std::array<Eigen::Vector3d, 12> link_curvatures;
  std::array<Eigen::Vector3d, 12> link_angular_velocities;
  std::array<Eigen::Vector3d, 12> link_angular_curvatures;
  std::array<Eigen::Vector3d, 6> joint_origins;
  std::array<Eigen::Vector3d, 6> joint_axes;
  Eigen::Vector3d tcp_position{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 3, 6> tcp_jacobian{Eigen::Matrix<double, 3, 6>::Zero()};
  Eigen::Vector3d tcp_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tcp_curvature{Eigen::Vector3d::Zero()};
  std::vector<ControlPoint> control_points;
  std::vector<Eigen::Matrix<double, 3, 6>> control_point_jacobians;
  std::vector<Eigen::Vector3d> control_point_velocities;
  std::vector<Eigen::Vector3d> control_point_curvatures;
};

class RB10Model
{
public:
  using JointVector = Eigen::Matrix<double, 6, 1>;
  using Jacobian = Eigen::Matrix<double, 3, 6>;

  enum LinkIndex : std::size_t
  {
    BASE_LINK = 0,
    LINK0,
    LINK1,
    LINK2,
    LINK3,
    LINK3_5,
    LINK4,
    LINK5,
    LINK6,
    TCP,
    TCP_RMP,
    TCP_GRIPPER,
    LINK_COUNT
  };

  static constexpr std::array<const char *, LINK_COUNT> link_names{
    "base_link", "link0", "link1", "link2", "link3", "link3_5", "link4", "link5", "link6", "tcp",
    "tcp_rmp", "tcp_gripper"
  };

  static constexpr std::array<const char *, 6> joint_names{
    "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"
  };

  static constexpr std::array<double, 6> joint_lower_limits{
    -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159
  };

  static constexpr std::array<double, 6> joint_upper_limits{
    3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159
  };

  inline static const std::array<ControlPointSpec, 5> control_point_specs{{
    {LINK1, LINK3, 10, 0.12},
    {LINK3, LINK4, 8, 0.10},
    {LINK4, LINK5, 5, 0.08},
    {LINK5, LINK6, 5, 0.06},
    {LINK6, TCP, 5, 0.05},
  }};

  inline static const std::array<SensorControlPointSpec, 20> sensor_control_points{{
    {"tof6_1_L", LINK5, Eigen::Vector3d(-0.06, 0.0, 0.11715),  0.1},
    {"tof6_1_F", LINK5, Eigen::Vector3d(0.0, 0.0, 0.17715), 0.1},
    {"tof6_1_R", LINK5, Eigen::Vector3d(0.06, 0.0, 0.11715), 0.1},
    {"tof6_1_U", LINK5, Eigen::Vector3d(0.0, 0.0667, 0.11715), 0.1},
    {"tof_S", LINK3_5, Eigen::Vector3d(0.0645, 0.0, 0.405075),  0.1},
    {"tof_E", LINK3_5, Eigen::Vector3d(0.0, 0.0645, 0.405075),  0.1},
    {"tof_N", LINK3_5, Eigen::Vector3d(-0.0645, 0.0, 0.405075), 0.1},
    {"tof_W", LINK3_5, Eigen::Vector3d(0.0, -0.0645, 0.405075), 0.1},
    {"tof3_1_S", LINK3_5, Eigen::Vector3d(0.0645, 0.0, 0.205075),  0.1},
    {"tof3_1_W", LINK3_5, Eigen::Vector3d(0.0, -0.0645, 0.205075), 0.1},
    {"tof3_1_N", LINK3_5, Eigen::Vector3d(-0.0645, 0.0, 0.205075), 0.1},
    {"tof3_1_E", LINK3_5, Eigen::Vector3d(0.0, 0.0645, 0.205075),  0.1},
    {"tof2_1_E", LINK2, Eigen::Vector3d(0.0, -0.1085, 0.2262), 0.1},
    {"tof2_1_S", LINK2, Eigen::Vector3d(0.079, -0.1875, 0.2262), 0.1},
    {"tof2_1_W", LINK2, Eigen::Vector3d(0.0, -0.2665, 0.2262), 0.1},
    {"tof2_1_N", LINK2, Eigen::Vector3d(-0.079, -0.1875, 0.2262), 0.1},
    {"tof2_E", LINK2, Eigen::Vector3d(0.0, -0.1085, 0.4262), 0.1},
    {"tof2_S", LINK2, Eigen::Vector3d(0.079, -0.1875, 0.4262), 0.1},
    {"tof2_W", LINK2, Eigen::Vector3d(0.0, -0.2665, 0.4262), 0.1},
    {"tof2_N", LINK2, Eigen::Vector3d(-0.079, -0.1875, 0.4262), 0.1},
  }};

  static Eigen::Affine3d origin_transform(
    double x, double y, double z,
    double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
  {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translate(Eigen::Vector3d(x, y, z));
    transform.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    transform.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    transform.rotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    return transform;
  }

  static Jacobian point_jacobian(
    const Eigen::Vector3d & point,
    const std::array<Eigen::Vector3d, 6> & joint_origins,
    const std::array<Eigen::Vector3d, 6> & joint_axes,
    int active_joints)
  {
    Jacobian jacobian = Jacobian::Zero();
    for (int joint = 0; joint < active_joints; ++joint) {
      jacobian.col(joint) = joint_axes[static_cast<std::size_t>(joint)].cross(
        point - joint_origins[static_cast<std::size_t>(joint)]);
    }
    return jacobian;
  }

  static Eigen::Affine3d link_frame_from_q(
    const JointVector & q,
    std::size_t parent_link)
  {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform = transform * Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitZ());
    transform = transform * origin_transform(0.0, 0.0, 0.197);
    if (parent_link == LINK2) {
      return transform * Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitY());
    }

    transform = transform * Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitY());
    transform = transform * origin_transform(0.0, -0.1875, 0.6127);
    transform = transform * Eigen::AngleAxisd(q[2], Eigen::Vector3d::UnitY());
    const Eigen::Affine3d link3_5_transform = transform * origin_transform(0.0, 0.1484, 0.0);
    if (parent_link == LINK3_5) {
      return link3_5_transform;
    }

    transform = link3_5_transform * origin_transform(0.0, 0.0, 0.57015);
    transform = transform * Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitY());
    transform = transform * origin_transform(0.0, -0.11715, 0.0);
    transform = transform * Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitZ());
    if (parent_link == LINK5) {
      return transform;
    }

    transform = transform * origin_transform(0.0, 0.0, 0.11715);
    if (parent_link == LINK6) {
      return transform * Eigen::AngleAxisd(q[5], Eigen::Vector3d::UnitY());
    }

    throw std::runtime_error("Unsupported sensor parent link index");
  }

  static Eigen::Vector3d sensor_position_from_q(
    const JointVector & q,
    const SensorControlPointSpec & sensor)
  {
    return link_frame_from_q(q, sensor.parent_link) * sensor.offset;
  }

  static KinematicsContext forward_context(const JointVector & q)
  {
    KinematicsContext context;
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    for (auto & jacobian : context.link_jacobians) {
      jacobian.setZero();
    }

    context.link_positions[BASE_LINK] = transform.translation();
    context.link_positions[LINK0] = transform.translation();
    context.link_rotations[BASE_LINK] = transform.linear();
    context.link_rotations[LINK0] = transform.linear();

    context.joint_origins[0] = transform.translation();
    context.joint_axes[0] = transform.linear() * Eigen::Vector3d::UnitZ();
    transform = transform * Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitZ());
    context.link_positions[LINK1] = transform.translation();
    context.link_rotations[LINK1] = transform.linear();

    transform = transform * origin_transform(0.0, 0.0, 0.197);
    context.joint_origins[1] = transform.translation();
    context.joint_axes[1] = transform.linear() * Eigen::Vector3d::UnitY();
    context.link_positions[LINK2] = transform.translation();
    context.link_jacobians[LINK2] = point_jacobian(
      context.link_positions[LINK2], context.joint_origins, context.joint_axes, 1);
    transform = transform * Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitY());
    context.link_rotations[LINK2] = transform.linear();

    transform = transform * origin_transform(0.0, -0.1875, 0.6127);
    context.joint_origins[2] = transform.translation();
    context.joint_axes[2] = transform.linear() * Eigen::Vector3d::UnitY();
    context.link_positions[LINK3] = transform.translation();
    context.link_jacobians[LINK3] = point_jacobian(
      context.link_positions[LINK3], context.joint_origins, context.joint_axes, 2);
    transform = transform * Eigen::AngleAxisd(q[2], Eigen::Vector3d::UnitY());
    context.link_rotations[LINK3] = transform.linear();

    const Eigen::Affine3d link3_5_transform = transform * origin_transform(0.0, 0.1484, 0.0);
    context.link_positions[LINK3_5] = link3_5_transform.translation();
    context.link_jacobians[LINK3_5] = point_jacobian(
      context.link_positions[LINK3_5], context.joint_origins, context.joint_axes, 3);
    context.link_rotations[LINK3_5] = link3_5_transform.linear();

    transform = link3_5_transform * origin_transform(0.0, 0.0, 0.57015);
    context.joint_origins[3] = transform.translation();
    context.joint_axes[3] = transform.linear() * Eigen::Vector3d::UnitY();
    context.link_positions[LINK4] = transform.translation();
    context.link_jacobians[LINK4] = point_jacobian(
      context.link_positions[LINK4], context.joint_origins, context.joint_axes, 3);
    transform = transform * Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitY());
    context.link_rotations[LINK4] = transform.linear();

    transform = transform * origin_transform(0.0, -0.11715, 0.0);
    context.joint_origins[4] = transform.translation();
    context.joint_axes[4] = transform.linear() * Eigen::Vector3d::UnitZ();
    context.link_positions[LINK5] = transform.translation();
    context.link_jacobians[LINK5] = point_jacobian(
      context.link_positions[LINK5], context.joint_origins, context.joint_axes, 4);
    transform = transform * Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitZ());
    context.link_rotations[LINK5] = transform.linear();

    transform = transform * origin_transform(0.0, 0.0, 0.11715);
    context.joint_origins[5] = transform.translation();
    context.joint_axes[5] = transform.linear() * Eigen::Vector3d::UnitY();
    context.link_positions[LINK6] = transform.translation();
    context.link_jacobians[LINK6] = point_jacobian(
      context.link_positions[LINK6], context.joint_origins, context.joint_axes, 5);
    transform = transform * Eigen::AngleAxisd(q[5], Eigen::Vector3d::UnitY());
    context.link_rotations[LINK6] = transform.linear();

    const Eigen::Affine3d tcp_transform = transform * origin_transform(0.0, -0.1153, 0.0);
    const Eigen::Affine3d tcp_rmp_transform =
      tcp_transform * origin_transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Eigen::Affine3d tcp_gripper_transform =
      tcp_transform * origin_transform(0.0, -0.285398, 0.0, 0.0, 1.5707963268, 1.5707963268);
    context.tcp_position = tcp_rmp_transform.translation();
    context.tcp_jacobian = point_jacobian(
      context.tcp_position, context.joint_origins, context.joint_axes, 6);
    context.link_positions[TCP] = tcp_transform.translation();
    context.link_jacobians[TCP] = point_jacobian(
      context.link_positions[TCP], context.joint_origins, context.joint_axes, 6);
    context.link_rotations[TCP] = tcp_transform.linear();
    context.link_positions[TCP_RMP] = context.tcp_position;
    context.link_jacobians[TCP_RMP] = context.tcp_jacobian;
    context.link_rotations[TCP_RMP] = tcp_rmp_transform.linear();
    context.link_positions[TCP_GRIPPER] = tcp_gripper_transform.translation();
    context.link_jacobians[TCP_GRIPPER] = point_jacobian(
      context.link_positions[TCP_GRIPPER], context.joint_origins, context.joint_axes, 6);
    context.link_rotations[TCP_GRIPPER] = tcp_gripper_transform.linear();

    context.control_points.reserve(sensor_control_points.size());
    context.control_point_jacobians.reserve(sensor_control_points.size());
    for (const auto & sensor : sensor_control_points) {
      const Eigen::Vector3d position =
        context.link_positions[sensor.parent_link] +
        context.link_rotations[sensor.parent_link] * sensor.offset;
      context.control_points.push_back(ControlPoint{position, sensor.radius});
      context.control_point_jacobians.push_back(
        numerical_jacobian(q, [&sensor](const JointVector & sample_q) {
          return sensor_position_from_q(sample_q, sensor);
        }));
    }

    return context;
  }

  static std::array<Eigen::Vector3d, LINK_COUNT> link_positions(const JointVector & q)
  {
    return forward_context(q).link_positions;
  }

  static Eigen::Vector3d tcp_position(const JointVector & q)
  {
    return forward_context(q).tcp_position;
  }

  static std::vector<ControlPoint> control_points(const JointVector & q)
  {
    return forward_context(q).control_points;
  }

  static Jacobian numerical_jacobian(
    const JointVector & q,
    const std::function<Eigen::Vector3d(const JointVector &)> & fn,
    double epsilon = 1e-5)
  {
    Jacobian jacobian = Jacobian::Zero();
    const Eigen::Vector3d reference = fn(q);
    for (int column = 0; column < q.size(); ++column) {
      JointVector perturbed = q;
      perturbed[column] += epsilon;
      jacobian.col(column) = (fn(perturbed) - reference) / epsilon;
    }
    return jacobian;
  }

  static JointVector clamp_positions(const JointVector & q)
  {
    JointVector clamped = q;
    for (int index = 0; index < clamped.size(); ++index) {
      clamped[index] = std::clamp(clamped[index], joint_lower_limits[index], joint_upper_limits[index]);
    }
    return clamped;
  }
};

inline Eigen::Matrix<double, 6, 3> damped_pseudoinverse(
  const Eigen::Matrix<double, 3, 6> & jacobian,
  double damping = 1e-4)
{
  const Eigen::Matrix3d jj_t =
    jacobian * jacobian.transpose() + damping * Eigen::Matrix3d::Identity();
  return jacobian.transpose() * jj_t.inverse();
}

}  // namespace rb10_rmpflow_rviz
