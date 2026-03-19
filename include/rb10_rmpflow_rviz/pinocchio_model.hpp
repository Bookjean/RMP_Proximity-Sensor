#pragma once

#include <array>
#include <string>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "rb10_rmpflow_rviz/rb10_model.hpp"

namespace rb10_rmpflow_rviz
{

class PinocchioModel
{
public:
  explicit PinocchioModel(const std::string & urdf_path);

  KinematicsContext forward_context(
    const RB10Model::JointVector & q,
    const RB10Model::JointVector & qd) const;

  const std::array<double, 6> & lower_limits() const
  {
    return lower_limits_;
  }

  const std::array<double, 6> & upper_limits() const
  {
    return upper_limits_;
  }

private:
  pinocchio::Model model_;
  std::array<pinocchio::FrameIndex, RB10Model::LINK_COUNT> frame_ids_{};
  std::array<pinocchio::FrameIndex, 4> sensor_frame_ids_{};
  std::array<double, 6> lower_limits_{};
  std::array<double, 6> upper_limits_{};

public:
  const pinocchio::Model & model() const
  {
    return model_;
  }
};

}  // namespace rb10_rmpflow_rviz
