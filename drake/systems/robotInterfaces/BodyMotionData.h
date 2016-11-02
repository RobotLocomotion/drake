#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/common/drake_export.h"  // TODO(tkoolen): exports

class BodyMotionData {
 public:  // TODO(tkoolen): would be better to make this private
  int body_or_frame_id;
  PiecewisePolynomial<double> trajectory;
  std::vector<bool> toe_off_allowed;
  std::vector<bool> in_floating_base_nullspace;
  std::vector<bool> control_pose_when_in_contact;
  Eigen::Isometry3d transform_task_to_world;
  Eigen::Vector3d xyz_proportional_gain_multiplier;
  Eigen::Vector3d xyz_damping_ratio_multiplier;
  double exponential_map_proportional_gain_multiplier;
  double exponential_map_damping_ratio_multiplier;
  Eigen::Matrix<double, 6, 1> weight_multiplier;

 public:
  int findSegmentIndex(double t) const;

  int getBodyOrFrameId() const;

  bool isToeOffAllowed(int segment_index) const;

  bool isInFloatingBaseNullSpace(int segment_index) const;

  bool isPoseControlledWhenInContact(int segment_index) const;

  double getExponentialMapDampingRatioMultiplier() const;

  double getExponentialMapProportionalGainMultiplier() const;

  const PiecewisePolynomial<double>& getTrajectory() const;

  PiecewisePolynomial<double>& getTrajectory();

  const Eigen::Isometry3d& getTransformTaskToWorld() const;

  const Eigen::Matrix<double, 6, 1>& getWeightMultiplier() const;

  const Eigen::Vector3d& getXyzDampingRatioMultiplier() const;

  const Eigen::Vector3d& getXyzProportionalGainMultiplier() const;
};
