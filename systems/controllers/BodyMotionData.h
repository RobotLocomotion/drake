#ifndef SYSTEMS_CONTROLLERS_BODYMOTIONDATA_H_
#define SYSTEMS_CONTROLLERS_BODYMOTIONDATA_H_

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "PiecewisePolynomial.h"

class BodyMotionData
{
private:
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
  BodyMotionData(int body_or_frame_id, const std::vector<double>& segment_times);

  int findSegmentIndex(double t);

  int getBodyOrFrameId() const;

  bool isToeOffAllowed(int segment_index) const;

  bool isInFloatingBaseNullSpace(int segment_index) const;

  bool isPoseControlledWhenInContact(int segment_index) const;

  double getExponentialMapDampingRatioMultiplier() const;

  double getExponentialMapProportionalGainMultiplier() const;

  const PiecewisePolynomial<double>& getTrajectory() const;

  void setTrajectory(const PiecewisePolynomial<double>& trajectory);

  const Eigen::Isometry3d& getTransformTaskToWorld() const;

  const Eigen::Matrix<double, 6, 1>& getWeightMultiplier() const;

  const Eigen::Vector3d& getXyzDampingRatioMultiplier() const;

  const Eigen::Vector3d& getXyzProportionalGainMultiplier() const;
};

#endif /* SYSTEMS_CONTROLLERS_BODYMOTIONDATA_H_ */
