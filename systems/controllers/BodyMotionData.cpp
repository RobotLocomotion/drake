#include "BodyMotionData.h"

int BodyMotionData::findSegmentIndex(double t) const
{
  return trajectory.getSegmentIndex(t);
}

int BodyMotionData::getBodyOrFrameId() const
{
  return body_or_frame_id;
}

bool BodyMotionData::isToeOffAllowed(int segment_index) const {
  trajectory.segmentNumberRangeCheck(segment_index);
  return toe_off_allowed[segment_index];
}

bool BodyMotionData::isInFloatingBaseNullSpace(int segment_index) const {
  trajectory.segmentNumberRangeCheck(segment_index);
  return in_floating_base_nullspace[segment_index];
}

bool BodyMotionData::isPoseControlledWhenInContact(int segment_index) const {
  trajectory.segmentNumberRangeCheck(segment_index);
  return control_pose_when_in_contact[segment_index];
}

double BodyMotionData::getExponentialMapDampingRatioMultiplier() const
{
  return exponential_map_damping_ratio_multiplier;
}

double BodyMotionData::getExponentialMapProportionalGainMultiplier() const
{
  return exponential_map_proportional_gain_multiplier;
}

const PiecewisePolynomial<double>& BodyMotionData::getTrajectory() const
{
  return trajectory;
}

PiecewisePolynomial<double>& BodyMotionData::getTrajectory()
{
  return trajectory;
}

const Eigen::Isometry3d& BodyMotionData::getTransformTaskToWorld() const
{
  return transform_task_to_world;
}

const Eigen::Vector3d& BodyMotionData::getXyzDampingRatioMultiplier() const
{
  return xyz_damping_ratio_multiplier;
}

const Eigen::Matrix<double, 6, 1>& BodyMotionData::getWeightMultiplier() const
{
  return weight_multiplier;
}

const Eigen::Vector3d& BodyMotionData::getXyzProportionalGainMultiplier() const
{
  return xyz_proportional_gain_multiplier;
}
