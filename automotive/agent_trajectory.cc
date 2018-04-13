#include "drake/automotive/agent_trajectory.h"

namespace drake {
namespace automotive {

using multibody::SpatialVelocity;
using trajectories::PiecewisePolynomial;
using trajectories::PiecewiseQuaternionSlerp;

PoseVelocity::PoseVelocity()
    : PoseVelocity::PoseVelocity(
          Eigen::Quaternion<double>::Identity(),
          Eigen::Translation<double, 3>::Identity(),
          SpatialVelocity<double>(Vector6<double>::Zero())) {}

PoseVelocity::PoseVelocity(const Eigen::Quaternion<double>& rotation,
                           const Eigen::Translation<double, 3>& translation,
                           const SpatialVelocity<double>& velocity)
    : rotation_(rotation), translation_(translation), velocity_(velocity) {}

PoseVelocity AgentTrajectory::value(double time) const {
  const Eigen::Quaternion<double> rotation(rotation_.orientation(time));
  const Eigen::Translation<double, 3> translation(translation_.value(time));
  const Eigen::Vector3d translation_dot = translation_dot_.value(time);
  const Eigen::Vector3d rpy_dot = rotation_.angular_velocity(time);
  const SpatialVelocity<double> velocity(rpy_dot, translation_dot);
  return PoseVelocity{rotation, translation, velocity};
}

AgentTrajectory AgentTrajectory::Make(
    const std::vector<double>& times,
    const std::vector<Eigen::Isometry3d>& knots,
    const InterpolationType& interp_type) {
  DRAKE_THROW_UNLESS(times.size() == knots.size());
  std::vector<Eigen::Quaternion<double>> knots_rotation(times.size());
  std::vector<Eigen::MatrixXd> knots_translation(times.size());
  for (int i{0}; i < static_cast<int>(times.size()); ++i) {
    knots_rotation[i] = knots[i].rotation();
    knots_translation[i] = knots[i].translation();
  }
  const PiecewiseQuaternionSlerp<double> rotation(times, knots_rotation);
  PiecewisePolynomial<double> translation;
  switch (interp_type) {
    case InterpolationType::kZeroOrderHold:
      translation =
          PiecewisePolynomial<double>::ZeroOrderHold(times, knots_translation);
      break;
    case InterpolationType::kFirstOrderHold:
      translation =
          PiecewisePolynomial<double>::FirstOrderHold(times, knots_translation);
      break;
    case InterpolationType::kCubic:
      translation =
          PiecewisePolynomial<double>::Cubic(times, knots_translation);
      break;
    case InterpolationType::kPchip:
      translation =
          PiecewisePolynomial<double>::Pchip(times, knots_translation);
      break;
    default:
      throw std::logic_error("The provided interp_type is not supported.");
  }
  return AgentTrajectory{translation, rotation};
}

AgentTrajectory::AgentTrajectory(
    const PiecewisePolynomial<double>& translation,
    const PiecewiseQuaternionSlerp<double>& rotation)
    : translation_(translation),
      rotation_(rotation),
      translation_dot_(translation.derivative()) {}

}  // namespace automotive
}  // namespace drake
