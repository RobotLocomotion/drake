#include "drake/automotive/agent_trajectory.h"

namespace drake {
namespace automotive {

namespace {
// The number of raw data elements in PoseVelocity.
static constexpr int kSize = 13;
}  // namespace

using multibody::SpatialVelocity;
using trajectories::PiecewisePolynomial;

PoseVelocity::PoseVelocity()
    : PoseVelocity::PoseVelocity(
          Eigen::Quaternion<double>::Identity(),
          Eigen::Translation<double, 3>::Identity(),
          SpatialVelocity<double>(Vector6<double>::Zero())) {}

PoseVelocity::PoseVelocity(const Eigen::Quaternion<double>& rotation,
                           const Eigen::Translation<double, 3>& translation,
                           const SpatialVelocity<double>& velocity)
    : rotation_(rotation), translation_(translation), velocity_(velocity) {}

Eigen::VectorXd AgentTrajectory::to_vector(const PoseVelocity& pose_velocity) {
  Eigen::VectorXd vector(kSize);
  vector(Indices::kTx) = pose_velocity.translation().x();
  vector(Indices::kTy) = pose_velocity.translation().y();
  vector(Indices::kTz) = pose_velocity.translation().z();
  vector(Indices::kRw) = pose_velocity.rotation().w();
  vector(Indices::kRx) = pose_velocity.rotation().x();
  vector(Indices::kRy) = pose_velocity.rotation().y();
  vector(Indices::kRz) = pose_velocity.rotation().z();
  vector(Indices::kWx) = pose_velocity.velocity().rotational().x();
  vector(Indices::kWy) = pose_velocity.velocity().rotational().y();
  vector(Indices::kWz) = pose_velocity.velocity().rotational().z();
  vector(Indices::kVx) = pose_velocity.velocity().translational().x();
  vector(Indices::kVy) = pose_velocity.velocity().translational().y();
  vector(Indices::kVz) = pose_velocity.velocity().translational().z();
  return vector;
}

PoseVelocity AgentTrajectory::from_vector(const Eigen::VectorXd& vector) {
  DRAKE_DEMAND(vector.size() == kSize);
  Eigen::Quaternion<double> rotation(
      vector(Indices::kRw), vector(Indices::kRx), vector(Indices::kRy),
      vector(Indices::kRz));
  Eigen::Translation<double, 3> translation(
      vector(Indices::kTx), vector(Indices::kTy), vector(Indices::kTz));
  multibody::SpatialVelocity<double> velocity(
      Eigen::Vector3d(vector(Indices::kWx), vector(Indices::kWy),
                      vector(Indices::kWz)),
      Eigen::Vector3d(vector(Indices::kVx), vector(Indices::kVy),
                      vector(Indices::kVz)));
  return PoseVelocity{rotation, translation, velocity};
}

AgentTrajectory AgentTrajectory::Make(const std::vector<double>& times,
                                      const std::vector<PoseVelocity>& knots,
                                      const InterpolationType& interp_type) {
  DRAKE_THROW_UNLESS(times.size() == knots.size());
  std::vector<Eigen::MatrixXd> knots_vector(times.size());
  for (int i{0}; i < static_cast<int>(times.size()); ++i) {
    knots_vector[i] = AgentTrajectory::to_vector(knots[i]);
  }
  switch (interp_type) {
    case InterpolationType::kZeroOrderHold:
      return AgentTrajectory(
          PiecewisePolynomial<double>::ZeroOrderHold(times, knots_vector));
    case InterpolationType::kFirstOrderHold:
      return AgentTrajectory(
          PiecewisePolynomial<double>::FirstOrderHold(times, knots_vector));
    case InterpolationType::kCubic:
      return AgentTrajectory(
          PiecewisePolynomial<double>::Cubic(times, knots_vector));
    case InterpolationType::kPchip:
      return AgentTrajectory(
          PiecewisePolynomial<double>::Pchip(times, knots_vector));
    default:
      throw std::logic_error("The provided interp_type is not supported.");
  }
}

AgentTrajectory::AgentTrajectory(const PiecewisePolynomial<double>& trajectory)
    : trajectory_(trajectory) {}

}  // namespace automotive
}  // namespace drake
