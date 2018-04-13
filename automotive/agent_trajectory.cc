#include "drake/automotive/agent_trajectory.h"

namespace drake {
namespace automotive {

namespace {
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

PoseVelocity::PoseVelocity(const Eigen::VectorXd vector)
    : rotation_(Eigen::Quaternion<double>(
          vector(Indices::kRw), vector(Indices::kRx), vector(Indices::kRy),
          vector(Indices::kRz))),
      translation_(Eigen::Translation<double, 3>(
          vector(Indices::kTx), vector(Indices::kTy), vector(Indices::kTz))),
      velocity_(Eigen::Vector3d(vector(Indices::kWx), vector(Indices::kWy),
                                vector(Indices::kWz)),
                Eigen::Vector3d(vector(Indices::kVx), vector(Indices::kVy),
                                vector(Indices::kVz))) {
  DRAKE_DEMAND(vector.size() == kSize);
}

Eigen::VectorXd PoseVelocity::to_vector() const {
  Eigen::VectorXd vector(kSize);
  vector(Indices::kTx) = translation_.x();
  vector(Indices::kTy) = translation_.y();
  vector(Indices::kTz) = translation_.z();
  vector(Indices::kRw) = rotation_.w();
  vector(Indices::kRx) = rotation_.x();
  vector(Indices::kRy) = rotation_.y();
  vector(Indices::kRz) = rotation_.z();
  vector(Indices::kWx) = velocity_.rotational().x();
  vector(Indices::kWy) = velocity_.rotational().y();
  vector(Indices::kWz) = velocity_.rotational().z();
  vector(Indices::kVx) = velocity_.translational().x();
  vector(Indices::kVy) = velocity_.translational().y();
  vector(Indices::kVz) = velocity_.translational().z();
  return vector;
}

AgentTrajectory AgentTrajectory::Make(const std::vector<double>& times,
                                      const std::vector<PoseVelocity>& knots,
                                      const InterpolationType& interp_type) {
  DRAKE_THROW_UNLESS(times.size() == knots.size());
  std::vector<Eigen::MatrixXd> knots_vector(times.size());
  for (int i{0}; i < static_cast<int>(times.size()); ++i) {
    knots_vector[i] = knots[i].to_vector();
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
