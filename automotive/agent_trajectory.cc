#include "drake/automotive/agent_trajectory.h"

#include <algorithm>

namespace drake {
namespace automotive {

namespace {

// Helper to construct a vector of Eigen::MatrixXd from a vector of
// Eigen::Vector3d.
std::vector<Eigen::MatrixXd> ToVectorOfMatrixXd(
    const std::vector<Eigen::Vector3d>& translations) {
  std::vector<Eigen::MatrixXd> result{};
  for (const auto& translation : translations) {
    result.emplace_back(translation);
  }
  return result;
}

}  // namespace

using multibody::SpatialVelocity;
using trajectories::PiecewisePolynomial;
using trajectories::PiecewiseQuaternionSlerp;

PoseVelocity::PoseVelocity()
    : PoseVelocity::PoseVelocity(
          Eigen::Quaternion<double>::Identity(), Eigen::Vector3d::Zero(),
          SpatialVelocity<double>(Vector6<double>::Zero())) {}

PoseVelocity::PoseVelocity(const Eigen::Quaternion<double>& rotation,
                           const Eigen::Vector3d& translation,
                           const SpatialVelocity<double>& velocity)
    : rotation_(rotation), translation_(translation), velocity_(velocity) {}

PoseVelocity AgentTrajectory::value(double time) const {
  const Eigen::Quaternion<double> rotation(rotation_.orientation(time));
  const Eigen::Vector3d translation(translation_.value(time));
  const Eigen::Vector3d translation_dot(translation_dot_.value(time));
  const Eigen::Vector3d rpy_dot(rotation_.angular_velocity(time));
  const SpatialVelocity<double> velocity(rpy_dot, translation_dot);
  return PoseVelocity{rotation, translation, velocity};
}

AgentTrajectory AgentTrajectory::Make(
    const std::vector<double>& times,
    const std::vector<Eigen::Quaternion<double>>& knots_rotation,
    const std::vector<Eigen::Vector3d>& knots_translation,
    const InterpolationType& interp_type) {
  DRAKE_THROW_UNLESS(times.size() == knots_rotation.size());
  DRAKE_THROW_UNLESS(times.size() == knots_translation.size());
  const PiecewiseQuaternionSlerp<double> rotation(times, knots_rotation);
  PiecewisePolynomial<double> translation;
  switch (interp_type) {
    case InterpolationType::kFirstOrderHold:
      translation = PiecewisePolynomial<double>::FirstOrderHold(
          times, ToVectorOfMatrixXd(knots_translation));
      break;
    case InterpolationType::kCubic:
      translation = PiecewisePolynomial<double>::Cubic(
          times, ToVectorOfMatrixXd(knots_translation));
      break;
    case InterpolationType::kPchip:
      translation = PiecewisePolynomial<double>::Pchip(
          times, ToVectorOfMatrixXd(knots_translation));
      break;
    default:
      throw std::logic_error("The provided interp_type is not supported.");
  }
  return AgentTrajectory{translation, rotation};
}

AgentTrajectory AgentTrajectory::MakeCubicFromWaypoints(
    const std::vector<Eigen::Quaternion<double>>& waypoints_rotation,
    const std::vector<Eigen::Vector3d>& waypoints_translation,
    const std::vector<double>& speeds) {
  DRAKE_THROW_UNLESS(!waypoints_rotation.empty());
  DRAKE_THROW_UNLESS(!waypoints_translation.empty());
  DRAKE_THROW_UNLESS(speeds.size() == waypoints_rotation.size());
  DRAKE_THROW_UNLESS(speeds.size() == waypoints_translation.size());
  std::vector<double> times(waypoints_rotation.size());
  std::vector<Eigen::MatrixXd> translations =
      ToVectorOfMatrixXd(waypoints_translation);
  times[0] = 0.;
  // Populate the segment times given a piecewise-linear travel time estimate.
  for (int i{0}; i < static_cast<int>(speeds.size()) - 1; i++) {
    DRAKE_THROW_UNLESS(speeds[i] >= 0.);
    // speed_k == 0. ⇒ speed_k+1 > 0., ∀ k = 0..N-1
    DRAKE_THROW_UNLESS(speeds[i + 1] > 0. || speeds[i] != 0.);
    const double distance = (translations[i] - translations[i + 1]).norm();
    const double average_speed = 0.5 * (speeds[i] + speeds[i + 1]);
    const double delta_t = distance / average_speed;
    times[i + 1] = delta_t + times[i];
  }
  const PiecewiseQuaternionSlerp<double> rotation(times, waypoints_rotation);

  // Starting with a piecewise-linear estimate of spline segment lengths, make
  // a loop that refines the segment lengths based on the constructed spline,
  // iterating until a tolerance is met.
  std::vector<Eigen::MatrixXd> linear_velocities(times.size());
  for (int i{0}; i < static_cast<int>(times.size()); i++) {
    const Eigen::Matrix3d rotation_matrix =
        math::RotationMatrix<double>(waypoints_rotation[i]).matrix();
    // Represent forward speed in frame A as velocity in frame W.
    linear_velocities[i] = rotation_matrix * Eigen::Vector3d{speeds[i], 0., 0.};
  }
  const PiecewisePolynomial<double> translation =
      PiecewisePolynomial<double>::Cubic(times, translations,
                                         linear_velocities);

  return AgentTrajectory(translation, rotation);
}

AgentTrajectory AgentTrajectory::MakeCubicFromWaypoints(
    const std::vector<Eigen::Quaternion<double>>& waypoints_rotation,
    const std::vector<Eigen::Vector3d>& waypoints_translation, double speed) {
  DRAKE_THROW_UNLESS(waypoints_rotation.size() == waypoints_translation.size());
  DRAKE_THROW_UNLESS(speed > 0.);
  std::vector<double> speeds(waypoints_rotation.size());
  std::fill(speeds.begin(), speeds.end(), speed);
  return MakeCubicFromWaypoints(waypoints_rotation, waypoints_translation,
                                speeds);
}

AgentTrajectory::AgentTrajectory(
    const PiecewisePolynomial<double>& translation,
    const PiecewiseQuaternionSlerp<double>& rotation)
    : translation_(translation),
      rotation_(rotation),
      translation_dot_(translation.derivative()) {}

}  // namespace automotive
}  // namespace drake
