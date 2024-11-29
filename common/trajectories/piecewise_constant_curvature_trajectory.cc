#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"

#include <functional>
#include <limits>

namespace drake {
namespace trajectories {

template <typename T>
PiecewiseConstantCurvatureTrajectory<T>::PiecewiseConstantCurvatureTrajectory(
    const std::vector<T>& breaks, const std::vector<T>& turning_rates,
    const Vector3<T>& initial_curve_tangent, const Vector3<T>& plane_normal)
    : PiecewiseTrajectory<T>(breaks),
      segment_turning_rates_(turning_rates),
      segment_start_poses_(MakeSegmentStartPoses(
          MakeBaseFrame(initial_curve_tangent, plane_normal), turning_rates,
          breaks)) {
  if (turning_rates.size() != breaks.size() - 1) {
    throw std::invalid_argument(
        "The number of turning rates must be equal to the number of segments.");
  }
}

template <typename T>
std::unique_ptr<Trajectory<T>> PiecewiseConstantCurvatureTrajectory<T>::Clone()
    const {
  auto base_frame = segment_start_poses_[0].rotation();
  return std::make_unique<PiecewiseConstantCurvatureTrajectory<T>>(
      this->breaks(), segment_turning_rates_,
      base_frame.col(kCurveTangentIndex), base_frame.col(kPlaneNormalIndex));
}

template <typename T>
math::RigidTransform<T> PiecewiseConstantCurvatureTrajectory<T>::GetPose(
    const T& t) const {
  int segment_index = this->get_segment_index(t);
  const math::RigidTransform<T> X_FiF =
      CalcRelativePoseInSegment(segment_turning_rates_[segment_index],
                                t - this->start_time(segment_index));
  return segment_start_poses_[segment_index] * X_FiF;
}

template <typename T>
multibody::SpatialVelocity<T>
PiecewiseConstantCurvatureTrajectory<T>::CalcSpatialVelocity(const T& t) const {
  const math::RotationMatrix<T> R_AF = GetPose(t).rotation();
  const T& k_i = segment_turning_rates_[this->get_segment_index(t)];

  multibody::SpatialVelocity<T> spatial_velocity;
  // The linear speed ds/dt along the curve is always defined to be 1 [m/s].
  // The curvature is defined to be abs(kᵢ).
  //
  // From Frenet-Serret analysis and the class doc for
  // PiecewiseConstantCurvatureTrajectory, the rotational velocity is equal to
  // the Darboux vector ds/dt * curvature * b̂ = kᵢ * ẑ_F_A.
  spatial_velocity.rotational() = k_i * R_AF.col(kPlaneNormalIndex);

  // The translational velocity is also then equal to ds/dt * t̂ = x̂_F_A(t).
  spatial_velocity.translational() = R_AF.col(kCurveTangentIndex);

  return spatial_velocity;
}

template <typename T>
multibody::SpatialAcceleration<T>
PiecewiseConstantCurvatureTrajectory<T>::CalcSpatialAcceleration(
    const T& t) const {
  const math::RotationMatrix<T> R_AF = GetPose(t).rotation();
  const T& k_i = segment_turning_rates_[this->get_segment_index(t)];

  multibody::SpatialAcceleration<T> spatial_acceleration;
  // As ds/dt = 1 [m/s], we have ds²/dt² = 0.
  // Thus, there is no azumithal/transverse/Euler acceleration component.
  //
  // As the curve does not have continuous acceleration at the break times, by
  // convention we set the acceleration at the break time tᵢ to be the limit
  // as approached from the right -- i.e. the acceleration is continuous on each
  // segoment domain [tᵢ, tᵢ₊₁) (see implementation of
  // PiecewiseTrajectory::get_segment_index for convention on the segment index
  // of tᵢ).
  //
  // From the Frenet-Serrat analysis in the class doc for
  // PiecewiseConstantCurvatureTrajectory, we know that the angular velocity is
  // kᵢ * ẑ_F_A. As ẑ_F_A and kᵢ are constant everywhere except the break times,
  // the angular acceleration is then equal to zero.
  spatial_acceleration.rotational() = Vector3<T>::Zero();

  // We also know that the translational velocity is x̂_F_A and thus the
  // translational acceleration is dx̂_F_A/dt = kᵢ * ŷ_F_A(t).
  spatial_acceleration.translational() = k_i * R_AF.col(kCurveNormalIndex);
  return spatial_acceleration;
}

template <typename T>
boolean<T> PiecewiseConstantCurvatureTrajectory<T>::IsNearlyPeriodic(
    double tolerance) const {
  return GetPose(this->start_time())
      .IsNearlyEqualTo(GetPose(this->end_time()), tolerance);
}

template <typename T>
math::RigidTransform<T>
PiecewiseConstantCurvatureTrajectory<T>::CalcRelativePoseInSegment(
    const T& k_i, const T& dt) {
  Vector3<T> p_FioFo_Fi = Vector3<T>::Zero();
  // Calculate rotation angle
  const T theta = dt * k_i;
  math::RotationMatrix<T> R_FiF = math::RotationMatrix<T>::MakeZRotation(theta);
  if (k_i == T(0)) {
    // Case 1: zero curvature (straight line)
    //
    // The tangent axis is constant, and the tangential speed is 1 [m/s].
    // Thus the displacement p_FioFo_Fi is just t̂ * Δt.
    p_FioFo_Fi(kCurveTangentIndex) = dt;
  } else {
    // Case 2: non-zero curvature (circular arc)
    //
    // The entire trajectory lies in a plan with normal ẑ, and thus
    // the arc is embedded in the x-y plane.
    //
    //     ŷ_Fᵢ
    //      ↑
    //    C o             x
    //      │\            x
    //      │ \           x
    //      │  \         x
    //      │   \       x
    //      │    \     x
    //      │     \  xx
    //      │ p_Fₒ ox
    //      │   xxx
    //      └xxx───────────────→ x̂_Fᵢ
    //     p_Fᵢₒ
    //
    // The circular arc's centerpoint C is located at p_FᵢₒC = (1/kᵢ) * ŷ_Fᵢ,
    // with initial direction x̂_Fᵢ, radius abs(1/kᵢ), and constant tagential
    // speed 1 [m/s]. Thus the angle traveled along the arc is θ = kᵢ * Δt,
    // with the sign of kᵢ handeling the clockwise/counterclockwise direction.
    // The kᵢ > 0 case is shown above.
    p_FioFo_Fi(kCurveNormalIndex) = (T(1) - cos(theta)) / k_i;
    p_FioFo_Fi(kCurveTangentIndex) = sin(theta) / k_i;
  }
  return math::RigidTransform<T>(R_FiF, p_FioFo_Fi);
}

template <typename T>
math::RotationMatrix<T> PiecewiseConstantCurvatureTrajectory<T>::MakeBaseFrame(
    const Vector3<T>& initial_curve_tangent, const Vector3<T>& plane_normal) {
  const Vector3<T> initial_y_hat_F_A =
      plane_normal.cross(initial_curve_tangent);

  return math::RotationMatrix<T>::MakeFromOrthonormalColumns(
      initial_curve_tangent.normalized(), initial_y_hat_F_A.normalized(),
      plane_normal.normalized());
}

template <typename T>
std::vector<math::RigidTransform<T>>
PiecewiseConstantCurvatureTrajectory<T>::MakeSegmentStartPoses(
    const math::RotationMatrix<T>& base_frame,
    const std::vector<T>& turning_rates, const std::vector<T>& breaks) {
  const size_t num_breaks = breaks.size();
  const size_t num_segments = num_breaks - 1;

  // calculate angular change and length of each segment.
  const VectorX<T> breaks_eigen =
      Eigen::Map<const VectorX<T>>(breaks.data(), num_breaks);
  const VectorX<T> segment_durations =
      breaks_eigen.tail(num_segments) - breaks_eigen.head(num_segments);

  // build frames for the start of each segment.
  std::vector<math::RigidTransform<T>> segment_start_poses;
  segment_start_poses.reserve(num_segments);
  segment_start_poses.push_back(math::RigidTransform<T>(base_frame));

  for (size_t i = 0; i < (num_segments - 1); i++) {
    math::RigidTransform<T> X_FiFip1 =
        CalcRelativePoseInSegment(turning_rates[i], segment_durations[i]);
    segment_start_poses.push_back(segment_start_poses.back() * X_FiFip1);
  }

  return segment_start_poses;
}

// Explicit instantiation for the types specified in the header
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewiseConstantCurvatureTrajectory);

}  // namespace trajectories
}  // namespace drake
