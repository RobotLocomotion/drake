#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"

#include <functional>
#include <limits>

namespace drake {
namespace trajectories {

template <typename T>
PiecewiseConstantCurvatureTrajectory<T>::PiecewiseConstantCurvatureTrajectory(
    const std::vector<T>& breaks, const std::vector<T>& turning_rates,
    const Vector3<T>& initial_curve_tangent, const Vector3<T>& plane_normal,
    const Vector3<T>& initial_position, bool is_periodic)
    : PiecewiseTrajectory<T>(breaks),
      segment_turning_rates_(turning_rates),
      is_periodic_(is_periodic) {
  if (turning_rates.size() != breaks.size() - 1) {
    throw std::logic_error(
        "The number of turning rates must be equal to the number of segments.");
  }
  if (breaks[0] != 0) {
    throw std::logic_error(
        "The first break must be 0, as the breaks are in arclength units.");
  }
  if (initial_curve_tangent.norm() == 0) {
    throw std::logic_error("The norm of the initial tangent is zero.");
  }
  if (plane_normal.norm() == 0) {
    throw std::logic_error("The norm of the plane's normal is zero.");
  }
  using std::abs;
  constexpr double tolerance =
      math::RotationMatrix<T>::get_internal_tolerance_for_orthonormality();
  if (abs(initial_curve_tangent.dot(plane_normal)) > tolerance) {
    throw std::logic_error(
        "The initial curve's tangent vector must be perpendicular to the "
        "plane's normal.");
  }
  break_poses_ = MakeBreakPoses(
      MakeInitialPose(initial_curve_tangent, plane_normal, initial_position),
      breaks, turning_rates);
}

template <typename T>
math::RigidTransform<T> PiecewiseConstantCurvatureTrajectory<T>::CalcPose(
    const T& s) const {
  const T w = maybe_wrap(s);
  int segment_index = this->get_segment_index(w);
  const math::RigidTransform<T> X_MiM =
      CalcRelativePoseInSegment(segment_turning_rates_[segment_index],
                                w - this->start_time(segment_index));
  return break_poses_[segment_index] * X_MiM;  // = X_AM
}

template <typename T>
multibody::SpatialVelocity<T>
PiecewiseConstantCurvatureTrajectory<T>::CalcSpatialVelocity(
    const T& s, const T& s_dot) const {
  const math::RotationMatrix<T> R_AM = CalcPose(s).rotation();
  return R_AM * CalcSpatialVelocityInM(s, s_dot);  // V_AM_A
}

template <typename T>
multibody::SpatialVelocity<T>
PiecewiseConstantCurvatureTrajectory<T>::CalcSpatialVelocityInM(
    const T& s, const T& s_dot) const {
  const T& rho_i = curvature(s);

  multibody::SpatialVelocity<T> spatial_velocity;
  // From Frenet–Serret analysis and the class doc for
  // PiecewiseConstantCurvatureTrajectory, the rotational velocity is equal to
  // the Darboux vector (plane normal p̂ (= Mz)) times the tangential velocity:
  // ρ⋅Mz⋅ds/dt. But Mz expressed in M is just [0 0 1].
  spatial_velocity.rotational() = Vector3<T>(0, 0, rho_i * s_dot);

  // The translational velocity is equal to the tangent direction times the
  // tangential velocity: dr(s)/dt = t̂(s)⋅ds/dt = Mx(s)⋅ds/dt. But Mx
  // expressed in M is just [1 0 0].
  spatial_velocity.translational() = Vector3<T>(s_dot, 0, 0);

  return spatial_velocity;  // V_AM_M
}

template <typename T>
multibody::SpatialAcceleration<T>
PiecewiseConstantCurvatureTrajectory<T>::CalcSpatialAcceleration(
    const T& s, const T& s_dot, const T& s_ddot) const {
  const math::RotationMatrix<T> R_AM = CalcPose(s).rotation();
  return R_AM * CalcSpatialAccelerationInM(s, s_dot, s_ddot);
}

template <typename T>
multibody::SpatialAcceleration<T>
PiecewiseConstantCurvatureTrajectory<T>::CalcSpatialAccelerationInM(
    const T& s, const T& s_dot, const T& s_ddot) const {
  const T& rho_i = curvature(s);

  // The spatial acceleration is the time derivative of the spatial velocity.
  // We compute the acceleration by applying the chain rule to the formulas
  // in CalcSpatialVelocity.
  multibody::SpatialAcceleration<T> spatial_acceleration;

  // The angular velocity is ρᵢ⋅Mz⋅ds/dt. As Mz is constant everywhere, and ρᵢ
  // constant everywhere except the breaks, the angular acceleration is then
  // equal to ρᵢ⋅Mz⋅d²s/dt². But Mz expressed in M is just [0 0 1].
  spatial_acceleration.rotational() = Vector3<T>(0, 0, rho_i * s_ddot);

  // The translational velocity is dr(s)/dt = Mx(s)⋅ds/dt. Thus by product and
  // chain rule, the translational acceleration is:
  //    a(s) = d²r(s)/ds² = dMx(s)/ds⋅(ds/dt)² + Mx⋅d²s/dt².
  // From the class doc, we know dMx/ds = ρᵢ⋅My.
  // Thus the translational acceleration is a(s) = ρᵢ⋅My⋅(ds/dt)² + Mx⋅d²s/dt².
  // But expressed in M, Mx=[1 0 0] and My=[0 1 0].
  spatial_acceleration.translational() =
      Vector3<T>(s_ddot, rho_i * s_dot * s_dot, 0);

  return spatial_acceleration;  // A_AM_M
}

template <typename T>
boolean<T> PiecewiseConstantCurvatureTrajectory<T>::EndpointsAreNearlyEqual(
    double tolerance) const {
  return break_poses_.front().IsNearlyEqualTo(break_poses_.back(), tolerance);
}

template <typename T>
std::unique_ptr<Trajectory<T>>
PiecewiseConstantCurvatureTrajectory<T>::DoClone() const {
  auto initial_pose = get_initial_pose();
  auto initial_frame = initial_pose.rotation();
  return std::make_unique<PiecewiseConstantCurvatureTrajectory<T>>(
      this->breaks(), segment_turning_rates_,
      initial_frame.col(kCurveTangentIndex),
      initial_frame.col(kPlaneNormalIndex), initial_pose.translation(),
      is_periodic_);
}

template <typename T>
math::RigidTransform<T>
PiecewiseConstantCurvatureTrajectory<T>::CalcRelativePoseInSegment(
    const T& rho_i, const T& ds) {
  Vector3<T> p_MioMo_Mi = Vector3<T>::Zero();
  // Calculate rotation angle.
  const T theta = ds * rho_i;
  math::RotationMatrix<T> R_MiM = math::RotationMatrix<T>::MakeZRotation(theta);
  if (rho_i == T(0)) {
    // Case 1: zero curvature (straight line)
    //
    // The tangent axis is constant, thus the displacement p_MioMo_Mi is just
    // t̂_Fi⋅Δs.
    p_MioMo_Mi(kCurveTangentIndex) = ds;
  } else {
    // Case 2: non-zero curvature (circular arc)
    //
    // The entire trajectory lies in a plane with normal Miz, and thus
    // the arc is embedded in the Mix-Miy plane.
    //
    //     Miy
    //      ↑
    //    C o             x
    //      │\            x
    //      │ \           x
    //      │  \         x
    //      │   \       x
    //      │    \     x
    //      │     \  xx
    //      │ p_Mo ox
    //      │   xxx
    //      └xxx───────────────→ Mix
    //     p_Mio
    //
    // The circular arc's centerpoint C is located at p_MioC = (1/ρᵢ)⋅Miy,
    // with initial direction Mix and radius abs(1/ρᵢ). Thus the angle traveled
    // along the arc is θ = ρᵢ⋅Δs, with the sign of ρᵢ handling the
    // clockwise/counterclockwise direction. The ρᵢ > 0 case is shown above.
    p_MioMo_Mi(kCurveNormalIndex) = (T(1) - cos(theta)) / rho_i;
    p_MioMo_Mi(kCurveTangentIndex) = sin(theta) / rho_i;
  }
  return math::RigidTransform<T>(R_MiM, p_MioMo_Mi);
}

template <typename T>
math::RigidTransform<T>
PiecewiseConstantCurvatureTrajectory<T>::MakeInitialPose(
    const Vector3<T>& initial_curve_tangent, const Vector3<T>& plane_normal,
    const Vector3<T>& initial_position) {
  const Vector3<T> initial_My_A = plane_normal.cross(initial_curve_tangent);
  const auto R_AM = math::RotationMatrix<T>::MakeFromOrthonormalColumns(
      initial_curve_tangent.normalized(), initial_My_A.normalized(),
      plane_normal.normalized());
  return math::RigidTransform<T>(R_AM, initial_position);  // X_AM
}

template <typename T>
std::vector<math::RigidTransform<T>>
PiecewiseConstantCurvatureTrajectory<T>::MakeBreakPoses(
    const math::RigidTransform<T>& initial_pose, const std::vector<T>& breaks,
    const std::vector<T>& turning_rates) {
  const size_t num_breaks = breaks.size();
  const size_t num_segments = num_breaks - 1;

  // Calculate angular change and length of each segment.
  const VectorX<T> breaks_eigen =
      Eigen::Map<const VectorX<T>>(breaks.data(), num_breaks);
  const VectorX<T> segment_durations =
      breaks_eigen.tail(num_segments) - breaks_eigen.head(num_segments);

  // Build frames for the start of each segment.
  std::vector<math::RigidTransform<T>> segment_break_poses;
  segment_break_poses.reserve(num_breaks);
  segment_break_poses.push_back(initial_pose);  // X_AM(0)

  for (size_t i = 0; i < (num_breaks - 1); i++) {
    math::RigidTransform<T> X_MiMip1 =
        CalcRelativePoseInSegment(turning_rates[i], segment_durations[i]);
    // X_AM(i+1)
    segment_break_poses.push_back(segment_break_poses.back() * X_MiMip1);
  }

  return segment_break_poses;
}

// Explicit instantiation for the types specified in the header
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewiseConstantCurvatureTrajectory);

}  // namespace trajectories
}  // namespace drake
