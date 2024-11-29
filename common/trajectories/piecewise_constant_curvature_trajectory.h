#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_trajectory.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace trajectories {

/**
 * Represents a piecewise-constant-curvature, continuously-differentiable
 * trajectory within a 2D plane embedded in 3D space, i.e. a trajectory composed
 * of circular arcs and line segments.
 *
 * This class tracks X_AF(t), a pose F relative to a reference frame A as a
 * funciton of time t [s], with position r(t) = p_AoFo_A and R(t) = R_AF. The
 * initial position r(t₀) is assumed to be equal to the zero vector. r(t) is the
 * nominal value of the trajectory, and the full pose and its derivatives are
 * available through CalcPose(), CalcSpatialVelocity(), and
 * CalcSpatialAcceleration().
 *
 * The trajectory is arclength-parameterized; this means velocity along the
 * curve tangent is always 1 [m/s], and t̂(t) = dr(t)/dt is the curve's tangent
 * unit vector (in the Frenet-Serrat sence).
 *
 * The plane has a normal axis p̂. The curvatures are defined piecewise
 * leveraging the "segment times" concept from PiecewiseTrajectory. The segment
 * break times t₀ [s] < t₁ [s] < ... < tₙ [s] define the finite list of
 * points at which the curvature instantly changes. Like all PiecewiseTrajectory
 * types, tᵢ - tᵢ₋₁ must be at least PiecewiseTrajectory::kEpsilonTime. On
 * "segment" i, defined as [tᵢ₋₁, tᵢ], the curvature is parameterized by a
 * constant "turning rate" kᵢ [1/m], setting the angular velocity of F in W
 * (equivalently the Darboux vector) as w_AF = kᵢ * p̂, and the curvature is
 * equal to the magnitude of the turning rate abs(k). Thus,
 *
 *   * kᵢ > 0 corresponds to a counterclockwise turn along a circular arc of
 *     radius 1/kᵢ [m].
 *   * kᵢ = 0 corresponds to a straight line segment.
 *   * kᵢ < 0 corresponds to a clockwise turn along a circular arc of radius
 * -1/kᵢ [m].
 *
 * The angular velocity w_AF is finite and constant within the segments. Thus
 * the angular acceleration alpha_AF is zero within the segments and undefined
 * at the segment endpoints. R(t) is continuous.
 *
 * For brevity, we refer to the pose at the start of each segment as Fᵢ, and
 * the associated transfrom X_AF(tᵢ) as X_AFᵢ.
 *
 * Before t₀ and after tₙ, the pose, velocity, and acceleration are
 * continuously extrapolated with constant curvatures k_1 and k_N respectively.
 *
 * The orientation of the curve R(t) = [x̂_F_A(t), ŷ_F_A(t), ẑ_F_A(t)] rotates
 * with the direction of the curve, similar to but distinct from the
 * Frenet-Serret/tangent-normal-binormal (TNB) frame [t̂, n̂, b̂]:
 *
 *   * x̂_F_A(t) = dr(t)/dt = t̂ is the curve tangent at time t
 *   * ẑ_F_A(t) = ẑ_F_A(0) = p̂, the plane normal. ẑ = sign(kᵢ) * b̂.
 *   * ŷ_F_A(t) = ẑ_F_A x x̂_F_A = sign(k) * n̂.
 *
 * From the Frenet-Serret formulas, we know:
 *
 *   * dx̂_F_A(t)/dt =  kᵢ * ŷ_F_A(t)
 *   * dŷ_F_A(t)/dt = -kᵢ * x̂_F_A(t)
 *   * dẑ_F_A(t)/dt =  0
 *
 * @tparam_default_scalar
 */
template <typename T>
class PiecewiseConstantCurvatureTrajectory final
    : public PiecewiseTrajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewiseConstantCurvatureTrajectory);

  /**
   * Constructs an empty piecewise constant curvature trajectory.
   */
  PiecewiseConstantCurvatureTrajectory() = default;

  /**
   * Constructs a piecewise constant curvature trajectory.
   *
   * Endpoints of each constant-curvature segments are defined by break times
   * t₀ [s] < t₁ [s] < ... < tₙ [s] passed through `breaks`. N is the
   * number of segments. The turning rates k₁ [1/m], ... kₙ [1/m] are passed
   * through `turning_rates`. There must be exactly 1 turning rate provided per
   * segment (len(turning_rates) = len(breaks) -1).
   *
   * The parent class, PiecewiseTrajectory, will internally enforce that the
   * break times increase by at least PiecewiseTrajectory::kEpsilonTime.
   *
   * The initial rotation in the reference frame R_AF(t₀) is set with
   * initial_curve_tangent as x̂_F_A(t₀); and plane_normal as the ŷ_F_A(t); and
   * ŷ_F_A(0) set by right-hand rule. Thus if the axis are not unit-norm and
   * orthogonal, construction of R_AF may fail; see MakeBaseFrame for details.
   *
   * @param breaks A vector of time break points between segments.
   * @param turning_rates A vector of curvatures for each segment.
   * @param initial_curve_tangent The initial tangent of the curve expressed in
   * the parent frame, [t̂(t₀)]_A.
   * @param plane_normal The normal axis of the 2D plane in which the curve
   * lies, expressed in the parent frame, [p̂]_A. lies,
   *
   * @throws std::invalid_argument if the number of turning rates does not
   * match the number of segments.
   */
  PiecewiseConstantCurvatureTrajectory(const std::vector<T>& breaks,
                                       const std::vector<T>& turning_rates,
                                       const Vector3<T>& initial_curve_tangent,
                                       const Vector3<T>& plane_normal);

  /**
   * Constructs a PiecewiseConstantCurvatureTrajectory from another scalar
   * type.
   *
   * @tparam_default_scalar
   * @return The cloned trajectory.
   */
  template <typename U>
  explicit PiecewiseConstantCurvatureTrajectory(
      const PiecewiseConstantCurvatureTrajectory<U> other)
      : PiecewiseConstantCurvatureTrajectory(
            CloneSegmentDataFromScalar<U>(other.get_segment_times()),
            CloneSegmentDataFromScalar<U>(other.segment_turning_rates_),
            other.segment_start_poses_[0]
                .rotation()
                .col(kCurveTangentIndex)
                .template cast<U>(),
            other.segment_start_poses_[0]
                .rotation()
                .col(kPlaneNormalIndex)
                .template cast<U>()) {}

  /**
   * @return The number of rows in the trajectory's output.
   */
  Eigen::Index rows() const override { return 3; }

  /**
   * @return The number of columns in the trajectory's output.
   */
  Eigen::Index cols() const override { return 1; }

  /**
   * Returns the trajectory's pose X_AF(t) at the given time t [s]
   * (equivalently, arclength [m]).
   *
   * @param t The query time in seconds (equivalently, arclenth in meters).
   * @return The corresponding pose X_AF(t) as a RigidTransform.
   */
  math::RigidTransform<T> GetPose(const T& t) const;

  /**
   * Returns the trajectory position p_AoFo_A(t) at the given time t [s]
   * (equivalently, arclength [m]).
   *
   * @param t The query time in seconds (equivalently, arclenth in meters).
   * @return The 3x1 position vector.
   */
  MatrixX<T> value(const T& t) const override {
    return GetPose(t).translation();
  }

  /**
   * Creates a deep copy of this trajectory.
   *
   * @return A unique pointer to the cloned trajectory.
   */
  std::unique_ptr<Trajectory<T>> Clone() const override;

  /**
   * Returns the spatial velocity V_AF_A(t) at the given time t [s]
   * (equivalently, arclength [m]).
   *
   * @param t The query time in seconds (equivalently, arclenth in meters).
   * @return The correponding spatial velocity V_AF_A(t).
   */
  multibody::SpatialVelocity<T> CalcSpatialVelocity(const T& t) const;

  /**
   * Returns the spatial acceleration A_AF_A(t) at the given time t [s]
   * (equivalently, arclength [m]).
   *
   * @param t The query time in seconds (equivalently, arclenth in meters).
   * @return The correponding spatial acceleration A_AF_A(t).
   */
  multibody::SpatialAcceleration<T> CalcSpatialAcceleration(const T& t) const;

  /**
   * Checks if the trajectory is nearly periodic within the given tolerance.
   *
   * Periodicity is defined as the beginning and end poses X_AF(t₀) and
   * X_AF(tₙ) being equal up to the same tolerance, checked via
   * RigidTransform::IsNearlyEqualTo with the same tolerance.
   *
   * @param tolerance The tolerance for periodicity check.
   * @return True if the trajectory is nearly periodic, false otherwise.
   */
  boolean<T> IsNearlyPeriodic(double tolerance) const;

  /**
   * Gives the intial orientation of the trajectory in its reference frame,
   * R_AF(t₀).
   *
   * @return The base frame as a RotationMatrix.
   */
  const math::RotationMatrix<T>& get_base_frame() const {
    return segment_start_poses_[0].rotation();
  }

  /**
   * Gives the normal axis of the plane in which the curve lies, [p̂]_A.
   *
   * @return The plane normal axis, [p̂]_A.
   */
  const Eigen::Ref<const Vector3<T>> get_plane_normal_axis() const {
    return segment_start_poses_[0].rotation().col(kPlaneNormalIndex);
  }

 private:
  template <typename U>
  friend class PiecewiseConstantCurvatureTrajectory;

  /**
   * Helper function to convert segment data (break times, turning rates) scalar
   * types, to aid conversion of the trajectory to different scalar types.
   *
   * @param segment_data The vector of segment data to be converted.
   * @return The segment data converted to new scalar type.
   * @tparam_default_scalar
   */
  template <typename U>
  static std::vector<T> CloneSegmentDataFromScalar(
      const std::vector<U>& segment_data) {
    std::vector<T> converted_segment_data;
    systems::scalar_conversion::ValueConverter<U, T> converter;
    for (const U& segment : segment_data) {
      converted_segment_data.push_back(converter(segment));
    }
    return converted_segment_data;
  }
  /**
   * Calculates the relative transform between the start of a constant-curvature
   * segment tᵢ [s] and the pose after a given duration Δt [s] (equivalently,
   * arclength [s]) within the segment.
   *
   * This transform is equal to X_AFᵢ⁻¹ * X_AF(tᵢ + Δt). As X_AF(t)
   * defines the normal of the plane as the z axis of F, this displacement
   * consists of a circular arc or line segment in the x-y plane, and a
   * corresponding z-axis rotation.
   *
   * @param k_i The turning rate of the segment.
   * @param dt The duration [s] within the segment.
   * @return The pose relative to to the start of the segment.
   */
  static math::RigidTransform<T> CalcRelativePoseInSegment(const T& k_i,
                                                           const T& dt);

  /**
   * Builds the base frame, R_AF(t₀), from the given tangent and plane axes.
   *
   * The plane normal p̂ is defined as the z axis ẑ_F_A; the initial heading is
   * the x axis x̂_F_A; and the y axis is determined by cross product. R_AF is
   * constructed by passing normalized versions of these vectors to
   * RotationMatrix::MakeFromOrthonormalColumns, which may fail if the provided
   * axes are not unit norm and orthogonal.
   *
   * @param initial_curve_tangent The tangent of the curve at time t₀.
   * @param plane_normal The normal axis of the plane.
   *
   * @return The base frame as a RotationMatrix.
   */
  static math::RotationMatrix<T> MakeBaseFrame(
      const Vector3<T>& initial_curve_tangent, const Vector3<T>& plane_normal);

  /**
   * Calculates the curve's pose at the beginning of each segment.
   *
   * For each segment i, the returned vector's i-th element contains the
   * relative transform X_AFᵢ = X_AF(tᵢ).
   *
   * @param base_frame The base frame of the trajectory.
   * @param turning_rates THe vector of turning rates [1/m] for each segment.
   * @param breaks The vector of break points [s] between segments tᵢ.
   *
   * @return Vector (of length num_segments) of poses at the beggining of each
   * segment.
   */
  static std::vector<math::RigidTransform<T>> MakeSegmentStartPoses(
      const math::RotationMatrix<T>& base_frame,
      const std::vector<T>& turning_rates, const std::vector<T>& breaks);

  std::vector<T> segment_turning_rates_;
  std::vector<math::RigidTransform<T>> segment_start_poses_;

  static inline constexpr size_t kCurveTangentIndex = 0;
  static inline constexpr size_t kCurveNormalIndex = 1;
  static inline constexpr size_t kPlaneNormalIndex = 2;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewiseConstantCurvatureTrajectory);
