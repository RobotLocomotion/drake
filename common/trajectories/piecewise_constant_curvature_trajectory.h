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
 * This class tracks X_AF(s), a pose F relative to a reference frame A as a
 * function of arclength s [m], with position r(s) = p_AoFo_A(s) and orientation
 * R(s) = R_AF(s). The initial position r(s₀) is provided at construction. r(s)
 * is the quantity defined through Trajectory::value, and the full pose and its
 * derivatives are available through CalcPose(), CalcSpatialVelocity(), and
 * CalcSpatialAcceleration(). We note that these quantities are derivatives with
 * respect to arclength s, and not time t.
 *
 * The trajectory is arclength-parameterized; thus t̂_A(s) = dr(s)/ds is the
 * curve's tangent unit vector (in the Frenet–Serret sense).
 *
 * The plane has a normal axis p̂. The curvatures are defined piecewise
 * leveraging the "segment times" concept from PiecewiseTrajectory, except this
 * class uses arclength s [m] as the independet variable. The segment breaks s₀
 * < s₁ < ... < sₙ [m] define segments within which curvature is constant. On
 * "segment" i, defined as [sᵢ, sᵢ₊₁), the curvature is parameterized by a
 * constant "turning rate" kᵢ [1/m], setting the angular velocity of F in A
 * (equivalently the Darboux vector) as w_AF = kᵢ * p̂, and the curvature is
 * equal to the magnitude of the turning rate abs(k). Thus,
 *
 *   * kᵢ > 0 indicates a counterclockwise turn along a circular arc of
 *     radius 1/kᵢ [m].
 *   * kᵢ = 0 indicates a straight line segment.
 *   * kᵢ < 0 indicates a clockwise turn along an arc of radius -1/kᵢ [m].
 *
 * The angular velocity w_AF is finite and constant within the segments. Thus
 * the angular acceleration alpha_AF is zero within the segments and undefined
 * at the segment endpoints. R(s) is continuous.
 *
 * For brevity, we refer to the pose at the start of each segment as Fi, and
 * the associated transfrom X_AF(sᵢ) as X_AFi.
 *
 * Before s₀ and after sₙ, the pose, velocity, and acceleration are
 * continuously extrapolated with constant curvatures k_1 and k_N respectively.
 *
 * The orientation of the curve R(s) = [Fx_A(s), Fy_A(s), Fz_A(s)] rotates
 * with the direction of the curve, similar to but distinct from the
 * Frenet–Serret/tangent-normal-binormal (TNB) frame vectors t̂, n̂, b̂:
 *
 *   * Fx(s) = t̂(s) is the curve tangent at arclength s
 *   * Fz(s) = Fz(s₀) = p̂, the plane normal. ẑ = sign(kᵢ) * b̂.
 *   * Fy(s) = Fz(s) x Fx(s) = sign(k) * n̂.
 *
 * For constant curvature paths on a plane, the <a
 * href="https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas">Frenet–Serret
 * formulas</a> simplify to:
 *
 *   * dFx/ds =  kᵢ * Fy
 *   * dFy/ds = -kᵢ * Fx
 *   * dFz/ds =  0
 *
 * @tparam_default_scalar
 * @warning Note that this class models a curve with arclength [m] as the
 * independent variable, rather than time [s] as the Trajectory class and many
 * of its inheriting types assume.
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
   * Endpoints of each constant-curvature segments are defined by breaks
   * s₀ = 0 < s₁ < ... < sₙ [m] passed through `breaks`. N is the
   * number of segments. The turning rates k₀ [1/m], ... kₙ₋₁ [1/m] are passed
   * through `turning_rates`. There must be exactly 1 turning rate provided per
   * segment (len(turning_rates) = len(breaks) -1).
   *
   * The parent class, PiecewiseTrajectory, will internally enforce that the
   * breaks increase by at least PiecewiseTrajectory::kEpsilonTime.
   *
   * The initial rotation in the reference frame R_AF(s₀) is set with
   * initial_curve_tangent as Fx_A(s₀); and plane_normal as the Fy_A(s); and
   * Fy(0) set by right-hand rule. Thus if the axis are not unit-norm and
   * orthogonal, construction of R_AF may fail; see MakeBaseFrame for details.
   *
   * @param breaks A vector of break values sᵢ between segments.
   * @param turning_rates A vector of turning rates kᵢ for each segment.
   * @param initial_curve_tangent The initial tangent of the curve expressed in
   * the parent frame, t̂_A(s₀).
   * @param plane_normal The normal axis of the 2D plane in which the curve
   * lies, expressed in the parent frame, p̂_A.
   * @param initial_position The initial position of the curve expressed in
   * the parent frame, p_AoFo_A(s₀).
   *
   * @throws std::invalid_argument if the number of turning rates does not
   * match the number of segments, or s₀ is not 0.
   */
  PiecewiseConstantCurvatureTrajectory(const std::vector<T>& breaks,
                                       const std::vector<T>& turning_rates,
                                       const Vector3<T>& initial_curve_tangent,
                                       const Vector3<T>& plane_normal,
                                       const Vector3<T>& initial_position);

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
            other.get_base_pose()
                .rotation()
                .col(kCurveTangentIndex)
                .template cast<U>(),
            other.get_base_pose()
                .rotation()
                .col(kPlaneNormalIndex)
                .template cast<U>(),
            other.get_base_pose().translation().template cast<U>()) {}

  /**
   * @return The number of rows in the trajectory's output.
   */
  Eigen::Index rows() const override { return 3; }

  /**
   * @return The number of columns in the trajectory's output.
   */
  Eigen::Index cols() const override { return 1; }

  /**
   * @return The total arclength of the curve in meters
   */
  T length() const { return this->end_time(); }

  /**
   * Returns the trajectory's pose X_AF(s) at the given arclength s [m].
   *
   * @param s The query arclength in meters.
   * @return The corresponding pose X_AF(s) as a RigidTransform.
   */
  math::RigidTransform<T> GetPose(const T& s) const;

  /**
   * Returns the trajectory position p_AoFo_A(s) at the given arclength s [m].
   *
   * @param s The query arclength in meters.
   * @return The 3x1 position vector.
   */
  MatrixX<T> value(const T& s) const override {
    return GetPose(s).translation();
  }

  /**
   * Creates a deep copy of this trajectory.
   *
   * @return A unique pointer to the cloned trajectory.
   */
  std::unique_ptr<Trajectory<T>> Clone() const override;

  /**
   * Returns the spatial velocity V_AF_A(s) at the given arclength s [m].
   *
   * @param s The query arclength in meters.
   * @return The corresponding spatial velocity V_AF_A(s).
   */
  multibody::SpatialVelocity<T> CalcSpatialVelocity(const T& s) const;

  /**
   * Returns the spatial acceleration A_AF_A(s) at the given arclength s [m].
   *
   * @param s The query arclength in meters.
   * @return The corresponding spatial acceleration A_AF_A(s).
   */
  multibody::SpatialAcceleration<T> CalcSpatialAcceleration(const T& s) const;

  /**
   * Checks if the trajectory is nearly periodic within the given tolerance.
   *
   * Periodicity is defined as the beginning and end poses X_AF(s₀) and
   * X_AF(sₙ) being equal up to the same tolerance, checked via
   * RigidTransform::IsNearlyEqualTo with the same tolerance.
   *
   * @param tolerance The tolerance for periodicity check.
   * @return True if the trajectory is nearly periodic, false otherwise.
   */
  boolean<T> IsNearlyPeriodic(double tolerance) const;

  /**
   * Gives the initial pose of the trajectory in its reference frame, X_AF(s₀).
   *
   * @return The base pose as a RigidTransform.
   */
  const math::RigidTransform<T>& get_base_pose() const {
    return segment_start_poses_[0];
  }

  /**
   * Gives the normal axis of the plane in which the curve lies, p̂_A.
   *
   * @return The plane normal axis, p̂_A.
   */
  const Eigen::Ref<const Vector3<T>> get_plane_normal() const {
    return segment_start_poses_[0].rotation().col(kPlaneNormalIndex);
  }

 private:
  template <typename U>
  friend class PiecewiseConstantCurvatureTrajectory;

  /**
   * Helper function to convert segment data (breaks, turning rates) scalar
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
   * segment sᵢ [m] and the pose after a given length of travel Δs [m] within
   * the segment.
   *
   * This transform is equal to X_AFi⁻¹ * X_AF(sᵢ + Δs). As X_AF(s)
   * defines the normal of the plane as the z axis of F, this displacement
   * consists of a circular arc or line segment in the x-y plane, and a
   * corresponding z-axis rotation.
   *
   * @param k_i The turning rate of the segment.
   * @param ds The length [m] within the segment.
   * @return The pose relative to to the start of the segment.
   */
  static math::RigidTransform<T> CalcRelativePoseInSegment(const T& k_i,
                                                           const T& ds);

  /**
   * Builds the base pose, X_AF(s₀), from the given tangent and plane axes.
   *
   * p_AoFo_A(s₀) is provided directly as an argument, and R_AF(s₀) is
   * calculated as follows: The plane normal p̂ is defined as the z axis Fz; the
   * initial heading is the x axis Fx; and the y axis is determined by cross
   * product. R_AF is constructed by passing normalized versions of these
   * vectors to RotationMatrix::MakeFromOrthonormalColumns, which may fail if
   * the provided axes are not unit norm and orthogonal.
   *
   * @param initial_curve_tangent The tangent of the curve at arclength s₀ = 0.
   * @param plane_normal The normal axis of the plane.
   * @param initial_position The initial position of the curve, p_AoFo_A(s₀).
   *
   * @return The base pose as a RotationMatrix.
   */
  static math::RigidTransform<T> MakeBasePose(
      const Vector3<T>& initial_curve_tangent, const Vector3<T>& plane_normal,
      const Vector3<T>& initial_position);

  /**
   * Calculates the curve's pose at the beginning of each segment.
   *
   * For each segment i, the returned vector's i-th element contains the
   * relative transform X_AFi = X_AF(sᵢ).
   *
   * @param base_pose The base pose of the trajectory.
   * @param breaks The vector of break points sᵢ [m] between segments.
   * @param turning_rates The vector of turning rates kᵢ [1/m] for each segment.
   *
   * @return Vector (of length num_segments) of poses at the beginning of each
   * segment.
   */
  static std::vector<math::RigidTransform<T>> MakeSegmentStartPoses(
      const math::RigidTransform<T>& base_pose, const std::vector<T>& breaks,
      const std::vector<T>& turning_rates);

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
