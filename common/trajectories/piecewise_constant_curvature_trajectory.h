#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_trajectory.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/wrap_to.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace trajectories {

/** A piecewise constant curvature trajectory in a plane, where the plane is
 posed arbitrarily in three dimensions.

 The trajectory is defined by the position vector r(s): ℝ → ℝ³, parameterized by
 arclength s, and lies in a plane with normal vector p̂. The parameterization is
 C¹, i.e. position r(s) and tangent vector t̂(s) = dr(s)/ds are continuous
 functions of arclength s. The trajectory's length is divided into segments
 s₀ < s₁ < ... < sₙ, where each interval [sᵢ, sᵢ₊₁) has constant curvature
 determined by the turning rate ρᵢ (with units of 1/m). The turning rate is
 defined such that curvature is κᵢ = |ρᵢ|, with its sign indicating the curve's
 direction around p̂ according ot the right-hand rule (counterclockwise if
 positive and clockwise if negative) . For ρᵢ = 0, the segment is a straight
 line.

 Given the tangent vector t̂(s) = dr(s)/ds and the plane's normal p̂, we define
 the _normal_ vector along the curve as n̂(s) = p̂ × t̂(s). These three vectors
 are used to define a moving frame M along the curve with basis vectors
 Mx, My, Mz coincident with vectors t̂, n̂, p̂, respectively.

 A trajectory is said to be periodic if the pose X_AM of frame M is a periodic
 function of arclength, i.e. X_AM(s) = X_AM(s + k⋅L) ∀ k ∈ ℤ, where L is the
 length of a single cycle along the trajectory. Periodicity is determined
 at construction.

 @note Though similar, frame M is distinct from the Frenet–Serret frame defined
 by the tangent-normal-binormal vectors T̂, N̂, B̂ See <a
 href="https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas">Frenet–Serret
 formulas</a> for further reading.

 For constant curvature paths on a plane, the <a
 href="https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas">Frenet–Serret
 formulas</a> simplify and we can write: <pre>
     dMx/ds(s) =  ρ(s)⋅My(s)
     dMy/ds(s) = -ρ(s)⋅Mx(s)
     dMz/ds(s) =  0
 </pre>
 for the entire trajectory.

 The spatial orientation of the curve is defined by the plane's normal p̂ and
 the initial tangent vector t̂₀ at s = 0. At construction, these are provided
 in a reference frame A, defining the pose X_AM₀ at s = 0. This class provides
 functions to compute the curve's kinematics given by its pose X_AM (comprising
 position vector r(s) = p_AoMo_A(s) and orientation R_AM(s)), spatial velocity
 V_AM_A(s) and spatial acceleration A_AM_A(s). We also provide functions to
 return these quantities in the M frame (where they are much simpler), i.e.
 V_AM_M(s) and A_AM_M(s).

 @warning Note that this class models a curve parameterized by arclength,
 rather than time as the Trajectory class and many of its inheriting types
 assume. Time derivatives, i.e. spatial velocity and acceleration, are computed
 for externally provided values of velocity ṡ and acceleration s̈ along the
 curve.

 @see multibody::CurvilinearJoint

 @tparam_default_scalar */
template <typename T>
class PiecewiseConstantCurvatureTrajectory final
    : public PiecewiseTrajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewiseConstantCurvatureTrajectory);

  /** An empty piecewise constant curvature trajectory. */
  PiecewiseConstantCurvatureTrajectory() = default;

  template <typename U>
  using ScalarValueConverter =
      typename systems::scalar_conversion::template ValueConverter<T, U>;

  /** Constructs a piecewise constant curvature trajectory.

   Endpoints of each constant-curvature segments are defined by n breaks
   s₀ = 0 < s₁ < ... < sₙ (in meters). The turning rates ρ₀, ..., ρₙ₋₁ (in 1/m)
   are passed through `turning_rates`. There must be exactly one turning rate
   per segment, i.e. turning_rates.size() == breaks.size() - 1.

   @warning Users of this class are responsible for providing
   `initial_curve_tangent`, `plane_normal` and `initial_position` expressed in
   the same reference frame A.

   The `initial_curve_tangent` t̂₀ and the `plane_normal` p̂, along with the
   initial curve's normal n̂₀ = p̂ × t̂₀, define the initial orientation R_AM₀ of
   frame M at s = 0.

   @param breaks A vector of n break values sᵢ between segments. The parent
   class, PiecewiseTrajectory, enforces that the breaks increase by at least
   PiecewiseTrajectory::kEpsilonTime.
   @param turning_rates A vector of n-1 turning rates ρᵢ for each segment.
   @param initial_curve_tangent The initial tangent of the curve expressed in
   the parent frame, t̂_A(s₀) = Mx_A(s₀).
   @param plane_normal The normal axis of the 2D plane in which the curve
   lies, expressed in the parent frame, p̂_A = Mz_A (constant).
   @param initial_position The initial position of the curve expressed in
   the parent frame, p_AoMo_A(s₀).
   @param is_periodic If true, then the newly constructed trajectory will be
   periodic. That is, X_AM(s) = X_AM(s + k⋅L) ∀ k ∈ ℤ, where L equals length().
   Subsequent calls to is_periodic() will return `true`.

   @throws std::exception if the number of turning rates does not match
   the number of segments
   @throws std::exception if or s₀ is not 0.
   @throws std::exception if initial_curve_tangent or plane_normal have zero
   norm.
   @throws std::exception if initial_curve_tangent is not perpendicular to
   plane_normal. */
  PiecewiseConstantCurvatureTrajectory(const std::vector<T>& breaks,
                                       const std::vector<T>& turning_rates,
                                       const Vector3<T>& initial_curve_tangent,
                                       const Vector3<T>& plane_normal,
                                       const Vector3<T>& initial_position,
                                       bool is_periodic = false);

  /** Scalar conversion constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit PiecewiseConstantCurvatureTrajectory(
      const PiecewiseConstantCurvatureTrajectory<U> other)
      : PiecewiseConstantCurvatureTrajectory(
            ScalarConvertStdVector<U>(other.get_segment_times()),
            ScalarConvertStdVector<U>(other.segment_turning_rates_),
            other.get_initial_pose()
                .rotation()
                .col(kCurveTangentIndex)
                .unaryExpr(ScalarValueConverter<U>{}),
            other.get_initial_pose()
                .rotation()
                .col(kPlaneNormalIndex)
                .unaryExpr(ScalarValueConverter<U>{}),
            other.get_initial_pose().translation().unaryExpr(
                ScalarValueConverter<U>{}),
            other.is_periodic()) {}

  /** @returns the total arclength of the curve in meters. */
  T length() const { return this->end_time(); }

  /** @returns `true` if `this` trajectory is periodic.
   That is, X_AM(s) = X_AM(s + k⋅L) ∀ k ∈ ℤ, where L equals length(). */
  bool is_periodic() const { return is_periodic_; }

  /** Calculates the trajectory's pose X_AM(s) at the given arclength s.

   @note If the trajectory is aperiodic, for s < 0 and s > length() the pose is
   extrapolated as if the curve continued with the curvature of the
   corresponding end segment.

   @param s The query arclength in meters.
   @returns the pose X_AM(s). */
  math::RigidTransform<T> CalcPose(const T& s) const;

  /** Computes r(s), the trajectory's position p_AoMo_A(s) expressed in
   reference frame A, at the given arclength s.

   @param s The query arclength in meters.
   @returns position vector r(s). */
  MatrixX<T> value(const T& s) const {
    // We shadowed the base class to add documentation, not to change logic.
    return PiecewiseTrajectory<T>::value(s);
  }

  /** Returns the signed curvature ρ(s).

   @param s The query arclength in meters.
   @returns curvature ρ(s) */
  const T& curvature(const T& s) const {
    const T w = maybe_wrap(s);
    return segment_turning_rates_[this->get_segment_index(w)];
  }

  /** Computes the spatial velocity V_AM_A(s,ṡ) of frame M measured and
   expressed in the reference frame A. See the class's documentation for frame
   definitions.

   In frame invariant notation, the angular velocity ω(s) and translational
   velocity v(s) are: <pre>
     ω(s) = ṡ⋅ρ(s)⋅p̂
     v(s) = ṡ⋅t̂(s)
   </pre>
   where ρ(s) and t̂(s) are extrapolated for s < 0 and s > length() keeping the
   constant curvature of the corresponding end segment.

   @param s The query arclength, in meters.
   @param s_dot The magnitude ṡ of the tangential velocity along the curve, in
     meters per second.
   @retval V_AM_A The spatial velocity of M in A, expressed in A. */
  multibody::SpatialVelocity<T> CalcSpatialVelocity(const T& s,
                                                    const T& s_dot) const;

  /** Computes the spatial velocity V_AM_M(s,ṡ) of frame M measured in frame A
   but expressed in frame M.

   See CalcSpatialVelocity() for details. Note that when expressed in frame M,
   both p̂ and t̂ are constant, with p̂_M = [0 0 1]ᵀ and t̂_M = [1 0 0]ᵀ so the
   returned spatial velocity is just `V_AM_M = [0 0 ṡ⋅ρ(s) ṡ 0 0]ᵀ`.

   @param s The query arclength, in meters.
   @param s_dot The magnitude ṡ of the tangential velocity along the curve, in
     meters per second.
   @retval V_AM_M The spatial velocity of M in A, expressed in M. */
  multibody::SpatialVelocity<T> CalcSpatialVelocityInM(const T& s,
                                                       const T& s_dot) const;

  /** Computes the spatial acceleration A_AM_A(s,ṡ,s̈) of frame M measured and
   expressed in the reference frame A. See the class's documentation for frame
   definitions.

   In frame invariant notation, the angular acceleration α(s) and translational
   acceleration a(s) are: <pre>
     α(s) = s̈⋅ρ(s)⋅p̂
     a(s) = ṡ²⋅ρ(s)⋅n̂(s) + s̈⋅t̂(s)
   </pre>
   where ρ(s), t̂(s) and n̂(s) are extrapolated for s < 0 and s > length()
   keeping the constant curvature of the corresponding end segment.

   As the curve does not have continuous acceleration at the breaks, by
   convention we set the acceleration at the break sᵢ to be the limit as
   approached from the right -- i.e. the acceleration is continuous on each
   segment domain [sᵢ, sᵢ₊₁).

   @param s The query arclength, in meters.
   @param s_dot The magnitude ṡ of the tangential velocity along the curve, in
     meters per second.
   @param s_ddot The magnitude s̈ of the tangential acceleration along the
     curve, in meters per second squared.

   @retval A_AM_A The spatial acceleration of M in A, expressed in A. */
  multibody::SpatialAcceleration<T> CalcSpatialAcceleration(
      const T& s, const T& s_dot, const T& s_ddot) const;

  /** Computes the spatial acceleration A_AM_M(s,ṡ,s̈) of frame M measured in
   frame A but expressed in frame M.

   See CalcSpatialAcceleration() for details. Note that when expressed in frame
   M, the vectors p̂, t̂, and n̂ are constant, with p̂_M = [0 0 1]ᵀ, t̂_M = [1 0 0]ᵀ,
   and n̂_M = [0 1 0]ᵀ so the returned spatial acceleration is just
   `A_AM_M = [0 0 s̈⋅ρ(s) s̈ ṡ²⋅ρ(s) 0]ᵀ`.

   @param s The query arclength, in meters.
   @param s_dot The magnitude ṡ of the tangential velocity along the curve, in
     meters per second.
   @param s_ddot The magnitude s̈ of the tangential acceleration along the
     curve, in meters per second squared.

   @retval A_AM_M The spatial acceleration of M in A, expressed in M. */
  multibody::SpatialAcceleration<T> CalcSpatialAccelerationInM(
      const T& s, const T& s_dot, const T& s_ddot) const;

  /** @returns `true` if the trajectory is periodic within a given `tolerance`.

   Periodicity is defined as the beginning and end poses X_AM(s₀) and
   X_AM(sₙ) being equal up to the same tolerance, checked via
   RigidTransform::IsNearlyEqualTo() using `tolerance`.

   @param tolerance The tolerance for the pose equality check. */
  boolean<T> EndpointsAreNearlyEqual(double tolerance) const;

 private:
  template <typename U>
  friend class PiecewiseConstantCurvatureTrajectory;

  // Trajectory overrides.
  std::unique_ptr<Trajectory<T>> DoClone() const final;
  MatrixX<T> do_value(const T& s) const final {
    return CalcPose(s).translation();
  }
  Eigen::Index do_rows() const final { return 3; }
  Eigen::Index do_cols() const final { return 1; }

  /* Helper function to scalar convert a std::vector<U> into a std::vector<T>.
   @param segment_data std::vector storing types U.
   @returns the input vector scalar converted T. */
  template <typename U>
  static std::vector<T> ScalarConvertStdVector(
      const std::vector<U>& segment_data) {
    std::vector<T> converted_segment_data;
    ScalarValueConverter<U> converter;
    for (const U& segment : segment_data) {
      converted_segment_data.push_back(converter(segment));
    }
    return converted_segment_data;
  }

  /* If the trajectory is periodic, this helper wraps the distance coordinate s
   to the path's length. If not periodic, then it simply returns s. */
  T maybe_wrap(const T& s) const {
    return is_periodic_ ? math::wrap_to(s, T(0.), length()) : s;
  }

  /* Calculates pose X_MiM of the frame M at distance ds from the start of the
   i-th segment, relative to frame Mi at the start of the segment.

   That is, X_MiM = X_AMi⁻¹ * X_AM(sᵢ + ds). As X_AM(s) defines the normal of
   the plane as the z axis of M, this pose consists of a circular arc or line
   segment in the x-y plane, and a corresponding z-axis rotation.

   @param rho_i The turning rate of the segment.
   @param ds The length within the segment.
   @retval X_MiM. */
  static math::RigidTransform<T> CalcRelativePoseInSegment(const T& rho_i,
                                                           const T& ds);

  /* Builds the initial pose, X_AM₀, from the given tangent and plane axes,
   expressed in a same reference frame A.

   p_AoMo_A(s₀) is given by `initial_position`, and R_AM₀ is
   calculated as follows: The plane normal p̂ is defined as the z axis Mz; the
   initial heading is the x axis Mx; and the y axis is determined by cross
   product. R_AM is constructed by passing normalized versions of these
   vectors to RotationMatrix::MakeFromOrthonormalColumns, which may fail if
   the provided axes are not unit norm and orthogonal.

   @param initial_curve_tangent The tangent of the curve at arclength s₀ = 0.
   @param plane_normal The normal axis of the plane.
   @param initial_position The initial position of the curve, p_AoMo_A(s₀).

   @pre Norm of initial_curve_tangent is not zero.
   @pre Norm of plane_normal is not zero.
   @pre initial_curve_tangent is orthogonal to plane_normal.

   @returns The initial pose X_AM. */
  static math::RigidTransform<T> MakeInitialPose(
      const Vector3<T>& initial_curve_tangent, const Vector3<T>& plane_normal,
      const Vector3<T>& initial_position);

  /* Calculates pose X_AM at the beginning of each segment.

   For each segment i, the returned vector's i-th element contains the
   relative transform X_AMi = X_AM(sᵢ), for 0 <= i < to breaks.size().

   @param initial_pose The initial pose of the trajectory (at s₀ = 0).
   @param breaks The vector of break points sᵢ between segments.
   @param turning_rates The vector of turning rates ρᵢ for each segment.

   @returns A vector with as many entries as breaks, storing at element i the
   pose X_AMi at the i-th break. */
  static std::vector<math::RigidTransform<T>> MakeBreakPoses(
      const math::RigidTransform<T>& initial_pose, const std::vector<T>& breaks,
      const std::vector<T>& turning_rates);

  /* @returns the initial pose X_AM₀ at s = 0.

   @pre Trajectory must not be empty.
  */
  const math::RigidTransform<T>& get_initial_pose() const {
    return break_poses_[0];
  }

  std::vector<T> segment_turning_rates_;
  bool is_periodic_;
  std::vector<math::RigidTransform<T>> break_poses_;

  static inline constexpr size_t kCurveTangentIndex = 0;
  static inline constexpr size_t kCurveNormalIndex = 1;
  static inline constexpr size_t kPlaneNormalIndex = 2;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewiseConstantCurvatureTrajectory);
