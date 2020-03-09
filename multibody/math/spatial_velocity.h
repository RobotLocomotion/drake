#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_vector.h"

namespace drake {
namespace multibody {

// Forward declaration to define dot product with a spatial force.
template <typename T> class SpatialForce;

// Forward declaration to define dot product with a spatial momentum.
template <typename T> class SpatialMomentum;

/// This class is used to represent a _spatial velocity_ (also called a
/// _twist_) that combines rotational (angular) and translational
/// (linear) velocity components. Spatial velocities are 6-element
/// quantities that are pairs of ordinary 3-vectors. Elements 0-2 are
/// the angular velocity component while elements 3-5 are the translational
/// velocity. Spatial velocities represent the motion of a "moving frame"
/// B measured with respect to a "measured-in" frame A. In addition,
/// the two contained vectors must be expressed in the same "expressed-in"
/// frame E, which may be distinct from either A or B. Finally,
/// while angular velocity is identical for any frame fixed to a rigid
/// body, translational velocity refers to a particular point. Only the
/// vector values are stored in a %SpatialVelocity object; the three
/// frames and the point must be understood from context. It is the
/// responsibility of the user to keep track of them. That is best
/// accomplished through disciplined notation. In source code we use
/// monogram notation where capital V is used to designate a spatial
/// velocity quantity. We write a point P fixed to body (or frame)
/// B as @f$B_P@f$ which appears in code and comments as `Bp`. Then
/// we write a particular spatial velocity as `V_ABp_E` where the `_E`
/// suffix indicates that the expressed-in frame is E. This symbol
/// represents the angular velocity of frame B in frame A, and the
/// translational velocity of point P in A, where P is fixed to
/// frame B, with both vectors expressed in E. Very often
/// the point of interest will be the body origin `Bo`; if no point is
/// shown the origin is understood, so `V_AB_E` means `V_ABo_E`.
/// For a more detailed introduction on spatial vectors and the monogram
/// notation please refer to section @ref multibody_spatial_vectors.
///
/// @tparam_default_scalar
template <typename T>
class SpatialVelocity : public SpatialVector<SpatialVelocity, T> {
  // We need the fully qualified class name below for the clang compiler to
  // work. Without qualifiers the code is legal according to the C++11 standard
  // but the clang compiler still gets confused. See:
  // http://stackoverflow.com/questions/17687459/clang-not-accepting-use-of-template-template-parameter-when-using-crtp
  typedef SpatialVector<::drake::multibody::SpatialVelocity, T> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialVelocity)

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial velocity are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial velocity fail fast,
  /// allowing fast bug detection.
  SpatialVelocity() : Base() {}

  /// SpatialVelocity constructor from an angular velocity @p w and a linear
  /// velocity @p v.
  SpatialVelocity(const Eigen::Ref<const Vector3<T>>& w,
                  const Eigen::Ref<const Vector3<T>>& v) : Base(w, v) {}

  /// SpatialVelocity constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of V is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialVelocity(const Eigen::MatrixBase<Derived>& V) : Base(V) {}

  /// In-place shift of a %SpatialVelocity from one point on a rigid body
  /// or frame to another point on the same body or frame.
  /// `this` spatial velocity `V_ABp_E` of a frame B at a point P fixed
  /// on B, measured in a frame A, and expressed in a frame E, is
  /// modified to become `V_ABq_E`, representing the velocity of another
  /// point Q on B instead (see class comment for more about this
  /// notation). This requires adjusting the translational (linear) velocity
  /// component to account for the velocity difference between P and Q
  /// due to the angular velocity of B in A.
  ///
  /// We are given the vector from point P to point Q, as a position
  /// vector `p_BpBq_E` (or `p_PQ_E`) expressed in the same frame E as the
  /// spatial velocity. The operation performed, in coordinate-free form, is:
  /// <pre>
  ///   w_AB  = w_AB,  i.e. the angular velocity is unchanged.
  ///   v_ABq = v_ABp + w_AB x p_BpBq
  /// </pre>
  /// where w and v represent the angular and linear velocity components
  /// respectively.
  /// Notice this operation is linear. [Jain 2010], (§1.4, page 12) uses the
  /// "rigid body transformation operator" to write this as: <pre>
  ///   V_ABq = Φᵀ(p_BpBq)V_ABp
  /// </pre>
  /// where `Φᵀ(p_PQ)` is the linear operator: <pre>
  ///   Φᵀ(p_PQ) = |  I₃    0  |
  ///              | -p_PQx I₃ |
  /// </pre>
  /// where `p_PQx` denotes the cross product, skew-symmetric, matrix such that
  /// `p_PQx v = p_PQ x v`.
  /// This same operator (not its transpose as for spatial velocities) allow us
  /// to shift spatial forces, see SpatialForce::Shift().
  ///
  /// - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
  ///               algorithms. Springer Science & Business Media, pp. 123-130.
  ///
  /// For computation, all quantities above must be expressed in a common
  /// frame E; we add an `_E` suffix to each symbol to indicate that.
  ///
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] p_BpBq_E
  ///   Shift vector from point P of body B to point Q of B,
  ///   expressed in frame E. The "from" point `Bp` must be the point
  ///   whose velocity is currently represented in this spatial velocity,
  ///   and E must be the same expressed-in frame as for this spatial
  ///   velocity.
  ///
  /// @returns A reference to `this` spatial velocity which is now `V_ABq_E`,
  ///   that is, the spatial velocity of frame B at point Q, still
  ///   measured in frame A and expressed in frame E.
  ///
  /// @see Shift() to compute the shifted spatial velocity without modifying
  ///      this original object.
  SpatialVelocity<T>& ShiftInPlace(const Vector3<T>& p_BpBq_E) {
    this->translational() += this->rotational().cross(p_BpBq_E);
    return *this;
  }

  /// Shift of a %SpatialVelocity from one point on a rigid body
  /// or frame to another point on the same body or frame.
  /// This is an alternate signature for shifting a spatial velocity's
  /// point that does not change the original object. See
  /// ShiftInPlace() for more information.
  ///
  /// @param[in] p_BpBq_E
  ///   Shift vector from point P of body B to point Q of B,
  ///   expressed in frame E. The "from" point `Bp` must be the point
  ///   whose velocity is currently represented in this spatial velocity,
  ///   and E must be the same expressed-in frame as for this spatial
  ///   velocity.
  ///
  /// @retval V_ABq_E
  ///   The spatial velocity of frame B at point Q, measured in frame
  ///   A and expressed in frame E.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial velocity in-place
  ///      modifying the original object.
  SpatialVelocity<T> Shift(const Vector3<T>& p_BpBq_E) const {
    return SpatialVelocity<T>(*this).ShiftInPlace(p_BpBq_E);
  }

  /// This method composes `this` spatial velocity `V_WP` of a frame P measured
  /// in a frame W, with that of a third frame B moving in P with spatial
  /// velocity `V_PB`. The result is the spatial velocity `V_WB` of frame B
  /// measured in W. At the instant in which the velocities are composed, frame
  /// B is located with its origin `Bo` at `p_PoBo` from P's origin Po.
  ///
  /// The composition cannot be performed directly since frames P and B do not
  /// have the same origins. To perform the composition `V_WB`, the velocity of
  /// P needs to be shifted to point `Bo`: <pre>
  ///   V_WB_E = V_WPb_E + V_PB_E = V_WP_E.Shift(p_PoBo_E) + V_PB_E
  /// </pre>
  /// where p_PoBo is the position vector from P's origin to B's origin and
  /// `V_WPb` is the spatial velocity of a new frame `Pb` which is an offset
  /// frame rigidly aligned with P, but with its origin shifted to B's origin.
  /// The key is that in the expression above, the two spatial velocities being
  /// added must be for frames with the same origin point, in this case Bo.
  ///
  /// For computation, all quantities above must be expressed in a common
  /// frame E; we add an `_E` suffix to each symbol to indicate that.
  ///
  /// @note If frame B moves rigidly together with frame P, as in a rigid body,
  /// `V_PB = 0` and the result of this method equals that of the Shift()
  /// operation.
  ///
  /// @param[in] p_PoBo_E
  ///   Shift vector from P's origin to B's origin, expressed in frame E.
  ///   The "from" point `Po` must be the point whose velocity is currently
  ///   represented in `this` spatial velocity, and E must be the same
  ///   expressed-in frame as for `this` spatial velocity.
  /// @param[in] V_PB_E
  ///   The spatial velocity of a third frame B in motion with respect to P,
  ///   expressed in the same frame E as `this` spatial velocity.
  /// @retval V_WB_E
  ///   The spatial velocity of frame B in W resulting from the composition of
  ///   `this` spatial velocity `V_WP` and B's velocity in P, `V_PB`. The result
  ///   is expressed in the same frame E as `this` spatial velocity.
  SpatialVelocity<T> ComposeWithMovingFrameVelocity(
      const Vector3<T>& p_PoBo_E, const SpatialVelocity<T>& V_PB_E) const {
    // V_WB_E = V_WPb_E + V_PB_E = V_WP_E.Shift(p_PoBo_E) + V_PB_E
    return this->Shift(p_PoBo_E) + V_PB_E;
  }

  /// Given `this` spatial velocity `V_IBp_E` of point P of body B,
  /// measured in an inertial frame I and expressed in a frame E,
  /// this method computes the 6-dimensional dot product with the spatial
  /// force `F_Bp_E` applied to point P, and expressed in the same
  /// frame E in which the spatial velocity is expressed.
  /// This dot-product represents the power generated by the spatial force
  /// when its body and application point have `this` spatial velocity.
  /// Although the two spatial vectors must be expressed in the same frame,
  /// the result is independent of that frame.
  ///
  /// @warning The result of this method cannot be interpreted as power unless
  ///          `this` spatial velocity is measured in an inertial frame I,
  ///          which cannot be enforced by this class.
  T dot(const SpatialForce<T>& F_Q_E) const;

  /// Given `this` spatial velocity `V_NBp_E` of rigid body B frame shifted to
  /// point P, measured in an inertial (or Newtonian) frame N and, expressed in
  /// a frame E this method computes the dot product with the spatial momentum
  /// `L_NBp_E` of rigid body B, about point P, and expressed in the same
  /// frame E.
  /// This dot-product is twice the kinetic energy `ke_NB` of body B in
  /// reference frame N. The kinetic energy `ke_NB` is independent of the
  /// about-point P and so is this dot product. Therefore it is always true
  /// that: <pre>
  ///   ke_NB = 1/2 (L_NBp⋅V_NBp) = 1/2 (L_NBcm⋅V_NBcm)
  /// </pre>
  /// where `L_NBcm` is the spatial momentum about the center of mass of body B
  /// and `V_NBcm` is the spatial velocity of frame B shifted to its center of
  /// mass. The above is true due to how spatial momentum and velocity shift
  /// when changing point P, see SpatialMomentum::Shift() and
  /// SpatialVelocity::Shift().
  T dot(const SpatialMomentum<T>& L_NBp_E) const;
};

/// Performs the addition of two spatial velocities. This operator
/// returns the spatial velocity that results from adding the operands as if
/// they were 6-dimensional vectors. In other words, the resulting spatial
/// velocity contains a rotational component which is the 3-dimensional
/// addition of the operand's rotational components and a translational
/// component which is the 3-dimensional addition of the operand's translational
/// components.
///
/// The addition of two spatial velocities has a clear physical meaning but
/// can only be performed if the operands meet strict conditions. In addition
/// the the usual requirement of common expressed-in frames, both spatial
/// velocities must be for frames with the same origin point. The general idea
/// is that if frame A has a spatial velocity with respect to M, and frame B
/// has a spatial velocity with respect to A, we want to "compose" them so that
/// we get frame B's spatial velocity in M. But that can't be done directly
/// since frames A and B don't have the same origin. So:
///
/// Given the velocity V_MA_E of a frame A in a measured-in frame M, and
/// the velocity V_AB_E of a frame B measured in frame A (both
/// expressed in a common frame E), we can calculate V_MB_E as their sum after
/// shifting A's velocity to point Bo: <pre>
///   V_MB_E = V_MA_E.Shift(p_AB_E) + V_AB_E
/// </pre>
/// where `p_AB_E` is the position vector from A's origin to B's origin,
/// expressed in E. This shift can also be thought of as yielding the spatial
/// velocity of a new frame Ab, which is an offset frame rigidly aligned with A,
/// but with its origin shifted to B's origin: <pre>
///   V_MAb_E = V_MA_E.Shift(p_AB_E)
///   V_MB_E = V_MAb_E + V_AB_E
/// </pre>
///
/// The addition in the last expression is what is carried out by this operator;
/// the caller must have already performed the necessary shift.
///
/// @relates SpatialVelocity
template <typename T>
inline SpatialVelocity<T> operator+(
    const SpatialVelocity<T>& V_MAb_E, const SpatialVelocity<T>& V_AB_E) {
  // N.B. We use SpatialVector's implementation, though we provide the overload
  // for specific documentation purposes.
  return SpatialVelocity<T>(V_MAb_E) += V_AB_E;
}

/// The addition of two spatial velocities relates to the composition of
/// the spatial velocities for two frames given we know the relative spatial
/// velocity between them, see
/// operator+(const SpatialVelocity<T>&, const SpatialVelocity<T>&) for
/// further details.
///
/// Mathematically, operator-(v1, v2) is equivalent ot operator+(v1, -v2).
///
/// Physically, the subtraction operation allow us to compute the relative
/// velocity between two frames. As an example, consider having the the spatial
/// velocities `V_MA` and `V_MB` of two frames A and B respectively measured in
/// the same frame M. The velocity of B in A can be obtained as: <pre>
///   V_AB_E = V_MB_E - V_MAb_E = V_AB_E = V_MB_E - V_MA_E.Shift(p_AB_E)
/// </pre>
/// where we have expressed all quantities in a common frame E. Notice that,
/// as explained in the documentation for
/// operator+(const SpatialVelocity<T>&, const SpatialVelocity<T>&) a shift
/// operation with SpatialVelocity::Shift() operation is needed.
///
/// @relates SpatialVelocity
template <typename T>
inline SpatialVelocity<T> operator-(
    const SpatialVelocity<T>& V_MB_E, const SpatialVelocity<T>& V_MAb_E) {
  // N.B. We use SpatialVector's implementation, though we provide the overload
  // for specific documentation purposes.
  return SpatialVelocity<T>(V_MB_E) -= V_MAb_E;
}

}  // namespace multibody
}  // namespace drake
