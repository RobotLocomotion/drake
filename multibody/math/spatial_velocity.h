#pragma once

#ifndef DRAKE_SPATIAL_ALGEBRA_HEADER
#error Please include "drake/multibody/math/spatial_algebra.h", not this file.
#endif

#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/math/spatial_momentum.h"
#include "drake/multibody/math/spatial_vector.h"

namespace drake {
namespace multibody {

/// This class represents a _spatial velocity_ (also called a _twist_) and has
/// 6 elements with a rotational (angular) velocity w (ordinary 3-vector) on top
/// of a translational (linear) velocity v (ordinary 3-vector).
/// A spatial velocity represents the rotational and translational motion of a
/// frame B with respect to a "measured-in" frame A. This class assumes that
/// both the rotational velocity w and translational velocity v are expressed
/// in the same "expressed-in" frame E.
/// Reminder: Translational velocity v is associated with a single point of
/// frame B, e.g., B's origin (Bo) or if B is a body, its center of mass Bcm.
/// This class only stores 6 elements (namely w and v) and does not store the
/// underlying frames B, A, E or the point of B associated with v.  The user is
/// responsible for keeping track of the underlying frames B, A, E, etc., which
/// is best done through monogram notation.  For a point Bp that is fixed to a
/// frame (or body) B, the spatial velocity of point Bp of frame B measured in
/// frame A, expressed in frame E is denoted `V_ABp_E`. `V_ABp_E` contains
/// w_AB_E (B's rotational velocity w measured in A, expressed in E) and
/// v_ABp_E (point Bp's translational velocity v measured in A, expressed in E).
/// The abbreviated monogram notation V_AB_E denotes the spatial velocity of
/// frame B's origin (Bo) measured in A, expressed in E.  Details on spatial
/// vectors and monogram notation are in section @ref multibody_spatial_vectors.
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

  /// Default constructor. In Release builds the elements of a newly constructed
  /// spatial velocity are uninitialized (for speed). In Debug builds the
  /// elements are set to NaN so that invalid operations on an uninitialized
  /// spatial velocity fail fast, allowing fast bug detection.
  SpatialVelocity() : Base() {}

  /// SpatialVelocity constructor from an angular velocity @p w and a
  /// translational velocity @p v.
  SpatialVelocity(const Eigen::Ref<const Vector3<T>>& w,
                  const Eigen::Ref<const Vector3<T>>& v) : Base(w, v) {}

  /// SpatialVelocity constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of V is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialVelocity(const Eigen::MatrixBase<Derived>& V) : Base(V) {}

  /// In-place shift of a %SpatialVelocity from one point Bp of a rigid body or
  /// frame B to another point Bq of B (both Bp and Bq are fixed to B).
  /// On entry, `this` is the spatial velocity `V_ABp_E` of frame B at Bp,
  /// measured in a frame A, and expressed in a frame E.  On return `this` is
  /// modified to be `V_ABq_E`, the spatial velocity of frame B at Bq,
  /// measured in frame A, and expressed in frame E.
  /// @param[in] p_BpBq_E position vector from point Bp of B to point Bq of B,
  ///   expressed in frame E. Point Bp is the point whose translational velocity
  ///   is stored in `this` spatial velocity on entry.  p_BpBq_E must have the
  ///   same expressed-in frame E as `this` spatial velocity.
  /// @retval V_ABq_E reference to `this` spatial velocity which has been
  ///   modified to represent the spatial velocity of frame B at point Bq,
  ///   still measured in frame A and expressed in frame E. This is calculated:
  /// <pre>
  ///  w_AB_E  = w_AB_E               (angular velocity of `this` is unchanged).
  ///  v_ABq_E = v_ABp_E + w_AB_E x p_BpBq_E   (translational velocity changes).
  /// </pre>
  /// @see Shift() to shift spatial velocity without modifying `this`.
  SpatialVelocity<T>& ShiftInPlace(const Vector3<T>& p_BpBq_E) {
    this->translational() += this->rotational().cross(p_BpBq_E);
    return *this;
    // Note: this operation is linear. [Jain 2010], (§1.4, page 12) uses the
    // "rigid body transformation operator" to write this as:
    //   V_ABq = Φᵀ(p_BpBq)V_ABp  where `Φᵀ(p_PQ)` is the linear operator:
    //   Φᵀ(p_PQ) = |  I₃    0  |
    //              | -p_PQx I₃ |
    // where `p_PQx` denotes the cross product, skew-symmetric, matrix such that
    // `p_PQx v = p_PQ x v`.
    // This same operator (not its transpose as for spatial velocities) allow us
    // to shift spatial forces, see SpatialForce::Shift().
    //
    // - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
    //               algorithms. Springer Science & Business Media, pp. 123-130.
  }

  /// Shift a %SpatialVelocity from one point Bp of a rigid body or
  /// frame B to another point Bq of B (both Bp and Bq are fixed to B).
  /// This method differs from ShiftInPlace() in that this method does not
  /// modify `this` whereas ShiftInPlace() does modify `this`.
  /// @param[in] p_BpBq_E position vector from point Bp of B to point Bq of B,
  ///   expressed in frame E. Point Bp is the point whose translational velocity
  ///   is stored in `this` spatial velocity.  p_BpBq_E must have the same
  ///   expressed-in frame E as `this` spatial velocity.
  /// @retval V_ABq_E spatial velocity of frame B at point Bq,
  ///   measured in frame A, and expressed in frame E.
  /// @see ShiftInPlace() for more information.
  SpatialVelocity<T> Shift(const Vector3<T>& p_BpBq_E) const {
    return SpatialVelocity<T>(*this).ShiftInPlace(p_BpBq_E);
  }

  /// This method implements the formula to calculate the spatial velocity of a
  /// frame B at point Bq that is moving on a frame A, measured in a frame W.
  /// @param[in] p_ApBq_E position vector from a point Ap of frame A to a point
  ///   Bq of frame B, expressed in a frame E.  Point Ap is the point whose
  ///   translational velocity is stored in `this` spatial velocity. p_ApBq_E
  ///   must have the same expressed-in frame E as `this` spatial velocity.
  /// @param[in] V_ABq_E spatial velocity of frame B at point Bq measured in
  ///   frame A, expressed in the same frame E as `this` spatial velocity.
  /// @retval V_WBq_E spatial velocity of frame B at point Bq,
  ///   measured in frame W and expressed in frame E.
  /// @note The angular velocity component w and translational velocity
  ///   component w of the returned spatial velocity V_WBq_E are calculated:
  /// <pre>
  ///  w_WB_E  = w_WA_E + w_AB_E
  ///  v_WBq_E = v_WAp_E + w_WA_E x p_ApBq_E + v_ABq_E
  /// </pre>
  /// If frame B is rigidly fixed to frame A (as in a rigid body), V_ABq_E = 0
  /// and this method produces a Shift() operation (albeit inefficiently).
  SpatialVelocity<T> ComposeWithMovingFrameVelocity(
      const Vector3<T>& p_ApBq_E, const SpatialVelocity<T>& V_ABq_E) const {
    // V_WBq_E = V_WAp_E.Shift(p_ApBq_E) + V_ABq_E
    return this->Shift(p_ApBq_E) + V_ABq_E;
  }

  /// Calculates the 6-element dot-product of `this` spatial velocity with a
  /// spatial force. Denoting `this` as the spatial velocity V_WBp_E of frame B
  /// at point Bp, measured in a frame W, and expressed in a frame E, V_WBp_E is
  /// dot-multiplied with the spatial force F_Bp_E. The sum of the resulting
  /// 6-elements is the power generated by the spatial force in frame W.
  /// @param[in] F_Bp_E spatial force on frame B at point Bp, expressed in the
  ///   same frame E as `this` spatial velocity V_WBp_E.
  /// @retval 6-element representation of the power of spatial force F_Bp_E in
  ///   frame W, equal to F_Bp_E ⋅ V_WBp_E.
  /// @note Just as equating force 𝐅 to mass times acceleration as 𝐅 = m𝐚 relies
  ///   on acceleration 𝐚 being measured in a world frame W (also called a
  ///   Newtonian or inertial frame), equating power = dK/dt (where K is
  ///   kinetic energy) relies on K being measured in a world frame W.
  /// @note Although the spatial vectors F_Bp_E and V_WBp_E must have the same
  ///   expressed-in frame E, the result is independent of that frame.
  inline T dot(const SpatialForce<T>& F_Q_E) const;
  // The dot() method is implemented at the end of this file, so that all of
  // the dot methods are co-located for easy understanding. We need the inline
  // keyword to ensure the method is still inlined even with `extern template`.

  /// Given `this` spatial velocity `V_NBp_E` of rigid body B frame shifted to
  /// point P, measured in an inertial (or Newtonian) frame N and, expressed in
  /// a frame E this method computes the dot product with the spatial momentum
  /// `L_NBp_E` of rigid body B, about point P, and expressed in the same
  /// frame E.
  /// This dot-product is twice the kinetic energy `ke_NB` of body B in
  /// reference frame N. The kinetic energy `ke_NB` is independent of the
  /// about-point P and so is this dot product. Therefore it is always true
  /// that: <pre>
  ///   ke_NB = 1/2 (L_NBp ⋅ V_NBp) = 1/2 (L_NBcm ⋅ V_NBcm)
  /// </pre>
  /// where `L_NBcm` is the spatial momentum about the center of mass of body B
  /// and `V_NBcm` is the spatial velocity of frame B shifted to its center of
  /// mass. The above is true due to how spatial momentum and velocity shift
  /// when changing point P, see SpatialMomentum::Shift() and
  /// SpatialVelocity::Shift().
  inline T dot(const SpatialMomentum<T>& L_NBp_E) const;
  // The dot() method is implemented at the end of this file, so that all of
  // the dot methods are co-located for easy understanding. We need the inline
  // keyword to ensure the method is still inlined even with `extern template`.
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
/// the usual requirement of common expressed-in frames, both spatial
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
/// velocity between two frames. As an example, consider having the spatial
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

template <typename T>
T SpatialVelocity<T>::dot(const SpatialForce<T>& F_Q_E) const {
  return this->get_coeffs().dot(F_Q_E.get_coeffs());
}

// This was declared in spatial_force.h, but must be implemented here in
// order to break the dependency cycle.
template <typename T>
T SpatialForce<T>::dot(const SpatialVelocity<T>& V_IBp_E) const {
  return V_IBp_E.dot(*this);  // dot-product is commutative.
}

// This was declared in spatial_momentum.h, but must be implemented here in
// order to break the dependency cycle.
template <typename T>
T SpatialMomentum<T>::dot(const SpatialVelocity<T>& V_NBp_E) const {
  return this->get_coeffs().dot(V_NBp_E.get_coeffs());
}

template <typename T>
T SpatialVelocity<T>::dot(const SpatialMomentum<T>& L_NBp_E) const {
  return L_NBp_E.dot(*this);  // dot-product is commutative.
}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::SpatialVelocity)
