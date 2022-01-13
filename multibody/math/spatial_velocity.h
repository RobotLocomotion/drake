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

  /// Calculates the dot-product of `this` spatial velocity with a spatial
  /// force.  Denoting `this` as the spatial velocity V_WBp_E of frame B at
  /// point Bp, measured in a frame W, and expressed in a frame E, V_WBp_E is
  /// dot-multiplied with the spatial force F_Bp_E. The resulting scalar is
  /// the power generated by the spatial force in frame W.
  /// @param[in] F_Bp_E spatial force on frame B at point Bp, expressed in
  ///   the same frame E as `this` spatial velocity V_WBp_E.
  /// @retval Power of spatial force F_Bp_E in frame W, i.e., F_Bp_E ⋅ V_WBp_E.
  /// @note Just as equating force 𝐅 to mass * acceleration as 𝐅 = m𝐚 relies
  ///   on acceleration 𝐚 being measured in a world frame W (also called a
  ///   Newtonian or inertial frame), equating power = dK/dt (where K is
  ///   kinetic energy) relies on K being measured in a world frame W.
  ///   It is unusual to use this method unless frame W is the world frame.
  /// @note Although the spatial vectors F_Bp_E and V_WBp_E must have the same
  ///   expressed-in frame E, the result is a scalar (independent of frame E).
  inline T dot(const SpatialForce<T>& F_Bp_E) const;
  // The dot() method is implemented at the end of this file, so that all of
  // the dot methods are co-located for easy understanding. We need the inline
  // keyword to ensure the method is still inlined even with `extern template`.

  /// Calculates the dot-product of `this` spatial velocity with a spatial
  /// momentum. Denoting `this` as the spatial velocity V_WBp_E of rigid body B
  /// at point Bp, measured in a frame W, and expressed in a frame E, this
  /// method forms twice the kinetic energy of body B in frame W via <pre>
  ///   2 * ke_WB = L_WBp_E ⋅ V_WBp_E = L_WBcm_E ⋅ V_WBcm_E
  /// </pre>
  /// where L_WBp_E is the spatial momentum of body B at point Bp, measured in
  /// frame W.  Note: body B's kinetic energy in frame W can be calculated
  /// using Bp (an arbitrary point fixed to B) or Bcm (B's center of mass).
  /// This non-obvious fact can be seen through the shift operators for spatial
  /// momentum and spatial velocity when changing from point Bcm to Bp. For more
  /// information, see SpatialMomentum::Shift() and SpatialVelocity::Shift().
  /// @param[in] L_WBp_E spatial momentum of body B at Bp, measured-in frame W,
  ///   expressed in the same frame E as `this` spatial velocity V_WBp_E.
  /// @returns Twice the kinetic energy of body B in frame W.
  /// @note In most situations, kinetic energy calculations are only useful when
  ///   frame W is a world frame (also called a Newtonian or inertial frame).
  ///   It is unusual to use this method unless frame W is the world frame.
  /// @note Although the spatial vectors V_WBp_E and L_WBp_E must have the same
  ///   expressed-in frame E, the result is a scalar (independent of frame E).
  inline T dot(const SpatialMomentum<T>& L_WBp_E) const;
  // The dot() method is implemented at the end of this file, so that all of
  // the dot methods are co-located for easy understanding. We need the inline
  // keyword to ensure the method is still inlined even with `extern template`.
};

/// Adds two spatial velocities by simply adding their 6 underlying elements.
/// @param[in] V1_E spatial velocity of an arbitrary frame A, measured-in an
///   another arbitrary frame B, expressed in the same frame E as V2_E.
/// @param[in] V2_E spatial velocity of an arbitrary frame C, measured-in an
///   another arbitrary frame D, expressed in the same frame E as V1_E.
/// @note The general utility of this method is questionable and this method
///   should only be used if you are sure it makes sense.  One use case is for
///   calculating the spatial velocity V_WBp of a frame B at point Bp
///   measured-in frame W when frame B is moving on a frame A and one has
///   pre-calculated V_WAp (the spatial velocity measured-in frame W of frame A
///   at point Ap, where Ap is the point of frame A that is coincident with Bp).
///   For this use case, this method returns V_WBp_E = V_WAp_E + V_ABp_E, where
///   the precalculated V_WAp_E is equal to V_WAo_E.Shift(p_AoAp_E).
/// @see related methods ShiftInPlace() and ComposeWithMovingFrameVelocity().
template <typename T>
inline SpatialVelocity<T> operator+(
    const SpatialVelocity<T>& V1_E, const SpatialVelocity<T>& V2_E) {
  // Although this method is implemented by simply calling the corresponding
  // SpatialVector implementation, it is needed for documentation.
  return SpatialVelocity<T>(V1_E) += V2_E;
}

/// Subtracts spatial velocities by subtracting their 6 underlying elements.
/// @param[in] V1_E spatial velocity of an arbitrary frame A, measured-in an
///   another arbitrary frame B, expressed in the same frame E as V2_E.
/// @param[in] V2_E spatial velocity of an arbitrary frame C, measured-in an
///   another arbitrary frame D, expressed in the same frame E as V1_E.
/// @note This method should only be used if you are sure it makes sense.
///   One use case is calculating relative spatial velocity, e.g., the spatial
///   velocity of a frame B at point Bp measured-in a frame W relative to the
///   spatial velocity of a frame A at point Ap measure in frame W.
///   For this use case, this method returns V_W_ApBp_E = V_WBp_E - V_WAp_E.
///
///   A second use case calculates the spatial velocity V_ABp of a frame B at
///   point Bp measured-in frame A when frame B is moving on a frame A and one
///   has pre-calculated V_WAp (the spatial velocity measured-in frame W of
///   frame A at point Ap, where Ap is the point of frame A that is coincident
///   with Bp. This use case returns V_ABp_E = V_WBp_E - V_WAp_E, where the
///   precalculated V_WAp_E is equal to V_WAo_E.Shift(p_AoAp_E).
/// @see related methods ShiftInPlace() and ComposeWithMovingFrameVelocity().
template <typename T>
inline SpatialVelocity<T> operator-(
    const SpatialVelocity<T>& V1_E, const SpatialVelocity<T>& V2_E) {
  // Although this method is implemented by simply calling the corresponding
  // SpatialVector implementation, it is needed for documentation.
  return SpatialVelocity<T>(V1_E) -= V2_E;
}

template <typename T>
T SpatialVelocity<T>::dot(const SpatialForce<T>& F_Bp_E) const {
  return this->get_coeffs().dot(F_Bp_E.get_coeffs());
}

// This was declared in spatial_force.h, but must be implemented here in
// order to break the dependency cycle.
template <typename T>
T SpatialForce<T>::dot(const SpatialVelocity<T>& V_WBp_E) const {
  return V_WBp_E.dot(*this);  // dot-product is commutative.
}

// This was declared in spatial_momentum.h, but must be implemented here in
// order to break the dependency cycle.
template <typename T>
T SpatialMomentum<T>::dot(const SpatialVelocity<T>& V_WBp_E) const {
  return this->get_coeffs().dot(V_WBp_E.get_coeffs());
}

template <typename T>
T SpatialVelocity<T>::dot(const SpatialMomentum<T>& L_WBp_E) const {
  return L_WBp_E.dot(*this);  // dot-product is commutative.
}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::SpatialVelocity)
