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
/// 6 elements with an angular (rotational) velocity ω (3-element vector) on top
/// of a translational (linear) velocity v (3-element vector).
/// A spatial velocity represents the rotational and translational motion of a
/// frame C with respect to a "measured-in" frame B. This class assumes that
/// both the angular velocity ω and translational velocity v are expressed
/// in the same "expressed-in" frame E.  This class only stores 6 elements
/// (namely ω and v) and does not store the underlying frames B, C, E. The user
/// is responsible for keeping track of the underlying frames B, C, E,
/// which is best done through @ref multibody_quantities "monogram notation".
/// %Frame C's spatial velocity measured in a frame B, expressed in a frame E is
/// denoted V_BC_E and contains ω_BC_E (C's angular velocity measured in B,
/// expressed in E) and v_BCo_E (Co's translational velocity measured in B,
/// expressed in E), where Co is frame C's origin point.
/// For an @ref multibody_frames_and_bodies "offset frame" Cp, the monogram
/// notation V_BCp_E denotes the spatial velocity of frame Cp measured in B,
/// expressed in E.  Details on spatial vectors and monogram notation are
/// in section @ref multibody_spatial_vectors.
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

  /// Default constructor. In Release builds, all 6 elements of a newly
  /// constructed spatial velocity are uninitialized (for speed).  In Debug
  /// builds, the 6 elements are set to NaN so that invalid operations on an
  /// uninitialized spatial velocity fail fast (fast bug detection).
  SpatialVelocity() : Base() {}

  /// SpatialVelocity constructor from an angular velocity @p ω and a
  /// translational velocity @p v.
  SpatialVelocity(const Eigen::Ref<const Vector3<T>>& w,
                  const Eigen::Ref<const Vector3<T>>& v) : Base(w, v) {}

  /// SpatialVelocity constructor from an Eigen expression that represents a
  /// six-dimensional vector, i.e., two 3-element vectors, namely an angular
  /// velocity ω and a translational velocity v.  This constructor will assert
  /// the size of V is six (6) either at compile-time for fixed sized Eigen
  /// expressions or at run-time for dynamic sized Eigen expressions.
  template <typename Derived>
  explicit SpatialVelocity(const Eigen::MatrixBase<Derived>& V) : Base(V) {}

  /// In-place shift of a %SpatialVelocity from a frame P to a frame Q, where
  /// both P and Q are fixed to the same rigid body or frame C. On entry, `this`
  /// is V_BP_E (frame P's spatial velocity measured in a frame B and expressed
  /// in a frame E). On return `this` is modified to V_BQ_E (frame Q's spatial
  /// velocity measured in frame B and expressed in frame E).
  /// @param[in] offset, which is the position vector p_PoQo_E from frame P's
  /// origin to frame Q's origin, expressed in frame E. p_PoQo_E must have the
  /// same expressed-in frame E as `this` spatial velocity.
  /// @retval V_BQ_E reference to `this` spatial velocity which has been
  /// modified to be frame Q's spatial velocity measured in frame B and
  /// expressed in frame E. The components of V_BQ_E are calculated as: <pre>
  ///  ω_BC_E  = ω_BC_E               (angular velocity of `this` is unchanged).
  ///  v_BQ_E = v_BP_E + ω_BC_E x p_PoQo_E     (translational velocity changes).
  /// </pre>
  /// @see Shift() to shift spatial velocity without modifying `this`.
  SpatialVelocity<T>& ShiftInPlace(const Vector3<T>& offset) {
    this->translational() += this->rotational().cross(offset);
    return *this;
    // Note: this operation is linear. [Jain 2010], (§1.4, page 12) uses the
    // "rigid body transformation operator" to write this as:
    //   V_BQ = Φᵀ(p_PoQo) V_BP  where `Φᵀ(p_PoQo)` is the linear operator:
    //   Φᵀ(p_PoQo) = |  I₃       0  |
    //                | -px_PoQo  I₃ |
    // where `px_PoQo` denotes the cross product skew-symmetric matrix such that
    // `px_PoQo vec = p_PoQo x vec` (where vec is any vector).
    // This same operator (not its transpose as for spatial velocities) allows
    // us to shift spatial forces, see SpatialForce::Shift().
    //
    // - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
    //               algorithms. Springer Science & Business Media, pp. 123-130.
  }

  /// Shifts a %SpatialVelocity from a frame P to a frame Q, where both P and Q
  /// are fixed to the same rigid body or frame C.
  /// This method differs from ShiftInPlace() in that this method does not
  /// modify `this` whereas ShiftInPlace() does modify `this`.
  /// @param[in] offset, which is the position vector p_PoQo_E from frame P's
  /// origin to frame Q's origin, expressed in frame E. p_PoQo_E must have the
  /// same expressed-in frame E as `this` spatial velocity (`this` = V_BP_E).
  /// @retval V_BQ_E which is frame Q's spatial velocity measured in frame B
  /// and expressed in frame E.
  /// @see ShiftInPlace() for more information and how V_BQ_E is calculated.
  SpatialVelocity<T> Shift(const Vector3<T>& offset) const {
    return SpatialVelocity<T>(*this).ShiftInPlace(offset);
  }

  /// For a frame C that moves relative to a frame B, calculates V_WC_E (frame
  /// C's spatial velocity measured in a frame W, expressed in a frame E).
  /// @param[in] position_of_moving_frame which is the position vector p_BoCo_E
  /// from Bo (frame B's origin) to Co (frame C's origin), expressed in frame E.
  /// p_BoCo_E must have the same expressed-in frame E as `this` spatial
  /// velocity (`this` = V_WB_E, i.e., frame B's spatial velocity measured in
  /// rame W, expressed in frame E).
  /// @param[in] velocity_of_moving_frame which is V_BC_E, frame C's spatial
  /// velocity measured in frame B, expressed in the same frame E as `this`
  /// spatial velocity (`this` = V_WB_E).
  /// @retval V_WC_E frame C's spatial velocity measured in frame W and
  /// expressed in frame E.
  /// @note The returned spatial velocity V_WC_E contains an angular velocity
  /// ω_wC_E and translational velocity v_WCo_E that are calculated as: <pre>
  ///  ω_WC_E  = ω_WB_E + ω_BC_E
  ///  v_WCo_E = v_WBo_E + ω_WB_E x p_BoCo_E + v_BCo_E
  /// </pre>
  /// If frame C is rigidly fixed to frame B, V_BC_E = 0 and this method
  /// produces a Shift() operation (albeit inefficiently).
  SpatialVelocity<T> ComposeWithMovingFrameVelocity(
      const Vector3<T>& position_of_moving_frame,
      const SpatialVelocity<T>& velocity_of_moving_frame) const {
    // V_WC_E = V_WB_E.Shift(p_BoCo_E) + V_BC_E
    return this->Shift(/*p_BoCo_E = */ position_of_moving_frame)
                      + /* V_BC_E = */ velocity_of_moving_frame;
  }

  /// For an arbitrary frame B, calculates the dot-product of V_WB_E (frame B's
  /// spatial velocity measured in frame W, expressed-in frame E, stored in
  /// `this`) with F_B_E (frame B's spatial force, expressed-in frame E). The
  /// resulting scalar is the power generated by the spatial force in frame W.
  /// @param[in] force which is F_B_E frame B's spatial force, expressed in the
  /// same frame E as `this` spatial velocity V_WB_E.
  /// @returns Power of spatial force F_B_E in frame W, i.e., F_B_E ⋅ V_WB_E.
  /// @note Just as equating force 𝐅 to mass * acceleration as 𝐅 = m𝐚 relies
  /// on acceleration 𝐚 being measured in a world frame W (also called a
  /// Newtonian or inertial frame), equating power = dK/dt (where K is kinetic
  /// energy) relies on K being measured in a world frame W.  Hence, it is
  /// unusual to use this method unless frame W is the world frame.
  /// @note Although the spatial vectors F_B_E and V_WB_E must have the same
  /// expressed-in frame E, the returned scalar is independent of frame E.
  inline T dot(const SpatialForce<T>& force) const;
  // The dot() method is implemented at the end of this file, so that all of
  // the dot methods are co-located for easy understanding. We need the inline
  // keyword to ensure the method is still inlined even with `extern template`.

  /// For a frame Bp that is fixed to a rigid body B, uses V_WBp_E (frame Bp's
  /// spatial velocity measured in a frame W, expressed in a frame E, which is
  /// stored in `this`) to calculate twice (2x) body B's kinetic energy measured
  /// in frame W as: <pre>
  ///   K = 1/2 (L_WBp · V_WBp) = 1/2 (L_WBcm · V_WBcm)
  /// </pre>
  /// where L_WBp_E is body B's spatial momentum measured in frame W, about
  /// frame Bp's origin, and expressed in the same frame E as V_WBp_E.
  /// As shown above, kinetic energy K can be calculated from an arbitrary frame
  /// Bp fixed on B (which may be body B's center of mass frame Bcm). This fact
  /// is due to how spatial momentum and spatial velocity shift when changing
  /// from Bcm to Bp. For more information, see
  /// SpatialMomentum::Shift() and SpatialVelocity::Shift().
  /// @param[in] momentum which is L_WBp_E body B's spatial momentum measured-in
  /// frame W, about frame Bp's origin, expressed in the same frame E as `this`
  /// spatial velocity V_WBp_E.
  /// @returns twice (2x) body B's kinetic energy in frame W.
  /// @note In most situations, kinetic energy calculations are only useful when
  /// frame W is a world frame (also called a Newtonian or inertial frame).
  /// Hence, it is unusual to use this method unless frame W is the world frame.
  /// @note Although the spatial vectors V_WBp_E and L_WBp_E must have the same
  /// expressed-in frame E, the resulting scalar is independent of frame E.
  inline T dot(const SpatialMomentum<T>& momentum) const;
  // The dot() method is implemented at the end of this file, so that all of
  // the dot methods are co-located for easy understanding. We need the inline
  // keyword to ensure the method is still inlined even with `extern template`.
};

/// Adds two spatial velocities by simply adding their 6 underlying elements.
/// @param[in] V1_E spatial velocity of an arbitrary frame P, measured-in an
/// another arbitrary frame Q, expressed in the same frame E as V2_E.
/// @param[in] V2_E spatial velocity of an arbitrary frame R, measured-in an
/// another arbitrary frame S, expressed in the same frame E as V1_E.
/// @note The general utility of this method is questionable and this method
/// should only be used if you are sure it makes sense.  One use case is for
/// calculating the spatial velocity V_WCp of a frame C measured-in a frame W
/// when frame C is moving relative to a frame B and one has pre-calculated
/// V_WBc (frame Bc's spatial velocity measured-in frame W, where frame Bc is
/// instantaneously coincident with frame C).
/// For this use case, this method returns V_WC_E = V_WBc_E + V_BC_E, where
/// the precalculated V_WBc_E is equal to V_WBo_E.Shift(p_BoCo_E).
/// @see related methods ShiftInPlace() and ComposeWithMovingFrameVelocity().
template <typename T>
inline SpatialVelocity<T> operator+(
    const SpatialVelocity<T>& V1_E, const SpatialVelocity<T>& V2_E) {
  // Although this method is implemented by simply calling the corresponding
  // SpatialVector implementation, it is needed for documentation.
  return SpatialVelocity<T>(V1_E) += V2_E;
}

/// Subtracts spatial velocities by subtracting their 6 underlying elements.
/// @param[in] V1_E spatial velocity of an arbitrary frame P, measured-in an
/// another arbitrary frame Q, expressed in the same frame E as V2_E.
/// @param[in] V2_E spatial velocity of an arbitrary frame R, measured-in an
/// another arbitrary frame S, expressed in the same frame E as V1_E.
/// @note This method should only be used if you are sure it makes sense.
/// One use case is for calculating relative spatial velocity, e.g., a frame C's
/// spatial velocity relative to a frame B, measure in a frame W.  For this use
/// case, this method returns V_W_BC_E = V_WC_E - V_WB_E, which contains ω_BC
/// (C's angular velocity measured in B) and v_W_BoCo (Co's velocity relative to
/// Bo, measured in W), where both ω_BC and v_W_BoCo are expressed in frame E.
/// <pre>
///  ω_BC  = ω_WC - ω_WB
///  v_W_BoCo = v_WCo - v_WBo = DtW(p_BoCo)
/// </pre>
///
/// A second use case has to do with a frame C that is moving on a frame B and
/// calculates frame C's spatial velocity measured-in frame B. It assumes you
/// have pre-calculated V_WBc (frame Bc's spatial velocity measured-in frame W,
/// where frame Bc is fixed to B and instantaneously coincident with frame C.
/// This use case returns V_BC_E = V_WC_E - V_WBc_E, where the precalculated
/// V_WBc_E is equal to V_WBo_E.Shift(p_BoBc_E).
/// @see related methods ShiftInPlace() and ComposeWithMovingFrameVelocity().
template <typename T>
inline SpatialVelocity<T> operator-(
    const SpatialVelocity<T>& V1_E, const SpatialVelocity<T>& V2_E) {
  // Although this method is implemented by simply calling the corresponding
  // SpatialVector implementation, it is needed for documentation.
  return SpatialVelocity<T>(V1_E) -= V2_E;
}

template <typename T>
T SpatialVelocity<T>::dot(const SpatialForce<T>& force) const {
  return this->get_coeffs().dot(force.get_coeffs());
}

// This was declared in spatial_force.h, but must be implemented here in
// order to break the dependency cycle.
template <typename T>
T SpatialForce<T>::dot(const SpatialVelocity<T>& velocity) const {
  return velocity.dot(*this);  // dot-product is commutative.
}

// This was declared in spatial_momentum.h, but must be implemented here in
// order to break the dependency cycle.
template <typename T>
T SpatialMomentum<T>::dot(const SpatialVelocity<T>& velocity) const {
  return this->get_coeffs().dot(velocity.get_coeffs());
}

template <typename T>
T SpatialVelocity<T>::dot(const SpatialMomentum<T>& momentum) const {
  return momentum.dot(*this);  // dot-product is commutative.
}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::SpatialVelocity)
