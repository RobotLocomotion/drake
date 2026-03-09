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

/// This class represents a _spatial velocity_ V (also called a _twist_) and has
/// 6 elements with an angular (rotational) velocity ω (3-element vector) on top
/// of a translational (linear) velocity v (3-element vector). Spatial velocity
/// represents the rotational and translational motion of a frame B with respect
/// to a _measured-in_ frame M. This class assumes that both the angular
/// velocity ω and translational velocity v are expressed in the same
/// _expressed-in_ frame E. This class only stores 6 elements (namely ω and v)
/// and does not store the underlying frames B, M, E. The user is responsible
/// for explicitly tracking the underlying frames with @ref multibody_quantities
/// "monogram notation". For example, V_MB_E denotes frame B's spatial velocity
/// measured in frame M, expressed in frame E and contains ω_MB_E (B's angular
/// velocity measured in M, expressed in E) and v_MBo_E (Bo's translational
/// velocity measured in M, expressed in E), where Bo is frame B's origin point.
/// For an @ref multibody_frames_and_bodies "offset frame" Bp, the monogram
/// notation V_MBp_E denotes the spatial velocity of frame Bp measured in M,
/// expressed in E. Details on spatial vectors and monogram notation are in
/// sections @ref multibody_spatial_vectors and @ref multibody_quantities.
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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialVelocity);

  /// Default constructor. In Release builds, all 6 elements of a newly
  /// constructed spatial velocity are uninitialized (for speed).  In Debug
  /// builds, the 6 elements are set to NaN so that invalid operations on an
  /// uninitialized spatial velocity fail fast (fast bug detection).
  SpatialVelocity() : Base() {}

  /// Constructs a spatial velocity V from an angular velocity ω and a
  /// translational velocity v.
  SpatialVelocity(const Eigen::Ref<const Vector3<T>>& w,
                  const Eigen::Ref<const Vector3<T>>& v)
      : Base(w, v) {}

  /// Constructs a spatial velocity V from an Eigen expression that represents a
  /// 6-element vector, i.e., two 3-element vectors, namely an angular velocity
  /// ω and a translational velocity v.  This constructor will assert the size
  /// of V is six (6) either at compile-time for fixed sized Eigen expressions
  /// or at run-time for dynamic sized Eigen expressions.
  template <typename Derived>
  explicit SpatialVelocity(const Eigen::MatrixBase<Derived>& V) : Base(V) {}

  /// In-place shift of a %SpatialVelocity from a frame B to a frame C, where
  /// both B and C are fixed to the same frame or rigid body. On entry, `this`
  /// is V_MB_E (frame B's spatial velocity measured in a frame M and expressed
  /// in a frame E). On return `this` is modified to V_MC_E (frame C's spatial
  /// velocity measured in frame M and expressed in frame E).
  /// The components of V_MC_E are calculated as: <pre>
  ///  ω_MC_E = ω_MB_E                (angular velocity of `this` is unchanged).
  ///  v_MC_E = v_MB_E + ω_MB_E x p_BoCo_E     (translational velocity changes).
  /// </pre>
  /// @param[in] offset which is the position vector p_BoCo_E from frame B's
  /// origin to frame C's origin, expressed in frame E. p_BoCo_E must have
  /// the same expressed-in frame E as `this` spatial velocity.
  /// @see Shift() to shift spatial velocity without modifying `this`.
  void ShiftInPlace(const Vector3<T>& offset) {
    this->translational() += this->rotational().cross(offset);  // 12 flops
    // Note: this operation is linear. [Jain 2010], (§1.4, page 12) uses the
    // "rigid body transformation operator" to write this as:
    //    V_MC = Φᵀ(p_BoCo) V_MB    where Φᵀ(p) is the linear operator:
    //   Φᵀ(p) = | I₃   0₃ |
    //           | -pₓ  I₃ |       I₃ is the 3x3 identity matrix, 0₃ is the 3x3
    // zero matrix, and pₓ denotes the skew-symmetric cross product matrix such
    // that pₓvec = p x vec (where vec is any vector). This Φᵀ operator shifts
    // spatial velocity whereas the Φ operator shifts spatial force and spatial
    // momentum (see SpatialForce::Shift() and SpatialMomentum:Shift()).
    //
    // - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
    //               algorithms. Springer Science & Business Media, pp. 123-130.
  }

  /// Shifts a %SpatialVelocity from a frame B to a frame C, where both B and C
  /// are fixed to the same frame or rigid body.
  /// @param[in] offset which is the position vector p_BoCo_E from frame B's
  /// origin to frame C's origin, expressed in frame E. p_BoCo_E must have the
  /// same expressed-in frame E as `this` spatial velocity (`this` = V_MB_E).
  /// @retval V_MC_E which is frame C's spatial velocity measured in frame M,
  /// expressed in frame E.
  /// @note Shift() differs from ShiftInPlace() in that Shift() does not modify
  /// `this` whereas ShiftInPlace() does modify `this`.
  /// @see ShiftInPlace() for more information and how V_MC_E is calculated.
  SpatialVelocity<T> Shift(const Vector3<T>& offset) const {
    SpatialVelocity<T> result(*this);
    result.ShiftInPlace(offset);  // 12 flops
    return result;
  }

  /// Compose `this` spatial velocity (measured in some frame M) with the
  /// spatial velocity of another frame to form the 𝐨𝐭𝐡𝐞𝐫 frame's spatial
  /// velocity measured in frame M. Herein, `this` is the spatial velocity of a
  /// frame (designated B) in frame M and the 𝐨𝐭𝐡𝐞𝐫 frame is designated C.
  /// @param[in] position_of_moving_frame which is the position vector p_BoCo_E
  /// (from frame B's origin Bo to frame C's origin Co), expressed in a frame E.
  /// p_BoCo_E must have the same expressed-in frame E as `this`, where `this`
  /// is V_MB_E (frame B's spatial velocity measured in M, expressed in E).
  /// @param[in] velocity_of_moving_frame which is V_BC_E, frame C's spatial
  /// velocity measured in frame B, expressed in frame E.
  /// @retval V_MC_E frame C's spatial velocity measured in frame M, expressed
  /// in frame E.
  /// @note The returned spatial velocity V_MC_E contains an angular velocity
  /// ω_MC_E and translational velocity v_MCo_E that are calculated as: <pre>
  ///  ω_MC_E  = ω_MB_E + ω_BC_E
  ///  v_MCo_E = v_MBo_E + ω_MB_E x p_BoCo_E + v_BCo_E
  /// </pre>
  /// If frame C is rigidly fixed to frame B, V_BC_E = 0 and this method
  /// produces a Shift() operation (albeit inefficiently). In other words, use
  /// Shift() if velocity_of_moving_frame = 0.
  /// @see SpatialAcceleration::ComposeWithMovingFrameAcceleration().
  SpatialVelocity<T> ComposeWithMovingFrameVelocity(
      const Vector3<T>& position_of_moving_frame,
      const SpatialVelocity<T>& velocity_of_moving_frame) const {
    const Vector3<T>& p_BoCo_E = position_of_moving_frame;
    const SpatialVelocity<T>& V_BC_E = velocity_of_moving_frame;
    // V_MC_E = V_MB_E.Shift(p_BoCo_E) + V_BC_E
    return this->Shift(p_BoCo_E) + V_BC_E;  // 18 flops
  }

  /// Calculates the power generated by a spatial force.
  /// For an arbitrary frame B, calculates the dot-product of `this` = V_MB_E
  /// (frame B's spatial velocity measured in a frame M, expressed in a frame E)
  /// with F_B_E (frame B's spatial force, expressed in frame E).
  /// @param[in] force which is F_B_E, frame B's spatial force, expressed in the
  /// same frame E as `this` = V_MB_E.
  /// @returns Power of spatial force F_B_E in frame M, i.e., F_B_E ⋅ V_MB_E.
  /// @note Just as equating force 𝐅 to mass * acceleration as 𝐅 = m𝐚 relies
  /// on acceleration 𝐚 being measured in a world frame (also called an inertial
  /// or Newtonian frame), equating power = dK/dt (where K is kinetic energy)
  /// relies on K being measured in a world frame.  Hence, it is unusual to use
  /// this method unless frame M is the world frame W.
  /// @note Although the spatial vectors F_B_E and V_MB_E must have the same
  /// expressed-in frame E, the returned scalar is independent of frame E.
  inline T dot(const SpatialForce<T>& force) const;
  // The dot() method is implemented at the end of this file, so that all of
  // the dot methods are co-located for easy understanding. We need the inline
  // keyword to ensure the method is still inlined even with `extern template`.

  /// Calculates twice (2x) a body B's kinetic energy measured in a frame M.
  /// For any frame (e.g., an @ref multibody_frames_and_bodies "offset frame")
  /// Bp that is fixed to a rigid body B, calculates the dot-product of
  /// `this` = V_MBp_E (frame Bp's spatial velocity measured in frame M,
  /// expressed in frame E) with L_MBp_E (body B's spatial momentum measured in
  /// frame M, about Bp's origin, expressed in frame E).
  /// @param[in] momentum which is L_MBp_E, body B's spatial momentum measured
  /// in frame M, about frame Bp's origin, expressed in the same frame E as
  /// `this` = V_MBp_E.
  /// @returns 2*K_MB, twice (2x) body B's kinetic energy measured in frame M.
  /// @note In general, kinetic energy calculations are only useful when frame M
  /// is a world frame (also called a Newtonian or inertial frame). Hence, it
  /// is unusual to use this method unless frame M is the world frame W.
  /// @note Although the spatial vectors V_MBp_E and L_MBp_E must have the same
  /// expressed-in frame E, the resulting scalar K_MB is independent of frame E.
  /// @note As shown below, K_MB can be calculated from any frame Bp fixed on B,
  /// including body B's center of mass frame Bcm. This is due to how spatial
  /// momentum and spatial velocity shift from Bcm to Bp. For more information,
  /// see SpatialMomentum::Shift() and SpatialVelocity::Shift(). <pre>
  ///   K_MB = 1/2 (L_MBp · V_MBp) = 1/2 (L_MBcm · V_MBcm)
  /// </pre>
  inline T dot(const SpatialMomentum<T>& momentum) const;
  // The dot() method is implemented at the end of this file, so that all of
  // the dot methods are co-located for easy understanding. We need the inline
  // keyword to ensure the method is still inlined even with `extern template`.
};

/// Adds two spatial velocities by simply adding their 6 underlying elements.
/// @param[in] V1_E spatial velocity expressed in the same frame E as V2_E.
/// @param[in] V2_E spatial velocity expressed in the same frame E as V1_E.
/// @note The general utility of this operator+() function is questionable and
/// it should only be used if you are sure it makes sense.  One use case is
/// for calculating the spatial velocity V_MC of a frame C measured in a frame M
/// when frame C is moving on a frame B and one has pre-calculated
/// V_MBc (frame Bc's spatial velocity measured in frame M, where frame Bc is
/// instantaneously coincident with frame C). For this use case, the operator+
/// function returns V_MC_E = V_MBc_E + V_BC_E, where the precalculated V_MBc_E
/// is equal to V_MBo_E.Shift(p_BoCo_E).
/// @see Shift(), ShiftInPlace(), and ComposeWithMovingFrameVelocity().
/// @relates SpatialVelocity
template <typename T>
inline SpatialVelocity<T> operator+(const SpatialVelocity<T>& V1_E,
                                    const SpatialVelocity<T>& V2_E) {
  // Although this implementation calls the base class operator, it is needed
  // for documentation.
  return SpatialVector<SpatialVelocity, T>::operator+(V1_E, V2_E);
}

/// Subtracts spatial velocities by simply subtracting their 6 underlying
/// elements.
/// @param[in] V1_E spatial velocity expressed in the same frame E as V2_E.
/// @param[in] V2_E spatial velocity expressed in the same frame E as V1_E.
/// @note The general utility of this operator-() function is questionable and
/// it should only be used if you are sure it makes sense.
/// One use case is for calculating relative spatial velocity, e.g., a frame C's
/// spatial velocity relative to a frame B, measure in a frame M. This use case
/// calculates V_M_BC_E = V_MC_E - V_MB_E, which contains ω_BC (C's angular
/// velocity measured in B) and v_M_BoCo (Co's velocity relative to Bo, measured
/// in M), where both ω_BC and v_M_BoCo are expressed in frame E. <pre>
///  ω_BC  = ω_MC - ω_MB
///  v_M_BoCo = v_MCo - v_MBo = DtM(p_BoCo)
/// </pre>
/// where DtM(p_BoCo) is the time-derivative in frame M of p_BoCo (position
/// vector from Bo to Co).
/// A second use case has to do with a frame C that is moving on a frame B and
/// calculates frame C's spatial velocity measured in frame B. It assumes you
/// have pre-calculated V_MBc (frame Bc's spatial velocity measured in frame M,
/// where frame Bc is fixed to B and instantaneously coincident with frame C.
/// This use case returns V_BC_E = V_MC_E - V_MBc_E, where the precalculated
/// V_MBc_E is equal to V_MBo_E.Shift(p_BoBc_E).
/// @see Shift(), ShiftInPlace(), and ComposeWithMovingFrameVelocity().
/// @relates SpatialVelocity
template <typename T>
inline SpatialVelocity<T> operator-(const SpatialVelocity<T>& V1_E,
                                    const SpatialVelocity<T>& V2_E) {
  // Although this implementation calls the base class operator, it is needed
  // for documentation.
  return SpatialVector<SpatialVelocity, T>::operator-(V1_E, V2_E);
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
    class ::drake::multibody::SpatialVelocity);
