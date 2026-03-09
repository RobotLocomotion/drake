#pragma once

#ifndef DRAKE_SPATIAL_ALGEBRA_HEADER
#error Please include "drake/multibody/math/spatial_algebra.h", not this file.
#endif

#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_vector.h"

namespace drake {
namespace multibody {

// Forward declaration to define dot product with a spatial velocity.
template <typename T>
class SpatialVelocity;

/// This class represents a _spatial momentum_ L and has 6 elements with an
/// angular (rotational) momentum 𝐡 (3-element vector) on top of a translational
/// (linear) momentum 𝐥 (3-element vector). A spatial momentum L stores the
/// angular momentum 𝐡 and translational momentum 𝐥 of a system S about a point
/// P, measured in a frame M, and expressed in a frame E. The system S may be a
/// particle, a rigid or deformable body, or a set of particles and/or bodies.
/// This class assumes that both the angular momentum 𝐡 and translational
/// momentum 𝐥 are expressed in the same _expressed-in_ frame E. This class only
/// stores 6 elements (namely 𝐡 and 𝐥) and does not store the underlying
/// system S, about-point P, measured-in frame M, or expressed-in frame E.
/// The user is responsible for explicitly tracking the underlying system,
/// about-point, and frames with @ref multibody_quantities "monogram notation".
/// For example, L_MSP_E denotes a system S's spatial momentum about point P,
/// measured in frame M, and expressed in frame E. L_MSP_E contains h_MSP_E
/// (S's angular momentum about point P, measured in M, expressed in E) and
/// l_MS_E (S's translational momentum measured in M, expressed in E). A body
/// B's spatial momentum about point Bo (B's origin), measured in frame M,
/// expressed in frame E has explicit monogram notation L_MBBo_E which can be
/// abbreviated L_MBo_E. Similarly L_MSScm_E is abbreviated L_MScm_E (Scm is
/// S's center of mass). Details on spatial vectors and monogram notation are in
/// sections @ref multibody_spatial_vectors and @ref multibody_quantities.
///
/// The typeset for L_MSP_E is @f$[^ML^{S/P}]_E@f$. For a set S of particles Qᵢ,
/// L_MSP contains S's angular momentum 𝐡 about-point P, measured in frame M
/// and S's translational momentum 𝐥 measured in frame M, defined as <pre>
///   h_MSP = ∑ h_MQᵢP = ∑ p_PQᵢ x l_MQᵢ  where l_MQᵢ = mᵢ v_MQᵢ.
///   l_MS  = ∑ l_MQᵢ  = ∑ mᵢ v_MQᵢ
/// </pre>
/// where mᵢ is the mass of particle Qᵢ, v_MQᵢ is Qᵢ's translational velocity
/// measured in frame M, l_MQᵢ = mᵢ v_MQQᵢ is Qᵢ's translational momentum
/// measured in frame M, h_MQᵢP is Qᵢ's angular momentum about point P
/// measured in frame M, and p_PQᵢ is the position vector from point P to Qᵢ.
/// These definitions extend to a body (continuum of particles) by using the
/// density ρ(r) of the body at each material location r as: <pre>
///   h_MSP = ∫p_PQ(r) x v_MQ(r) ρ(r) d³r
///   l_MS  = ∫v_MQ(r) ρ(r) d³r
/// </pre>
///
/// @tparam_default_scalar
template <typename T>
class SpatialMomentum : public SpatialVector<SpatialMomentum, T> {
  // We need the fully qualified class name below for the clang compiler to
  // work. Without qualifiers the code is legal according to the C++11 standard
  // but the clang compiler still gets confused. See:
  // http://stackoverflow.com/questions/17687459/clang-not-accepting-use-of-template-template-parameter-when-using-crtp
  typedef SpatialVector<::drake::multibody::SpatialMomentum, T> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialMomentum);

  /// Default constructor. In Release builds, all 6 elements of a newly
  /// constructed spatial momentum are uninitialized (for speed). In Debug
  /// builds, the 6 elements are set to NaN so that invalid operations on an
  /// uninitialized spatial momentum fail fast (fast bug detection).
  SpatialMomentum() : Base() {}

  /// Constructs a spatial momentum L from an angular momentum 𝐡
  /// and a translational momentum 𝐥.
  SpatialMomentum(const Eigen::Ref<const Vector3<T>>& h,
                  const Eigen::Ref<const Vector3<T>>& l)
      : Base(h, l) {}

  /// Constructs a spatial momentum L from an Eigen expression that represents
  /// a 6-element vector, i.e., a 3-element angular momentum 𝐡 and a
  /// 3-element translational momentum 𝐥. This constructor will assert the
  /// size of L is six (6) either at compile-time for fixed sized Eigen
  /// expressions or at run-time for dynamic sized Eigen expressions.
  template <typename Derived>
  explicit SpatialMomentum(const Eigen::MatrixBase<Derived>& L) : Base(L) {}

  /// In-place shift of a %SpatialMomentum from an about-point P to an
  /// about-point Q. On entry, `this` is L_MSP_E (system S's spatial momentum
  /// about point P, measured in a frame M and expressed in a frame E). On
  /// return `this` is modified to L_MSQ_E (S's spatial momentum about point Q,
  /// measured in frame M and expressed in frame E).
  /// The components of L_MSQ_E are: <pre>
  ///   l_MS_E = l_MS_E         (translational momentum of `this` is unchanged).
  ///  h_MSQ_E = h_MSP_E + p_QP_E x l_MS_E
  ///          = h_MSP_E - p_PQ_E x l_MS_E
  /// </pre>
  /// @param[in] offset which is the position vector p_PQ_E from point P to
  /// point Q, with the same expressed-in frame E as `this` spatial momentum.
  /// @note Spatial momenta shift similar to spatial force (see SpatialForce)
  /// and in a related/different way for spatial velocity (see SpatialVelocity).
  /// @see Shift() to shift spatial momentum without modifying `this`.
  void ShiftInPlace(const Vector3<T>& offset) {
    this->rotational() -= offset.cross(this->translational());
    // Note: this operation is linear. [Jain 2010], (§2.1, page 22) uses the
    // "rigid body transformation operator" to write this as:
    //  L_MSQ = Φ(-p_PQ) L_MSP
    //        =  Φ(p_QP) L_MSP    where Φ(p) is the linear operator:
    //   Φ(p) = | I₃   pₓ |
    //          | 0₃   I₃ |       I₃ is the 3x3 identity matrix, 0₃ is the 3x3
    // zero matrix and pₓ denotes the skew-symmetric cross product matrix such
    // that pₓvec = p x vec (where vec is any vector).
    // This same Φ operator shifts spatial force in an analogous way (see
    // SpatialForce::Shift()) whereas Φᵀ (the transpose of this operator)
    // shifts spatial velocity (see SpatialVelocity::Shift()).
    //
    // - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
    //               algorithms. Springer Science & Business Media, pp. 123-130.
  }

  /// Shifts a %SpatialMomentum from an about-point P to an about-point Q.
  /// @param[in] offset which is the position vector p_PQ_E from point P to
  /// point Q, expressed in frame E. p_PQ_E must have the same expressed-in
  /// frame E as `this` spatial momentum, where `this` is L_MSP_E (system S's
  /// spatial momentum about P, measured in frame M, expressed in frame E).
  /// @retval L_MSQ_E which is system S's spatial momentum about point Q,
  /// measured in frame M, expressed in frame E.
  /// @note Shift() differs from ShiftInPlace() in that Shift() does not modify
  /// `this` whereas ShiftInPlace() does modify `this`.
  /// @see ShiftInPlace() for more information and how L_MSQ_E is calculated.
  SpatialMomentum<T> Shift(const Vector3<T>& offset) const {
    SpatialMomentum<T> result(*this);
    result.ShiftInPlace(offset);
    return result;
  }

  /// Calculates twice (2x) a body B's kinetic energy measured in a frame M.
  /// For any frame (e.g., an @ref multibody_frames_and_bodies "offset frame")
  /// Bp that is fixed to a rigid body B, calculates the dot-product of
  /// `this` = L_MBp_E (body B's spatial momentum measured in frame M, about
  /// Bp's origin, expressed in frame E) with V_MBp_E (frame Bp's spatial
  /// velocity measured in frame M, expressed in frame E).
  /// @param[in] velocity which is V_MBp_E, frame Bp's spatial velocity measured
  /// in frame M, and expressed in the same frame E as `this` = L_MBp_E.
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
  inline T dot(const SpatialVelocity<T>& velocity) const;
  // The dot() method is implemented in spatial_velocity.h. We need the inline
  // keyword to ensure the method is still inlined even with `extern template`.
};

/// Adds two spatial momenta by simply adding their 6 underlying elements.
/// @param[in] L1_E spatial momentum expressed in the same frame E as L2_E.
/// @param[in] L2_E spatial momentum expressed in the same frame E as L1_E.
/// @note The general utility of this operator+() function seems limited to
/// situations when L1 and L2 are associated with different systems (S1 and S2),
/// but have the same about-point P, same measured-in frame M, and same
/// expressed-in frame E.
/// @relates SpatialMomentum
template <typename T>
inline SpatialMomentum<T> operator+(const SpatialMomentum<T>& L1_E,
                                    const SpatialMomentum<T>& L2_E) {
  // Although this implementation calls the base class operator, it is needed
  // for documentation.
  return SpatialVector<SpatialMomentum, T>::operator+(L1_E, L2_E);
}

/// Subtracts spatial momenta by simply subtracting their 6 underlying elements.
/// @param[in] L1_E spatial momentum expressed in the same frame E as L2_E.
/// @param[in] L2_E spatial momentum expressed in the same frame E as L1_E.
/// @note The general utility of this operator-() function is questionable and
/// it should only be used if you are sure it makes sense.
/// @relates SpatialMomentum
template <typename T>
inline SpatialMomentum<T> operator-(const SpatialMomentum<T>& L1_E,
                                    const SpatialMomentum<T>& L2_E) {
  // Although this implementation calls the base class operator, it is needed
  // for documentation.
  return SpatialVector<SpatialMomentum, T>::operator+(L1_E, L2_E);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::SpatialMomentum);
