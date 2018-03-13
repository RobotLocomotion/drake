#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"

namespace drake {
namespace multibody {

// Forward declaration to define dot product with a spatial velocity.
template <typename T> class SpatialVelocity;

/// This class is used to represent the _spatial momentum_ of a particle, system
/// of particles or body (whether rigid or soft.)
/// The linear momentum `h_NS` of a system of particles S in a reference frame
/// N is defined by: <pre>
///   h_NS = ∑h_NQi = ∑mᵢv_NQi
/// </pre>
/// where `mᵢ` and `v_NQi` are the mass and linear velocity (in frame N) of the
/// i-th particle in the system, respectively. Their product `h_NQi = mᵢv_NQi`
/// is the linear momentum of the i-th particle in the N reference frame.
/// The angular momentum `l_NSp` of a system of particles S in a reference frame
/// N about an arbitrary point P is defined by: <pre>
///   l_NSp = ∑ p_PQi x h_NQi
/// </pre>
/// where `p_PQi` is the position vector from point P to the i-th particle
/// position `Qi`.
/// The definitions above extends to a continuum of particles as: <pre>
///   l_NSp = ∫p_PQ(r) x v_NQ(r) ρ(r)d³r
///   h_NS = ∫v_NQ(r) ρ(r)d³r
/// </pre>
/// and in particular it also applies to rigid bodies.
///
/// Spatial momenta are elements of F⁶ (see [Featherstone 2008]) that combine
/// both rotational (angular momentum) and translational (linear momentum)
/// components. Spatial momenta are 6-element quantities that are pairs of
/// ordinary 3-vectors. Elements 0-2 are the angular momentum component while
/// elements 3-5 are the linear momentum component. As with any other spatial
/// vector, both vector components must be expressed
/// in the same frame.
///
/// Neither the expressed-in frame nor the about-point are stored with a
/// %SpatialMomentum object; they must be understood from context. It is the
/// responsibility of the user to keep track of the about-point and the
/// expressed-in frame. That is best accomplished through disciplined notation.
/// In source code we use monogram notation where H is used to designate a
/// spatial momentum quantity. We write a point P fixed to body (or frame) B as
/// @f$B_P@f$ which appears in code and comments as `Bp`. Then we write as
/// @f$[^NH^{S/B_P}]_E@f$, which appears in code as `H_NBp_E`, the spatial
/// momentum of a body B in a reference frame N, about a point P and, expressed
/// in frame E. Very often the about-point will be the body origin `Bo`; if no
/// point is shown the origin is understood, so `H_NB_E` means `H_NBo_E`.
/// For a more detailed introduction on spatial vectors and the monogram
/// notation please refer to section @ref multibody_spatial_vectors.
///
/// - [Featherstone 2008] Featherstone, R., 2008. Rigid body dynamics
///                       algorithms. Springer.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class SpatialMomentum : public SpatialVector<SpatialMomentum, T> {
  // We need the fully qualified class name below for the clang compiler to
  // work. Without qualifiers the code is legal according to the C++11 standard
  // but the clang compiler still gets confused. See:
  // http://stackoverflow.com/questions/17687459/clang-not-accepting-use-of-template-template-parameter-when-using-crtp
  typedef SpatialVector<::drake::multibody::SpatialMomentum, T> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialMomentum)

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial momentum are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial momentum fail fast,
  /// allowing fast bug detection.
  SpatialMomentum() : Base() {}

  /// SpatialMomentum constructor from an angular momentum l and a linear
  /// momentum h.
  SpatialMomentum(const Eigen::Ref<const Vector3<T>>& l,
                  const Eigen::Ref<const Vector3<T>>& h) : Base(l, h) {}

  /// SpatialMomentum constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of H is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialMomentum(const Eigen::MatrixBase<Derived>& H) : Base(H) {}

  /// In-place shift of a %SpatialMomentum from one "about-point" to another.
  /// `this` spatial momentum `H_NSp_E` for a system S in a reference frame N
  /// about a point P, and expressed in frame E, is modified to become the
  /// equivalent spatial momentum `H_NSq_E` of the same system about another
  /// point Q.
  ///
  /// We are given the vector from point P to point Q, as a position vector
  /// `p_PQ_E` expressed in the same frame E as the spatial momentum. The
  /// operation performed, in coordinate-free form, is:
  /// <pre>
  ///   l_NSq  = l_NSp -  p_PQ x h_NSp
  ///   h_NSq = h_NSp,  i.e. the linear momentum about point Q is the
  ///                   same as the linear momentum about point P.
  /// </pre>
  /// where l and `h` represent the angular and linear momentum components
  /// respectively. Notice that spatial momenta shift in the same way as spatial
  /// forces (see SpatialForce.)
  ///
  /// Notice this operation is linear. [Jain 2010], (§2.1, page 22) uses the
  /// "rigid body transformation operator" to write this as: <pre>
  ///   H_NSq = Φ(p_QP)H_NSp = Φ(-p_PQ)H_NSp
  /// </pre>
  /// where `Φ(p_PQ)` is the linear operator: <pre>
  ///   Φ(p_PQ) = | I₃ p_PQx |
  ///             | 0     I₃ |
  /// </pre>
  /// where `p_PQx` denotes the cross product, skew-symmetric, matrix such that
  /// `p_PQx v = p_PQ x v`.
  /// This same operator shifts spatial forces in analogous way (see
  /// SpatialForce::Shift()) while the transpose of this operator allow us to
  /// shift spatial velocities, see SpatialVelocity::Shift().
  ///
  /// - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
  ///               algorithms. Springer Science & Business Media, pp. 123-130.
  ///
  /// For computation, all quantities above must be expressed in a common
  /// frame E; we add an `_E` suffix to each symbol to indicate that.
  ///
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] p_PQ_E
  ///   Shift vector from point P to point Q, expressed in frame E.
  /// @returns A reference to `this` spatial momentum which is now `H_NSq_E`,
  ///          that is, the spatial momentum about point Q rather than P.
  ///
  /// @see Shift() to compute the shifted spatial momentum without modifying
  ///              this original object.
  SpatialMomentum<T>& ShiftInPlace(const Vector3<T>& p_PQ_E) {
    this->rotational() -= p_PQ_E.cross(this->translational());
    return *this;
  }

  /// Shift of a %SpatialMomentum from one application point to another.
  /// This is an alternate signature for shifting a spatial momentum's
  /// about-point that does not change the original object. See
  /// ShiftInPlace() for more information.
  ///
  /// @param[in] p_PQ_E
  ///   Shift vector from point P to point Q.
  /// @retval H_NSq_E
  ///   The equivalent shifted spatial momentum, now applied at point Q
  ///   rather than P.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial momentum in-place
  ///                     modifying the original object.
  SpatialMomentum<T> Shift(const Vector3<T>& p_PQ_E) const {
    return SpatialMomentum<T>(*this).ShiftInPlace(p_PQ_E);
  }

  /// Adds in a spatial momentum to `this` spatial momentum.
  /// @param[in] H_NSp_E
  ///   A spatial momentum to be added to `this` spatial momentum. It must be
  ///   about the same point P as this spatial momentum and expressed in the
  ///   same frame E.
  /// @returns
  ///   A reference to `this` spatial momentum, which has been updated to
  ///   include the given spatial momentum `H_NSp_E`.
  ///
  /// @warning This operation is only valid if both spatial momenta are applied
  /// about same point P and expressed in the same frame E.
  SpatialMomentum<T>& operator+=(const SpatialMomentum<T>& H_NSp_E) {
    this->get_coeffs() += H_NSp_E.get_coeffs();
    return *this;
  }

  /// Subtracts a spatial momentum from `this` spatial momentum.
  /// @param[in] H_NSp_E
  ///   A spatial momentum to be subtracted from `this` spatial momentum. It
  ///   must be about the same point P as this spatial momentum and expressed in
  ///   the same frame E.
  /// @returns
  ///   A reference to `this` spatial momentum, which has been updated to
  ///   exclude the given spatial momentum `H_NSp_E`.
  ///
  /// @warning This operation is only valid if both spatial momenta are applied
  /// about same point P and expressed in the same frame E.
  SpatialMomentum<T>& operator-=(const SpatialMomentum<T>& H_NSp_E) {
    this->get_coeffs() -= H_NSp_E.get_coeffs();
    return *this;
  }

  /// Given `this` spatial momentum `H_NBp_E` of a rigid body B, about point P
  /// and, expressed in a frame E, this method computes the dot
  /// product with the spatial velocity `V_NBp_E` of body B frame shifted to
  /// point P, measured in an inertial (or Newtonian) frame N and expressed in
  /// the same frame E in which the spatial momentum is expressed.
  /// This dot-product is twice the kinetic energy `K_NB` of body B in reference
  /// frame N. The kinetic energy `K_NB` is independent of the about-point P and
  /// so is this dot product. Therefore it is always true that:
  /// <pre>
  ///   K_NB = 1/2 (H_NBp⋅V_NBp) = 1/2 (H_NBcm⋅V_NBcm)
  /// </pre>
  /// where `H_NBcm` is the spatial momentum about the center of mass of body B
  /// and `V_NBcm` is the spatial velocity of frame B shifted to its center of
  /// mass. The above is true due to how spatial momentum and velocity shift
  /// when changing point P, see SpatialMomentum::Shift() and
  /// SpatialVelocity::Shift().
  T dot(const SpatialVelocity<T>& V_NBp_E) const;
};

/// Computes the resultant spatial momentum as the addition of two spatial
/// momenta `H1_NSp_E` and `H2_NSp_E` on a same system S, about the same point P
/// and expressed in the same frame E.
/// @retval Hc_NSp_E
///   The combined spatial momentum of system S from combining `H1_NSp_E`
///   and `H2_NSp_E`, applied about the same point P and in the same
///   expressed-in frame E as the operand spatial momenta.
template <typename T>
inline SpatialMomentum<T> operator+(
    const SpatialMomentum<T>& H1_NSp_E, const SpatialMomentum<T>& H2_NSp_E) {
  return SpatialMomentum<T>(H1_NSp_E.get_coeffs() + H2_NSp_E.get_coeffs());
}

}  // namespace multibody
}  // namespace drake
