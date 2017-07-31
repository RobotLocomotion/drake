#pragma once

#include <algorithm>
#include <exception>
#include <iostream>
#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/cross_product.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/unit_inertia.h"

namespace drake {
namespace multibody {

/// This class represents the physical concept of a _Spatial Inertia_. A
/// spatial inertia (or spatial mass matrix) encapsulates the mass, center of
/// mass, and rotational inertia of the mass distribution of a body or composite
/// body S, where with "composite body" we mean a collection of bodies welded
/// together containing at least one body (throughout this documentation "body"
/// is many times used instead of "composite body" but the same concepts apply
/// to a collection of bodies as well.)
/// A spatial inertia is an element of ℝ⁶ˣ⁶ that is symmetric, and positive
/// semi-definite. It logically consists of `3x3` sub-matrices arranged like
/// so, [Jain 2010]:
/// <pre>
///              Spatial mass matrix
///           ------------ ------------
///        0 |            |            |
///        1 |    I_SP    | m p_PScm×  |
///        2 |            |            |
///           ------------ ------------
///        3 |            |            |
///        4 | -m p_PScm× |     m Id   |
///        5 |            |            |
///           ------------ ------------
///                Symbol: M
/// </pre>
/// where, with the monogram notation described in
/// @ref multibody_spatial_inertia, `I_SP` is the rotational inertia of body or
/// composite body S computed about a point P, m is the mass of this composite
/// body, `p_PScm` is the position vector from point P to the center of mass
/// `Scm` of the composite body S with `p_PScm×` denoting its skew-symmetric
/// cross product matrix (defined such that `a× b = a.cross(b)`), and `Id` is
/// the identity matrix in ℝ³ˣ³. See Section 2.1, p. 17 of [Jain 2010].
/// The logical arrangement as shown above is chosen to be consistent with our
/// logical arrangement for spatial vectors as documented in
/// @ref multibody_spatial_algebra for which the rotational component comes
/// first followed by the translational component.
///
/// In typeset material we use the symbol @f$ [M^{S/P}]_E @f$ to represent the
/// spatial inertia of a body or composite body S about point P, expressed in
/// frame E. For this inertia, the monogram notation reads `M_SP_E`. If the
/// point P is fixed to a body B, we write that point as @f$ B_P @f$ which
/// appears in code and comments as `Bp`. So if the body or composite body is B
/// and the about point is `Bp`, the monogram notation reads `M_BBp_E`, which
/// can be abbreviated to `M_Bp_E` since the about point `Bp` also identifies
/// the body. Common cases are that the about point is the origin `Bo` of the
/// body, or it's the center of mass `Bcm` for which the rotational inertia in
/// monogram notation would read as `I_Bo_E` and `I_Bcm_E`, respectively.
/// Given `M_BP_E` (@f$[M^{B/P}]_E@f$), the rotational inertia of this spatial
/// inertia is `I_BP_E` (@f$[I^{B/P}]_E@f$) and the position vector of the
/// center of mass measured from point P and expressed in E is `p_PBcm_E`
/// (@f$[^Pp^{B_{cm}}]_E@f$).
///
/// @note This class does not implement any mechanism to track the frame E in
/// which a spatial inertia is expressed or about what point is computed.
/// Methods and operators on this class have no means to determine frame
/// consistency through operations. It is therefore the responsibility of users
/// of this class to keep track of frames in which operations are performed. We
/// suggest doing that using disciplined notation, as described above.
///
/// - [Jain 2010]  Jain, A., 2010. Robot and multibody dynamics: analysis and
///                algorithms. Springer Science & Business Media.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
template <typename T>
class SpatialInertia {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialInertia)

  /// Default SpatialInertia constructor initializes mass, center of mass and
  /// rotational inertia to invalid NaN's for a quick detection of
  /// uninitialized values.
  SpatialInertia() {}

  /// Constructs a spatial inertia for a physical body or composite body S about
  /// a point P from a given mass, center of mass and rotational inertia. The
  /// center of mass is specified by the position vector `p_PScm_E` from point P
  /// to the center of mass point `Scm`, expressed in a frame E.
  /// The rotational inertia is provided as the UnitInertia `G_SP_E` of the body
  /// or composite body S computed about point P and expressed in frame E.
  ///
  /// This constructor checks for the physical validity of the resulting
  /// %SpatialInertia with IsPhysicallyValid() and throws a std::runtime_error
  /// exception in the event the provided input parameters lead to
  /// non-physically viable spatial inertia.
  ///
  /// @param[in] mass The mass of the body or composite body S.
  /// @param[in] p_PScm_E The position vector from point P to the center of mass
  ///                     of body or composite body S expressed in frame E.
  /// @param[in] G_SP_E UnitInertia of the body or composite body S computed
  ///                   about origin point P and expressed in frame E.
  SpatialInertia(
      const T& mass, const Vector3<T>& p_PScm_E, const UnitInertia<T>& G_SP_E) :
      mass_(mass), p_PScm_E_(p_PScm_E), G_SP_E_(G_SP_E) {
    CheckInvariants();
  }

  /// Returns a new %SpatialInertia object templated on `Scalar` initialized
  /// from the value of `this` spatial inertia.
  ///
  /// @tparam Scalar The scalar type on which the new spatial inertia will
  /// be templated.
  ///
  /// @note `SpatialInertia<From>::cast<To>()` creates a new
  /// `SpatialInertia<To>` from a `SpatialInertia<From>` but only if
  /// type `To` is constructible from type `From`. As an example of this,
  /// `SpatialInertia<double>::cast<AutoDiffXd>()` is valid since
  /// `AutoDiffXd a(1.0)` is valid. However,
  /// `SpatialInertia<AutoDiffXd>::cast<double>()` is not.
  template <typename Scalar>
  SpatialInertia<Scalar> cast() const {
    return SpatialInertia<Scalar>(
        get_mass(),
        get_com().template cast<Scalar>(),
        get_unit_inertia().template cast<Scalar>());
  }

  /// Get a constant reference to the mass of this spatial inertia.
  const T& get_mass() const { return mass_;}

  /// Get a constant reference to the position vector `p_PScm_E` from the
  /// _about point_ P to the center of mass `Scm` of the body or composite body
  /// S, expressed in frame E. See the documentation of this class for details.
  const Vector3<T>& get_com() const { return p_PScm_E_;}

  /// Computes the center of mass moment vector `mass * p_PScm_E` given the
  /// position vector `p_PScm_E` from the _about point_ P to the center of mass
  /// `Scm` of the body or composite body S, expressed in frame E. See the
  /// documentation of this class for details.
  Vector3<T> CalcComMoment() const { return mass_ * p_PScm_E_;}

  /// Get a constant reference to the unit inertia `G_SP_E` of this
  /// spatial inertia, computed about point P and expressed in frame E. See the
  /// documentation of this class for details.
  const UnitInertia<T>& get_unit_inertia() const { return G_SP_E_;}

  /// Computes the rotational inertia `I_SP_E = mass * G_SP_E` of this
  /// spatial inertia, computed about point P and expressed in frame E. See the
  /// documentation of this class for details.
  RotationalInertia<T> CalcRotationalInertia() const { return mass_ * G_SP_E_;}

  /// Returns `true` if any of the elements in this spatial inertia is NaN
  /// and `false` otherwise.
  bool IsNaN() const {
    using std::isnan;
    if (isnan(mass_)) return true;
    if (G_SP_E_.IsNaN()) return true;
    if (p_PScm_E_.array().isNaN().any()) return true;
    return false;
  }

  /// Performs a number of checks to verify that this is a physically valid
  /// spatial inertia.
  /// The checks performed are:
  /// - No NaN entries.
  /// - Non-negative mass.
  /// - Non-negative principal moments about the center of mass.
  /// - Principal moments about the center of mass must satisfy the triangle
  ///   inequality:
  ///   - `Ixx + Iyy >= Izz`
  ///   - `Ixx + Izz >= Iyy`
  ///   - `Iyy + Izz >= Ixx`
  /// These are the tests performed by
  /// RotationalInertia::CouldBePhysicallyValid() which become a sufficient
  /// condition when performed on a rotational inertia about a body's center of
  /// mass.
  /// @see RotationalInertia::CouldBePhysicallyValid().
  bool IsPhysicallyValid() const {
    if (IsNaN()) return false;
    if (mass_ < T(0)) return false;
    // The tests in RotationalInertia become a sufficient condition when
    // performed on a rotational inertia computed about a body's center of mass.
    const UnitInertia<T> G_SScm_E = G_SP_E_.ShiftToCenterOfMass(p_PScm_E_);
    if (!G_SScm_E.CouldBePhysicallyValid()) return false;
    return true;  // All tests passed.
  }

  /// Copy to a full 6x6 matrix representation.
  Matrix6<T> CopyToFullMatrix6() const {
    using drake::math::VectorToSkewSymmetric;
    Matrix6<T> M;
    M.template block<3, 3>(0, 0) = mass_ * G_SP_E_.CopyToFullMatrix3();
    M.template block<3, 3>(0, 3) = mass_ * VectorToSkewSymmetric(p_PScm_E_);
    M.template block<3, 3>(3, 0) = -M.template block<3, 3>(0, 3);
    M.template block<3, 3>(3, 3) = mass_ * Matrix3<T>::Identity();
    return M;
  }

  /// Sets `this` spatial inertia to have NaN entries. Typically used for quick
  /// detection of uninitialized values.
  void SetNaN() {
    mass_ = nan();
    p_PScm_E_.setConstant(nan());
    G_SP_E_.SetToNaN();
  }

  /// Adds in a spatial inertia to `this` spatial inertia.
  /// @param[in] M_BP_E A spatial inertia of some body B to be added to
  ///                  `this` spatial inertia. It must be defined about the
  ///                   same point P as `this` inertia, and expressed in the
  ///                   same frame E.
  /// @returns A reference to `this` spatial inertia, which has been updated
  ///          to include the given spatial inertia `M_BP_E`.
  ///
  /// @note This operation aborts if the mass of the resulting spatial inertia
  ///       is zero since in that case the position vector from the about point
  ///       to the center of mass is not well defined.
  ///
  /// @warning This operation is only valid if both spatial inertias are
  /// computed about the same point P and expressed in the same frame E.
  /// Considering `this` spatial inertia to be `M_SP_E` for some body or
  /// composite body S, about some point P, the supplied spatial inertia
  /// `M_BP_E` must be for some other body or composite body B about the _same_
  /// point P; B's inertia is then included in S.
  SpatialInertia& operator+=(const SpatialInertia<T>& M_BP_E) {
    const T total_mass = get_mass() + M_BP_E.get_mass();
    DRAKE_ASSERT(total_mass != 0);
    p_PScm_E_ = (CalcComMoment() + M_BP_E.CalcComMoment()) / total_mass;
    G_SP_E_.SetFromRotationalInertia(
        CalcRotationalInertia() + M_BP_E.CalcRotationalInertia(), total_mass);
    mass_ = total_mass;
    return *this;
  }

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// taken about a point P and expressed in frame E, this method computes the
  /// same inertia re-expressed in another frame A.
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] R_AE Rotation matrix from frame E to frame A.
  /// @returns A reference to `this` rotational inertia about the same point P
  ///          but now re-expressed in frame A, that is, `M_SP_A`.
  ///
  /// @warning This method does not check whether the input matrix `R_AE`
  /// represents a valid rotation or not. It is the resposibility of users to
  /// provide valid rotation matrices.
  SpatialInertia& ReExpressInPlace(const Matrix3<T>& R_AE) {
    p_PScm_E_ = R_AE * p_PScm_E_;    // Now p_PScm_A
    G_SP_E_.ReExpressInPlace(R_AE);  // Now I_SP_A
    return *this;                    // Now M_SP_A
  }

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// taken about a point P and expressed in frame E, this method computes the
  /// same inertia re-expressed in another frame A.
  ///
  /// @param[in] R_AE Rotation matrix from frame E to frame A.
  /// @retval M_SP_A The same spatial inertia of S about P but now
  ///                re-expressed in frame A.
  /// @see ReExpressInPlace() for details.
  SpatialInertia ReExpress(const Matrix3<T>& R_AE) const {
    return SpatialInertia(*this).ReExpressInPlace(R_AE);
  }

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// computed about point P, and expressed in frame E, this method uses
  /// the _Parallel Axis Theorem_ for spatial inertias to compute the same
  /// spatial inertia about a new point Q. The result still is expressed in
  /// frame E.
  /// This operation is performed in-place modifying the original object.
  /// @see Shift() which does not modify this object.
  ///
  /// For details see Section 2.1.2, p. 20 of [Jain 2010].
  ///
  /// @param[in] p_PQ_E Vector from the original about point P to the new
  ///                   about point Q, expressed in the same frame E `this`
  ///                   spatial inertia is expressed in.
  /// @returns A reference to `this` spatial inertia for body or composite body
  ///          S but now computed about about a new point Q.
  SpatialInertia& ShiftInPlace(const Vector3<T>& p_PQ_E) {
    const Vector3<T> p_QScm_E = p_PScm_E_ - p_PQ_E;
    // The following two lines apply the parallel axis theorem (in place) so
    // that:
    //   G_SQ = G_SP + px_QScm² - px_PScm²
    G_SP_E_.ShiftFromCenterOfMassInPlace(p_QScm_E);
    G_SP_E_.ShiftToCenterOfMassInPlace(p_PScm_E_);
    p_PScm_E_ = p_QScm_E;
    // This would only mean a bug in the implementation. The Shift operation
    // should always lead to a valid spatial inertia.
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// computed about point P, and expressed in frame E, this method uses
  /// the _Parallel Axis Theorem_ for spatial inertias to compute the same
  /// spatial inertia about a new point Q. The result still is expressed in
  /// frame E.
  /// @see ShiftInPlace() for more details.
  ///
  /// @param[in] p_PQ_E Vector from the original about point P to the new
  ///                   about point Q, expressed in the same frame E `this`
  ///                   spatial inertia is expressed in.
  /// @retval `M_SQ_E` This same spatial inertia for body or composite body S
  ///                  but computed about about a new point Q.
  SpatialInertia Shift(const Vector3<T>& p_PQ_E) const {
    return SpatialInertia(*this).ShiftInPlace(p_PQ_E);
  }

 private:
  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // Mass of the body or composite body.
  T mass_{nan()};
  // Position vector from point P to the center of mass of body or composite
  // body S, expressed in a frame E.
  Vector3<T> p_PScm_E_{Vector3<T>::Constant(nan())};
  // Rotational inertia of body or composite body S computed about point P and
  // expressed in a frame E.
  UnitInertia<T> G_SP_E_{};  // Defaults to NaN initialized inertia.

  // Checks that the SpatialInertia is physically valid and throws an
  // exception if not. This is mostly used in Debug builds to throw an
  // appropriate exception.
  void CheckInvariants() const {
    if (!IsPhysicallyValid()) {
      throw std::runtime_error(
          "The resulting spatial inertia is not physically valid. "
              "See SpatialInertia::IsPhysicallyValid()");
    }
  }
};

/// Insertion operator to write SpatialInertia objects into a `std::ostream`.
/// Especially useful for debugging.
/// @relates SpatialInertia
template <typename T> inline
std::ostream& operator<<(std::ostream& o,
                         const SpatialInertia<T>& M) {
  return o
      << " mass = " << M.get_mass() << std::endl
      << " com = [" << M.get_com().transpose() << "]ᵀ" << std::endl
      << " I = " << std::endl
      << M.CalcRotationalInertia();
}

}  // namespace multibody
}  // namespace drake
