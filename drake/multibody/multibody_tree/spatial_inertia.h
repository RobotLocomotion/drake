#pragma once

#include <exception>
#include <memory>
#include <sstream>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/cross_product.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/unit_inertia.h"

#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

/// This class represents the physical concept of a _Spatial Inertia_. A
/// spatial inertia (or spatial mass matrix) encapsulates the mass, center of
/// mass, and rotational inertia of the mass distribution of a system S which
/// could consist of a single body B or even of a collection of bodies
/// (throughout this documentation "body" is many times used instead of "system"
/// but the same concepts apply to a system of bodies as well.)
/// As an element of ℛ⁶ˣ⁶ it is a symmetric, positive definite matrix that
/// logically consists of `3x3` sub-matrices arranged like so:
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
/// @ref multibody_spatial_inertia, `I_SP` is the rotational inertia of system
/// S computed about a point P, m is the mass of this system, `p_PBcm` is the
/// position vector from point P to the center of mass `Scm` of system with
/// `p_PScm×` denoting its skew-symmetric cross product matrix, and `Id` is the
/// identity matrix in ℛ³ˣ³.
///
/// In typeset material we use the symbol @f$ [M^{S/P}]_E @f$ to represent the
/// spatial inertia of a system S about point P, expressed in frame E. For
/// this inertia, the monogram notation reads `M_SP_E`. If the point P is fixed
/// to a body B, we write that point as @f$ B_P @f$ which appears in code and
/// comments as `Bp`. So if the system is a body B and the about point is `Bp`,
/// the monogram notation reads `M_BBp_E`, which can be abbreviated to `M_Bp_E`
/// since the about point `Bp` also identifies the system. Common cases are that
/// the about point is the origin `Bo` of the body, or its the center of mass
/// `Bcm` for which the rotational inertia in monogram notation would read as
/// `I_Bo_E` and `I_Bcm_E`, respectively.
/// Given `M_BP_E` (@f$[M^{B/P}]_E@f$), its rotational inertia is `I_BP_E`
/// (@f$[I^{B/P}]_E@f$) and the position vector of the center of mass measured
/// from point P and expressed in E is `p_PBcm_E` (@f$[^Pp^{B_{cm}}]_E@f$).
///
/// @note This class does not implement any mechanism to track the frame E in
/// which a spatial inertia is expressed or about what point is computed.
/// Methods and operators on this class have no means to determine frame
/// consistency through operations. It is therefore the responsibility of users
/// of this class to keep track of frames in which operations are performed. The
/// best way to do that is to use a disciplined notation as described below.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class SpatialInertia {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialInertia)

  /// Default SpatialInertia constructor initializes mass, center of mass and
  /// rotational inertia to invalid NaN's for a quick detection of
  /// uninitialized values.
  SpatialInertia() {}

  /// Constructs a spatial for a physical system S about a point P from a given
  /// mass, center of mass and rotational inertia. The center of mass is
  /// specified by the position vector `p_PScm_E` from point P to the systems's
  /// center of mass point `Scm`, expressed in a frame E.
  /// The rotational inertia is provided as the UnitInertia `G_SP_E` of system S
  /// computed about point P and expressed in frame E.
  ///
  /// This constructor checks for the physical validity of the resulting
  /// %SpatialInertia with IsPhysicallyValid() and throws an exception in the
  /// event the provided input parameters lead to non-physically viable spatial
  /// inertia.
  ///
  /// @param[in] mass The mass of the physical system or body S.
  /// @param[in] p_PScm_E The position vector from point P to the center of mass
  ///                     of system or body S expressed in frame E.
  /// @param[in] G_SP_E UnitInertia of the system or body S computed about
  ///                   origin point P and expressed in frame E.
  SpatialInertia(
      const T& mass, const Vector3<T>& p_PScm_E, const UnitInertia<T>& G_SP_E) :
      mass_(mass), p_PScm_E_(p_PScm_E), I_SP_E_(mass * G_SP_E) {
    if (!IsPhysicallyValid()) {
      throw std::runtime_error(
          "The resulting spatial inertia is not physically valid."
          "See SpatialInertia::IsPhysicallyValid()");
    }
  }

  /// Get a constant reference to the mass of this spatial inertia.
  const T& get_mass() const { return mass_;}

  /// Get a constant reference to the position vector `p_PScm_E` from the
  /// _about point_ P to the center of mass `Scm` of the physical system S,
  /// expressed in frame E. See the documentation of this clas for details.
  const Vector3<T>& get_com() const { return p_PScm_E_;}

  /// Get a constant reference to the rotational inertia `I_SP_E` of this
  /// spatial inertia, computed about point P and expressed in frame E. See the
  /// documentation of this clas for details.
  const RotationalInertia<T>& get_rotational_inertia() const { return I_SP_E_;}

  /// Returns `true` if any of the elements in this spatial inertia is NaN
  /// and `false` otherwise.
  bool IsNaN() const {
    using std::isnan;
    if (isnan(mass_)) return true;
    if (I_SP_E_.IsNaN()) return true;
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
  /// @see RotationalInertia::CouldBePhysicallyValid().
  bool IsPhysicallyValid() const {
    if (IsNaN()) return false;
    if (mass_ < T(0)) return false;
    // The tests in RotationalInertia become a sufficient condition when
    // performed on a rotational inertia computed about a body's center of mass.
    RotationalInertia<T> I_SScm_E(I_SP_E_);
    I_SScm_E -= mass_ * UnitInertia<T>::PointMass(p_PScm_E_);
    if (!I_SScm_E.CouldBePhysicallyValid()) return false;
    return true;  // All tests passed.
  }

  /// Copy to a full 6x6 matrix representation.
  Matrix6<T> CopyToFullMatrix6() const {
    using drake::math::VectorToSkewSymmetric;
    Matrix6<T> M;
    M.template block<3, 3>(0, 0) = I_SP_E_.CopyToFullMatrix3();
    M.template block<3, 3>(0, 3) = mass_ * VectorToSkewSymmetric(p_PScm_E_);
    M.template block<3, 3>(3, 0) = -M.template block<3, 3>(0, 3);
    M.template block<3, 3>(3, 3) = mass_ * Matrix3<T>::Identity();
    return M;
  }

  /// Sets `this` spatial inertia to have NaN entries. Typically used for the
  /// quick detection of uninitialized values.
  void SetNaN() {
    mass_ = nan();
    p_PScm_E_.setConstant(nan());
    I_SP_E_.SetToNaN();
  }

  /// Compares `this` spatial inertia to `other` rotational inertia within the
  /// specified `tolerance`.
  /// The comparsion returns `true` if the following are true:
  ///   - abs(this->get_mass() - other.get_mass()) < tolerance.
  ///   - this->get_com().isApprox(other.get_com(), tolerance).
  ///   - this->get_rotational_inertia().IsApprox(
  ///               other.get_rotational_inertia(), tolerance()).
  ///
  /// @returns `true` if `other` is within the specified `precision`. Returns
  ///   `false` otherwise.
  bool IsApprox(const SpatialInertia& other,
                double tolerance = Eigen::NumTraits<T>::epsilon()) {
    using std::abs;
    return
        abs(mass_ - other.get_mass()) < tolerance &&
        p_PScm_E_.isApprox(other.get_com(), tolerance) &&
        I_SP_E_.IsApprox(other.get_rotational_inertia(), tolerance);
  }

  /// Adds in a spatial inertia to `this` spatial inertia. This operation is
  /// only valid if both spatial inertias are computed about the same point `P`
  /// and expressed in the same frame `E`. Considering `this` spatial inertia to
  /// be `M_SP_E` for some system `S`, taken about some point `P`, the supplied
  /// spatial inertia `M_BP_E` must be for some system `B` taken about the
  /// _same_ point `P`; `B`'s inertia is then included in `S`.
  /// @param[in] M_BP_E A spatial inertia of some body `B` to be added to
  ///                  `this` spatial inertia. It must have been taken about the
  ///                   same point `P` as `this` inertia, and expressed in the
  ///                   same frame `E`.
  /// @returns A reference to `this` spatial inertia, which has been updated
  ///          to include the given spatial inertia `M_BP_E`.
  SpatialInertia& operator+=(const SpatialInertia<T>& M_BP_E) {
    p_PScm_E_ = get_mass() * get_com() + M_BP_E.get_mass() * M_BP_E.get_com();
    mass_ += M_BP_E.get_mass();
    p_PScm_E_ /= get_mass();
    I_SP_E_ += M_BP_E.get_rotational_inertia();
    return *this;
  }

  /// Given `this` spatial inertia `M_SP_E` for some system or body S, taken
  /// about a point P and expressed in frame E, this method computes the same
  /// inertia re-expressed in another frame A.
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] R_AE Rotation matrix from frame E to frame A.
  /// @returns A reference to `this` rotational inertia about the same point P
  ///          but now re-expressed in frame `A`, that is, `M_SP_A`.
  SpatialInertia& ReExpressInPlace(const Matrix3<T>& R_AE) {
    p_PScm_E_ = R_AE * p_PScm_E_;    // Now p_PScm_A
    I_SP_E_.ReExpressInPlace(R_AE);  // Now I_SP_A
    return *this;                    // Now M_SP_A
  }

  /// Given `this` spatial inertia `M_SP_E` for some system or body S, taken
  /// about a point P and expressed in frame E, this method computes the same
  /// inertia re-expressed in another frame A.
  ///
  /// @param[in] R_AE Rotation matrix from frame E to frame A.
  /// @retval M_SP_A The same spatial inertia of S about P but now
  ///                re-expressed in frame`A`.
  /// @see ReExpressInPlace()
  SpatialInertia ReExpress(const Matrix3<T>& R_AE) const {
    return SpatialInertia(*this).ReExpressInPlace(R_AE);
  }

#if 0
  /// Computes the product from the right between this spatial inertia with the
  /// spatial vector @p V. This spatial inertia and spatial vector @p V must be
  /// expressed in the same frame.
  /// @param[in] V Spatial vector to multiply from the right.
  /// @returns The product from the right of `this` inertia with @p V.
  SpatialVector<T> operator*(const SpatialVector<T>& V) const
  {
    const auto& v = V.linear();   // Linear velocity.
    const auto& w = V.angular();  // Angular velocity.
    const Vector3<T> mxp = mass_ * p_PScm_E_;
    return SpatialVector<T>(
        I_SP_E_ * w + mxp.cross(v), /* angular component */
        mass_ * v - mxp.cross(w));  /* linear component */
  }
#endif

  /// This methods perfomrs the "parallel axis theorem" for spatial inertias:
  /// given this spatial inertia `M_Bo_F` about `Bo` and expressed in frame `F`,
  /// this method modifies this spatial inertia to be computed about a new
  /// origin Xo. The result still is expressed in frame `F`.
  /// This operation is performed in-place modifying the original object.
  /// @see Shift() which does not modify this object.
  ///
  /// See Section 2.1.2, p. 20 of A. Jain's book.
  ///
  /// @param[in] p_BoXo_F Vector from the original origin `Bo` to the new origin
  /// `Xo`, expressed in the spatial inertia frame `F`.
  /// @returns `M_Xo_F` This same spatial inertia but computed about
  /// origin `Xo`.
  SpatialInertia& ShiftInPlace(const Vector3<T>& p_BoXo_F) {
    const Vector3<T> p_XoBc_F = p_PScm_E_ - p_BoXo_F;
    I_SP_E_ += get_mass() *
        (UnitInertia<T>::PointMass(p_XoBc_F) -
         UnitInertia<T>::PointMass(p_PScm_E_));
    p_PScm_E_ = p_XoBc_F;
    return *this;
  }

#if 0
  /// This methods perfomrs the "parallel axis theorem" for spatial inertias:
  /// given this spatial inertia `M_Bo_F` about `Bo` and expressed in frame `F`,
  /// this method returns this spatial inertia to but computed about a new
  /// origin Xo. The result still is expressed in frame `F`.
  /// @see ShiftInPlace() for the in-place operation.
  ///
  /// See Section 2.1.2, p. 20 of A. Jain's book.
  ///
  /// @param[in] p_BoXo_F Vector from the original origin `Bo` to the new origin
  /// `Xo`, expressed in the spatial inertia frame `F`.
  /// @returns `M_Xo_F` This same spatial inertia but computed about
  /// origin `Xo`.
  SpatialInertia Shift(const Vector3<T>& p_BoXo_F) const {
    return SpatialInertia(*this).ShiftInPlace(p_BoXo_F);
  }
#endif

 private:
  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // Mass of the system.
  T mass_{nan()};
  // Position vector from point P to the center of mass of physical system S,
  // expressed in a frame E.
  Vector3<T> p_PScm_E_{Vector3<T>::Constant(nan())};
  // Rotational inertia of physical system S computed about point P and
  // expressed in a frame E.
  RotationalInertia<T> I_SP_E_{};  // Defaults to NaN initialized inertia.
};

#if 0
template <typename T>
inline SpatialInertia<T> operator*(
    const T& s, const SpatialInertia<T>& I_Bo_F) {
  return SpatialInertia<T>(s * I_Bo_F.get_mass(),
                           s * I_Bo_F.get_com(),
                           s * I_Bo_F.get_rotational_inertia());
}
#endif

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
      << M.get_rotational_inertia();
}

}  // namespace multibody
}  // namespace drake
