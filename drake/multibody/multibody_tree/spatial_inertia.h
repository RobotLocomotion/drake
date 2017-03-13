#pragma once

#include <memory>
#include <sstream>
#include <vector>

#include "drake/common/drake_assert.h"
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
/// could consist of a single body B or even of a collection of bodies.
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
  /// Default SpatialInertia constructor initializes mass, center of mass and
  /// rotational inertia to invalid NaN's for a quick detection of
  /// uninitialized values.
  SpatialInertia() {}

  /// Constructs a spatial inertia from a given mass, center of mass and
  /// rotational inertia. The center of mass is measured from an origin Bo, i.e.
  /// the center of mass is a vector from origin `Bo` to the body's center of
  /// mass `Bc`, `com = p_BoBc`. The rotational inertia is expected to be
  /// computed about the same origin `Bo`.
  /// Both center of mass `com` and rotational inertia `I` are expected to be
  /// expressed on a same frame `F`.
  /// @param[in] mass The mass of the body.
  /// @param[in] com The center of mass of the body measured from an origin `Bo`
  /// and expressed in a frame `F`.
  /// @param[in] I Rotational inertia of the body computed about origin `Bo` and
  /// expressed in a frame `F`.
  SpatialInertia(
      const T& mass, const Vector3<T>& com, const UnitInertia<T>& I) :
      mass_(mass), p_BoBc_F_(com), I_Bo_F_(mass * I) {
    DRAKE_ASSERT(IsPhysicallyValid());
  }

  /// Get a constant reference to the mass of this spatial inertia.
  const T& get_mass() const { return mass_;}

  /// Get a constant reference to the center of mass vector of this spatial
  /// inertia.
  const Vector3<T>& get_com() const { return p_BoBc_F_;}

  /// Get a constant reference to the rotational inertia of this spatial
  /// inertia.
  const RotationalInertia<T>& get_rotational_inertia() const { return I_Bo_F_;}

  /// Returns `true` if any of the elements in this spatial inertia is NaN
  /// and `false` otherwise.
  bool IsNaN() const {
    using std::isnan;
    if (isnan(mass_)) return true;
    if (I_Bo_F_.IsNaN()) return true;
    if (p_BoBc_F_.array().isNaN().any()) return true;
    return false;
  }

  /// Performs a number of checks to verify that this is a physically valid
  /// spatial inertia.
  /// The checks performed are:
  /// - No NaN entries.
  /// - Positive mass.
  /// - Valid rotational inertia,
  /// @see RotationalInertia::IsPhysicallyValid().
  bool IsPhysicallyValid() const {
    if (IsNaN()) return false;
    if (mass_ < T(0)) return false;
    // The tests in RotationalInertia become a sufficient condition when
    // performed on a rotational inertia computed about a body's center of mass.
    RotationalInertia<T> I_Bcm_F = I_Bo_F_;// - mass_ * UnitInertia<T>::PointMass(p_BoBc_F_);
    if (!I_Bcm_F.CouldBePhysicallyValid()) return false;
    return true;  // All tests passed.
  }

#if 0
  // Default copy constructor and copy assignment.
  SpatialInertia(const SpatialInertia<T>& other) = default;
  SpatialInertia& operator=(const SpatialInertia<T>& other) = default;

  /// Get a copy to a full Matrix3 representation for this rotational inertia
  /// including both lower and upper triangular parts.
  Matrix6<T> CopyToFullMatrix6() const {
    using math::CrossProductMatrix;
    Matrix6<T> M;
    M.template block<3, 3>(0, 0) = I_Bo_F_.CopyToFullMatrix3();
    M.template block<3, 3>(0, 3) = mass_ * drake::math::VectorToSkewSymmetric(p_BoBc_F_);
    M.template block<3, 3>(3, 0) = -M.template block<3, 3>(0, 3);
    M.template block<3, 3>(3, 3) = mass_ * Matrix3<T>::Identity();
    return M;
  }

  /// Sets this spatial inertia to have NaN entries. Typically used to quickly
  /// detect uninitialized values since NaN will trigger a chain of invalid
  /// computations that then can be tracked to the source.
  void SetToNaN() {
    mass_ = nan();
    p_BoBc_F_.setConstant(nan());
    I_Bo_F_.SetToNaN();
  }

  bool IsApprox(const SpatialInertia& M_Bo_F,
                double tolerance = Eigen::NumTraits<T>::epsilon()) {
    using std::abs;
    return
        abs(mass_ - M_Bo_F.get_mass()) < tolerance &&
        p_BoBc_F_.isApprox(M_Bo_F.get_com(), tolerance) &&
        I_Bo_F_.IsApprox(M_Bo_F.get_rotational_inertia(), tolerance);
  }

  /// Adds spatial inertia @p `M_Bo_F` to this spatial inertia. This operation 
  /// is only valid if both inertias are computed about the same center `Bo`
  /// and expressed in the same frame `F`.
  /// @param[in] M_Bo_F A spatial inertia to be added to this inertia.
  /// @returns A reference to `this` spatial inetia.
  SpatialInertia& operator+=(const SpatialInertia<T>& M_Bo_F) {
    p_BoBc_F_ = get_mass() * get_com() + M_Bo_F.get_mass() * M_Bo_F.get_com();
    mass_ += M_Bo_F.get_mass();
    p_BoBc_F_ /= mass_;
    I_Bo_F_ += M_Bo_F.I_Bo_F_;
    return *this;
  }

  /// Computes the product from the right between this spatial inertia with the
  /// spatial vector @p V. This spatial inertia and spatial vector @p V must be
  /// expressed in the same frame.
  /// @param[in] V Spatial vector to multiply from the right.
  /// @returns The product from the right of `this` inertia with @p V.
  SpatialVector<T> operator*(const SpatialVector<T>& V) const
  {
    const auto& v = V.linear();   // Linear velocity.
    const auto& w = V.angular();  // Angular velocity.
    const Vector3<T> mxp = mass_ * p_BoBc_F_;
    return SpatialVector<T>(
        I_Bo_F_ * w + mxp.cross(v), /* angular component */
        mass_ * v - mxp.cross(w));  /* linear component */
  }

  /// Given this spatial inertia `M_Bo_F` about `Bo` and expressed in frame `F`,
  /// this method re-expresses the same spatial inertia in another frame `A`.
  /// This operation is performed in-place modifying the original object.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns A references to `this` object which now is the same spatial 
  /// inertia about `Bo` but expressed in frame `A`.
  SpatialInertia& ReExpressInPlace(const Matrix3<T>& R_AF) {
    p_BoBc_F_ = R_AF * p_BoBc_F_;
    I_Bo_F_.ReExpressInPlace(R_AF);
    return *this;
  }

  /// Given this spatial inertia `M_Bo_F` about `Bo` and expressed in frame `F`,
  /// this method computes the same spatial inertia but re-expressed in another
  /// frame `A`.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns M_Bo_A The same spatial inertia about `Bo` but expressed in
  /// frame `A`.
  SpatialInertia ReExpress(const Matrix3<T>& R_AF) const {
    return SpatialInertia(*this).ReExpressInPlace(R_AF);
  }

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
    using math::CrossProductMatrixSquared;
    const Vector3<T> p_XoBc_F = p_BoBc_F_ - p_BoXo_F;
    const Matrix3<T> Sp_BoBc_F = CrossProductMatrixSquared(p_BoBc_F_);
    const Matrix3<T> Sp_XoBc_F = CrossProductMatrixSquared(p_XoBc_F);
    I_Bo_F_.get_mutable_symmetric_matrix_view() =
        I_Bo_F_.get_matrix() + mass_ * (Sp_BoBc_F - Sp_XoBc_F);
    p_BoBc_F_ = p_XoBc_F;
    return *this;
  }

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

  // Mass of the body.
  T mass_{nan()};
  // Center of mass measured in X and expressed in Y. Typically X is the body
  // frame and Y is the world frame.
  Vector3<T> p_BoBc_F_{Vector3<T>::Constant(nan())};
  // Rotational inertia about Xo and expressed in Y.
  RotationalInertia<T> I_Bo_F_{};  // Defaults to NaN initialized inertia.
};

#if 0
template <typename T>
inline SpatialInertia<T> operator*(
    const T& s, const SpatialInertia<T>& I_Bo_F) {
  return SpatialInertia<T>(s * I_Bo_F.get_mass(),
                           s * I_Bo_F.get_com(),
                           s * I_Bo_F.get_rotational_inertia());
}

template <typename T> inline
std::ostream& operator<<(std::ostream& o,
                         const SpatialInertia<T>& M) {
  return o
      << " mass = " << M.get_mass() << std::endl
      << " com = [" << M.get_com().transpose() << "]^T" << std::endl
      << " I = " << std::endl
      << M.get_rotational_inertia();
}
#endif

}  // namespace multibody
}  // namespace drake
