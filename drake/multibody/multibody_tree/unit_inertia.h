#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

namespace drake {
namespace multibody {

/// This class is used to represent rotational inertias for unit mass bodies.
/// Therefore, unlike RotationalInertia whose units are kg⋅m², the units of a
/// %UnitInertia are those of length squared. A unit inertia is a useful concept
/// to represent the geometric distribution of mass in a body regardless of the
/// actual value of the body mass. The rotational inertia of a body can
/// therefore be obtained by multiplying its unit inertia by its mass.
/// Unit inertia matrices can also be called **gyration** matrices and therefore
/// we choose to represent them in source code notation with the capital letter
/// G. In contrast, the capital letter I is used to represent non-unit
/// mass rotational inertias.
/// This class restricts the set of allowed operations on a unit inertia to
/// ensure the unit-mass invariant. For instance, multiplication by a scalar can
/// only return a general RotationalInertia but not a %UnitInertia.
///
/// @note This class has no means to check at construction from user provided
/// parameters whether it actually represents the unit inertia or gyration
/// matrix of a unit-mass body. However, as previously noted, once a unit
/// inertia is created, a number of operations are dissallowed to ensure the
/// unit-mass invariant.
/// Also notice that once a unit inertia is created, it _is_ the unit inertia
/// of _some_ body, perhaps with scaled geometry from the user's intention.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
template <typename T>
class UnitInertia : public RotationalInertia<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UnitInertia)

  /// Default %UnitInertia constructor sets all entries to NaN for quick
  /// detection of uninitialized values.
  UnitInertia() {}

  /// Creates a unit inertia with moments of inertia `Ixx`, `Iyy`, `Izz`,
  /// and with each product of inertia set to zero.
  /// In debug builds, throws std::logic_error if unit inertia constructed from
  /// these arguments violates RotationalInertia::CouldBePhysicallyValid().
  UnitInertia(const T& Ixx, const T& Iyy, const T& Izz)
      : RotationalInertia<T>(Ixx, Iyy, Izz) {}

  /// Creates a unit inertia with moments of inertia `Ixx`, `Iyy`, `Izz`,
  /// and with products of inertia `Ixy`, `Ixz`, `Iyz`.
  /// In debug builds, throws std::logic_error if unit inertia constructed from
  /// these arguments violates RotationalInertia::CouldBePhysicallyValid().
  UnitInertia(const T& Ixx, const T& Iyy, const T& Izz,
              const T& Ixy, const T& Ixz, const T& Iyz)
      : RotationalInertia<T>(Ixx, Iyy, Izz, Ixy, Ixz, Iyz) {}

  /// Constructs a %UnitInertia from a RotationalInertia. This constructor has
  /// no way to verify that the input rotational inertia actually is a unit
  /// inertia. But the construction will nevertheless succeed, and the values of
  /// the input rotational inertia will henceforth be considered a valid unit
  /// inertia.
  /// It is the responsibility of the user to pass a valid unit inertia.
  explicit UnitInertia(const RotationalInertia<T>& I)
      : RotationalInertia<T>(I) {}

  /// Returns a new %UnitInertia object templated on `Scalar` initialized
  /// from the value of `this` unit inertia.
  ///
  /// @tparam Scalar The scalar type on which the new unit inertia will
  /// be templated.
  ///
  /// @note `UnitInertia<From>::cast<To>()` creates a new
  /// `UnitInertia<To>` from a `UnitInertia<From>` but only if
  /// type `To` is constructible from type `From`. As an example of this,
  /// `UnitInertia<double>::cast<AutoDiffXd>()` is valid since
  /// `AutoDiffXd a(1.0)` is valid. However,
  /// `UnitInertia<AutoDiffXd>::cast<double>()` is not.
  template <typename Scalar>
  UnitInertia<Scalar> cast() const {
    return UnitInertia<Scalar>(RotationalInertia<T>::template cast<Scalar>());
  }

  /// Sets `this` unit inertia from a generally non-unit inertia I corresponding
  /// to a body with a given `mass`.
  /// @note In Debug builds, this operation aborts if the provided `mass` is
  ///       not strictly positive.
  UnitInertia<T>& SetFromRotationalInertia(
      const RotationalInertia<T>& I, const T& mass) {
    DRAKE_ASSERT(mass > 0);
    RotationalInertia<T>::operator=(I / mass);
    return *this;
  }

  /// Re-express a unit inertia in a different frame, performing the operation
  /// in place and modifying the original object. @see ReExpress() for details.
  UnitInertia<T>& ReExpressInPlace(const Matrix3<T>& R_FE) {
    RotationalInertia<T>::ReExpressInPlace(R_FE);
    return *this;
  }

  /// Given `this` unit inertia `G_BP_E` of a body B about a point P and
  /// expressed in frame E, this method computes the same unit inertia
  /// re-expressed in another frame F as `G_BP_F = R_FE * G_BP_E * (R_FE)ᵀ`.
  /// @param[in] R_FE Rotation matrix from the basis of frame E to the basis
  ///                 of frame F.
  /// @retval G_BP_F The same unit inertia for body B about point P but now
  ///                re-expressed in frameF.
  /// @warning This method does not check whether the input matrix `R_FE`
  /// represents a valid rotation or not. It is the resposibility of users to
  /// provide valid rotation matrices.
  UnitInertia<T> ReExpress(const Matrix3<T>& R_FE) const {
    return UnitInertia<T>(RotationalInertia<T>::ReExpress(R_FE));
  }

  /// For a central unit inertia `G_Bcm_E` computed about a body's center of
  /// mass (or centroid) `Bcm` and expressed in a frame E, this method shifts
  /// this inertia using the parallel axis theorem to be computed about a
  /// point Q. This operation is performed in place, modifying the original
  /// object which is no longer a central inertia.
  /// @param[in] p_BcmQ_E A vector from the body's centroid `Bcm` to point Q
  ///                     expressed in the same frame E in which `this`
  ///                     inertia is expressed.
  /// @returns A reference to `this` unit inertia, which has now been taken
  ///          about point Q so can be written as `G_BQ_E`.
  UnitInertia<T>& ShiftFromCenterOfMassInPlace(const Vector3<T>& p_BcQ_E) {
    RotationalInertia<T>::operator+=(PointMass(p_BcQ_E));
    return *this;
  }

  /// Shifts this central unit inertia to a different point, and returns the
  /// result. See ShiftFromCenterOfMassInPlace() for details.
  /// @param[in] p_BcmQ_E A vector from the body's centroid `Bcm` to point Q
  ///                     expressed in the same frame E in which `this`
  ///                     inertia is expressed.
  /// @retval G_BQ_E This same unit inertia taken about a point Q instead of
  ///                the centroid `Bcm`.
  UnitInertia<T> ShiftFromCenterOfMass(
      const Vector3<T>& p_BcQ_E) const __attribute__((warn_unused_result)) {
    return UnitInertia<T>(*this).ShiftFromCenterOfMassInPlace(p_BcQ_E);
  }

  /// For the unit inertia `G_BQ_E` of a body or composite body B computed about
  /// a point Q and expressed in a frame E, this method shifts this inertia
  /// using the parallel axis theorem to be computed about the center of mass
  /// `Bcm` of B. This operation is performed in place, modifying the original
  /// object.
  ///
  /// @param[in] p_QBcm_E A position vector from the about point Q to the body's
  ///                     centroid `Bcm` expressed in the same frame E in which
  ///                     `this` inertia is expressed.
  /// @returns A reference to `this` unit inertia, which has now been taken
  ///          about point `Bcm` so can be written as `G_BBcm_E`, or `G_Bcm_E`.
  ///
  /// @warning This operation could result in a non-physical rotational inertia.
  /// The shifted inertia is obtained by subtracting the point mass unit inertia
  /// of point `Bcm` about point Q as:
  ///   G_Bcm_E = G_BQ_E - G_BcmQ_E = G_BQ_E - px_QBcm_E²
  /// Therefore the resulting inertia could have negative moments of inertia if
  /// the unit inertia of the unit mass at point `Bcm` is larger than `G_BQ_E`.
  /// Use with care.
  // TODO(mitiguy) Issue #6147.  If invalid inertia, should throw exception.
  UnitInertia<T>& ShiftToCenterOfMassInPlace(const Vector3<T>& p_QBcm_E) {
    RotationalInertia<T>::MinusEqualsUnchecked(PointMass(p_QBcm_E));
    return *this;
  }

  /// For the unit inertia `G_BQ_E` of a body or composite body B computed about
  /// a point Q and expressed in a frame E, this method shifts this inertia
  /// using the parallel axis theorem to be computed about the center of mass
  /// `Bcm` of B. See ShiftToCenterOfMassInPlace() for details.
  /// @param[in] p_QBcm_E A position vector from the about point Q to the body's
  ///                     centroid `Bcm` expressed in the same frame E in which
  ///                     `this` inertia is expressed.
  /// @retval G_Bcm_E This same unit which has now been taken about point `Bcm`
  ///                 so that it can be written as `G_BBcm_E`, or `G_Bcm_E`.
  ///
  /// @warning This operation could result in a non-physical rotational inertia.
  /// Use with care. See ShiftToCenterOfMassInPlace() for details.
  UnitInertia<T> ShiftToCenterOfMass(
      const Vector3<T>& p_QBcm_E) const __attribute__((warn_unused_result)) {
    return UnitInertia<T>(*this).ShiftToCenterOfMassInPlace(p_QBcm_E);
  }

  /// @name            Unit inertia for common 3D objects
  /// The following methods assist in the construction of %UnitInertia instances
  /// for common 3D objects such as boxes, spheres, rods and others.
  /// This method computes a %UnitInertia for body with unit mass, typically
  /// around its centroid, and in a frame aligned with its principal axes.
  /// To construct general %UnitInertia objects use these methods along with
  /// ShiftFromCenterOfMassInPlace() to move the point about which the inertia
  /// is computed and use ReExpress() to express in a different frame.
  /// A non-unit RotationalInertia is obtained by multiplying the generated
  /// %UnitInertia by a non-unit mass value.
  //@{

  /// Construct a unit inertia for a point mass of unit mass located at point Q,
  /// whose location in a frame F is given by the position vector `p_FQ`
  /// (that is, p_FoQ_F).
  /// The unit inertia `G_QFo_F` of point mass Q about the origin `Fo` of
  /// frame F and expressed in F for this unit mass point equals the square
  /// of the cross product matrix of `p_FQ`. In coordinate-free form:
  /// \f[
  ///   G^{Q/F_o} = (^Fp^Q_\times)^2 = (^Fp^Q_\times)^T \, ^Fp^Q_\times =
  ///               -^Fp^Q_\times \, ^Fp^Q_\times
  /// \f]
  /// where @f$ ^Fp^Q_\times @f$ is the cross product matrix of vector
  /// @f$ ^Fp^Q @f$. In source code the above expression is written as:
  /// <pre>
  ///   G_QFo_F = px_FQ² = px_FQᵀ * px_FQ = -px_FQ * px_FQ
  /// </pre>
  /// where `px_FQ` denotes the cross product matrix of the position vector
  /// `p_FQ` (expressed in F) such that the cross product with another vector
  /// `a` can be obtained as `px.cross(a) = px * a`. The cross product matrix
  /// `px` is skew-symmetric. The square of the cross product matrix is a
  /// symmetric matrix with non-negative diagonals and obeys the triangle
  /// inequality. Matrix `px²` can be used to compute the triple vector product
  /// as `-p x (p x a) = -p.cross(p.cross(a)) = px² * a`.
  static UnitInertia<T> PointMass(const Vector3<T>& p_FQ) {
    const Vector3<T> p2m = p_FQ.cwiseAbs2();
    const T mp0 = -p_FQ(0);
    const T mp1 = -p_FQ(1);
    return UnitInertia<T>(
        /*          Ixx,             Iyy,             Izz */
        p2m[1] + p2m[2], p2m[0] + p2m[2], p2m[0] + p2m[1],
        /*       Ixy,          Ixz,          Iyz */
        mp0 * p_FQ[1], mp0 * p_FQ[2], mp1 * p_FQ[2]);
  }

  /// Computes the unit inertia for a unit-mass solid sphere of uniform density
  /// and radius `r` taken about its center.
  static UnitInertia<T> SolidSphere(const T& r) {
    return UnitInertia<T>::TriaxiallySymmetric(T(0.4) * r * r);
  }

  /// Computes the unit inertia for a unit-mass hollow sphere of radius `r`
  /// consisting of an infinitesimally thin shell of uniform density.
  /// The unit inertia is taken about the center of the sphere.
  static UnitInertia<T> HollowSphere(const T& r) {
    return UnitInertia<T>::TriaxiallySymmetric(T(2)/T(3) * r * r);
  }

  /// Computes the unit inertia for a unit-mass solid box of uniform density
  /// taken about its geometric center. If one length is zero the inertia
  /// corresponds to that of a thin rectangular sheet. If two lengths are zero
  /// the inertia corresponds to that of a thin rod in the remaining direction.
  /// @param[in] Lx The length of the box edge in the principal x-axis.
  /// @param[in] Ly The length of the box edge in the principal y-axis.
  /// @param[in] Lz The length of the box edge in the principal z-axis.
  static UnitInertia<T> SolidBox(const T& Lx, const T& Ly, const T& Lz) {
    const T one_twelfth = T(1) / T(12);
    const T Lx2 = Lx * Lx, Ly2 = Ly * Ly, Lz2 = Lz * Lz;
    return UnitInertia(
        one_twelfth * (Ly2 + Lz2),
        one_twelfth * (Lx2 + Lz2),
        one_twelfth * (Lx2 + Ly2));
  }

  /// Computes the unit inertia for a unit-mass solid cube (a box with
  /// equal-sized sides) of uniform density taken about its geometric center.
  /// @param[in] L The length of each of the cube's sides.
  static UnitInertia<T> SolidCube(const T& L) {
    return SolidBox(L, L, L);
  }

  /// Computes the unit inertia for a unit-mass cylinder of uniform density
  /// oriented along the z-axis computed about its center.
  /// @param[in] r The radius of the cylinder.
  /// @param[in] L The length of the cylinder.
  static UnitInertia<T> SolidCylinder(const T& r, const T& L) {
    const T Iz = r * r / T(2);
    const T Ix = (T(3) * r * r + L * L) / T(12);
    return UnitInertia(Ix, Ix, Iz);
  }

  /// Computes the unit inertia for a unit-mass cylinder of uniform density
  /// oriented along the z-axis computed about a point at the center of
  /// its base.
  /// @param[in] r The radius of the cylinder.
  /// @param[in] L The length of the cylinder.
  static UnitInertia<T> SolidCylinderAboutEnd(const T& r, const T& L) {
    const T Iz = r * r / T(2);
    const T Ix = (T(3) * r * r + L * L) / T(12) + L * L / T(4);
    return UnitInertia(Ix, Ix, Iz);
  }

  /// Constructs a unit inertia with equal moments of inertia along its
  /// diagonal and with each product of inertia set to zero. This factory
  /// is useful for the unit inertia of a uniform-density sphere or cube.
  /// In debug builds, throws std::logic_error if I_triaxial is negative/NaN.
  /// @see UnitInertia::SolidSphere() and UnitInertia::SolidCube() for examples.
  // TODO(mitiguy) Per issue #6139  Update to ConstructTriaxiallySymmetric.
  static UnitInertia<T> TriaxiallySymmetric(const T& I_triaxial) {
    return UnitInertia<T>(
        RotationalInertia<T>::TriaxiallySymmetric(I_triaxial));
  }
  // End of Doxygen group
  //@}

  // Disable operators that may result in non-unit inertias
  // (these operators *are* defined in the RotationalInertia class).
  // Note: Certain methods such as the re-express and shift do not need to be
  // deleted (because they cannot produce non-unit inertias).
  // Disclaimer: Non-const methods in RotationalInertia need to be reconsidered
  // with respect to the UnitInertia subclass, and either = delete'd below,
  // or documented why they are allowable.
  // TODO(mitiguy) See issue #6109.  These deleted operators do not really
  //         protect a unit inertia from being non-unit. This code is broken.
  /// @name  Disable operators that may result in non-unit inertias.
  //@{
  UnitInertia<T>& operator+=(const RotationalInertia<T>&) = delete;
  UnitInertia<T>& operator-=(const RotationalInertia<T>&) = delete;
  UnitInertia<T>& operator*=(const T&) = delete;
  UnitInertia<T>& operator/=(const T&) = delete;
  // End of Doxygen group
  //@}
};

}  // namespace multibody
}  // namespace drake
