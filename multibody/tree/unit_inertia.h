#pragma once

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/rotational_inertia.h"

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
/// inertia is created, a number of operations are disallowed to ensure the
/// unit-mass invariant.
/// Also notice that once a unit inertia is created, it _is_ the unit inertia
/// of _some_ body, perhaps with scaled geometry from the user's intention.
///
/// @note The methods of this class satisfy the "basic exception guarantee": if
/// an exception is thrown, the program will still be in a valid
/// state. Specifically, no resources are leaked, and all objects' invariants
/// are intact. Be aware that UnitInertia objects may contain invalid inertia
/// data in cases where input checking is skipped.
/// @see https://en.cppreference.com/w/cpp/language/exceptions
///
/// @tparam_default_scalar
template <typename T>
class UnitInertia : public RotationalInertia<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UnitInertia);

  /// Default %UnitInertia constructor sets all entries to NaN for quick
  /// detection of uninitialized values.
  UnitInertia() {}

  /// Creates a unit inertia with moments of inertia `Ixx`, `Iyy`, `Izz`,
  /// and with each product of inertia set to zero.
  /// In debug builds, throws std::exception if unit inertia constructed from
  /// these arguments violates RotationalInertia::CouldBePhysicallyValid().
  UnitInertia(const T& Ixx, const T& Iyy, const T& Izz)
      : RotationalInertia<T>(Ixx, Iyy, Izz) {}

  /// Creates a unit inertia with moments of inertia `Ixx`, `Iyy`, `Izz`,
  /// and with products of inertia `Ixy`, `Ixz`, `Iyz`.
  /// In debug builds, throws std::exception if unit inertia constructed from
  /// these arguments violates RotationalInertia::CouldBePhysicallyValid().
  UnitInertia(const T& Ixx, const T& Iyy, const T& Izz, const T& Ixy,
              const T& Ixz, const T& Iyz)
      : RotationalInertia<T>(Ixx, Iyy, Izz, Ixy, Ixz, Iyz) {}

  /// Constructs a %UnitInertia from a RotationalInertia. This constructor has
  /// no way to verify that the input rotational inertia actually is a unit
  /// inertia. But the construction will nevertheless succeed, and the values of
  /// the input rotational inertia will henceforth be considered a valid unit
  /// inertia.
  /// @pre The user is responsible for passing a valid rotational inertia.
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
  /// @throws std::exception if the provided `mass` is not strictly positive.
  UnitInertia<T>& SetFromRotationalInertia(const RotationalInertia<T>& I,
                                           const T& mass) {
    DRAKE_THROW_UNLESS(mass > 0);
    RotationalInertia<T>::operator=(I / mass);
    return *this;
  }

  /// Re-express a unit inertia in a different frame, performing the operation
  /// in place and modifying the original object. @see ReExpress() for details.
  void ReExpressInPlace(const math::RotationMatrix<T>& R_AE) {
    RotationalInertia<T>::ReExpressInPlace(R_AE);  // 57 flops
  }

  /// Given `this` unit inertia `G_BP_E` of a body B about a point P and
  /// expressed in frame E, this method computes the same unit inertia
  /// re-expressed in another frame A as `G_BP_A = R_AE * G_BP_E * (R_AE)ᵀ`.
  /// @param[in] R_AE RotationMatrix relating frames A and E.
  /// @retval G_BP_A The same unit inertia for body B about point P but now
  ///                re-expressed in frame A.
  UnitInertia<T> ReExpress(const math::RotationMatrix<T>& R_AE) const {
    return UnitInertia<T>(RotationalInertia<T>::ReExpress(R_AE));
  }

  /// For a central unit inertia `G_Bcm_E` computed about a body's center of
  /// mass (or centroid) `Bcm` and expressed in a frame E, this method shifts
  /// this inertia using the parallel axis theorem to be computed about a
  /// point Q. This operation is performed in place, modifying the original
  /// object which is no longer a central inertia. On return, `this` is
  /// modified to be taken about point Q so can be written as `G_BQ_E`.
  /// @param[in] p_BcmQ_E A vector from the body's centroid `Bcm` to point Q
  ///                     expressed in the same frame E in which `this`
  ///                     inertia is expressed.
  void ShiftFromCenterOfMassInPlace(const Vector3<T>& p_BcmQ_E) {
    RotationalInertia<T>::operator+=(PointMass(p_BcmQ_E));  // 17 flops
  }

  /// Shifts this central unit inertia to a different point, and returns the
  /// result. See ShiftFromCenterOfMassInPlace() for details.
  /// @param[in] p_BcmQ_E A vector from the body's centroid `Bcm` to point Q
  ///                     expressed in the same frame E in which `this`
  ///                     inertia is expressed.
  /// @retval G_BQ_E This same unit inertia taken about a point Q instead of
  ///                the centroid `Bcm`.
  [[nodiscard]] UnitInertia<T> ShiftFromCenterOfMass(
      const Vector3<T>& p_BcmQ_E) const {
    UnitInertia<T> result(*this);
    result.ShiftFromCenterOfMassInPlace(p_BcmQ_E);
    return result;
  }

  /// For the unit inertia `G_BQ_E` of a body or composite body B computed about
  /// a point Q and expressed in a frame E, this method shifts this inertia
  /// using the parallel axis theorem to be computed about the center of mass
  /// `Bcm` of B. This operation is performed in place, modifying the original
  /// object. On return, `this` is modified to be taken about point `Bcm` so can
  /// be written as `G_BBcm_E`, or `G_Bcm_E`.
  ///
  /// @param[in] p_QBcm_E A position vector from the about point Q to the body's
  ///                     centroid `Bcm` expressed in the same frame E in which
  ///                     `this` inertia is expressed.
  ///
  /// @warning This operation could result in a non-physical rotational inertia.
  /// The shifted inertia is obtained by subtracting the point mass unit inertia
  /// of point `Bcm` about point Q as:
  ///   G_Bcm_E = G_BQ_E - G_BcmQ_E = G_BQ_E - px_QBcm_E²
  /// Therefore the resulting inertia could have negative moments of inertia if
  /// the unit inertia of the unit mass at point `Bcm` is larger than `G_BQ_E`.
  /// Use with care.
  // TODO(mitiguy) Issue #6147.  If invalid inertia, should throw exception.
  void ShiftToCenterOfMassInPlace(const Vector3<T>& p_QBcm_E) {
    // 17 flops
    RotationalInertia<T>::MinusEqualsUnchecked(PointMass(p_QBcm_E));
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
  [[nodiscard]] UnitInertia<T> ShiftToCenterOfMass(
      const Vector3<T>& p_QBcm_E) const {
    UnitInertia<T> result(*this);
    result.ShiftToCenterOfMassInPlace(p_QBcm_E);  // 17 flops
    return result;
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
    // Square each coefficient in p_FQ, better with p_FQ.array().square()?
    const Vector3<T> p2m = p_FQ.cwiseAbs2();  // [x²  y²  z²].
    const T mp0 = -p_FQ(0);                   // -x
    const T mp1 = -p_FQ(1);                   // -y
    return UnitInertia<T>(
        // Gxx = y² + z²,  Gyy = x² + z²,  Gzz = x² + y²
        p2m[1] + p2m[2], p2m[0] + p2m[2], p2m[0] + p2m[1],
        // Gxy = -x y,  Gxz = -x z,   Gyz = -y z
        mp0 * p_FQ[1], mp0 * p_FQ[2], mp1 * p_FQ[2]);
    // 11 flops total, hence should be inlined.
  }

  /// Computes the unit inertia for a unit-mass solid ellipsoid of uniform
  /// density taken about its center. The lengths of the semi-axes of the
  /// ellipsoid in the principal x,y,z-axes are `a`, `b`, and `c` respectively.
  static UnitInertia<T> SolidEllipsoid(const T& a, const T& b, const T& c);

  /// Computes the unit inertia for a unit-mass solid sphere of uniform density
  /// and radius `r` taken about its center.
  static UnitInertia<T> SolidSphere(const T& r) {
    return UnitInertia<T>::TriaxiallySymmetric(0.4 * r * r);
  }

  /// Computes the unit inertia for a unit-mass hollow sphere of radius `r`
  /// consisting of an infinitesimally thin shell of uniform density.
  /// The unit inertia is taken about the center of the sphere.
  static UnitInertia<T> HollowSphere(const T& r) {
    return UnitInertia<T>::TriaxiallySymmetric(2.0 / 3.0 * r * r);
  }

  /// Computes the unit inertia for a unit-mass solid box of uniform density
  /// taken about its geometric center. If one length is zero the inertia
  /// corresponds to that of a thin rectangular sheet. If two lengths are zero
  /// the inertia corresponds to that of a thin rod in the remaining direction.
  /// @param[in] Lx The length of the box edge in the principal x-axis.
  /// @param[in] Ly The length of the box edge in the principal y-axis.
  /// @param[in] Lz The length of the box edge in the principal z-axis.
  /// @throws std::exception if any of Lx, Ly, Lz are negative.
  static UnitInertia<T> SolidBox(const T& Lx, const T& Ly, const T& Lz);

  /// Computes the unit inertia for a unit-mass solid cube (a box with
  /// equal-sized sides) of uniform density taken about its geometric center.
  /// @param[in] L The length of each of the cube's sides.
  static UnitInertia<T> SolidCube(const T& L) { return SolidBox(L, L, L); }

  /// Creates a unit inertia for a uniform density solid cylinder B about
  /// its center of mass Bcm (which is coincident with B's geometric center Bo).
  /// @param[in] radius radius of the cylinder (meters).
  /// @param[in] length length of cylinder in unit_vector direction (meters).
  /// @param[in] unit_vector unit vector defining the axial direction of the
  /// cylinder, expressed in a frame E.
  /// @retval G_BBcm_E B's unit inertia about Bcm expressed in the same frame E
  /// as the unit_vector is expressed.
  /// @note B's unit inertia about Bcm is axially symmetric, meaning B has an
  /// equal moment of inertia about any line that both passes through Bcm and
  /// is perpendicular to unit_vector.
  /// @throws std::exception if radius or length is negative or if
  /// ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  /// @see SolidCylinderAboutEnd() to calculate G_BBp_E, B's unit inertia about
  /// point Bp (Bp is at the center of one of the cylinder's circular ends).
  static UnitInertia<T> SolidCylinder(const T& radius, const T& length,
                                      const Vector3<T>& unit_vector);

  /// Creates a unit inertia for a uniform density solid capsule B about
  /// its center of mass Bcm (which is coincident with B's geometric center Bo).
  /// @param[in] radius radius of the cylinder/half-sphere parts of the capsule.
  /// @param[in] length length of cylindrical part of the capsule.
  /// @param[in] unit_vector unit vector defining the axial direction of the
  /// cylindrical part of the capsule, expressed in a frame E.
  /// @retval G_BBcm_E B's unit inertia about Bcm expressed in the same frame E
  /// as the unit_vector is expressed.
  /// @note B's unit inertia about Bcm is axially symmetric, meaning B has an
  /// equal moment of inertia about any line that both passes through Bcm and
  /// is perpendicular to unit_vector.
  /// @throws std::exception if radius or length is negative or if
  /// ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  static UnitInertia<T> SolidCapsule(const T& radius, const T& length,
                                     const Vector3<T>& unit_vector);

  /// Creates a unit inertia for a uniform-density solid cylinder B about an
  /// end-point Bp of the cylinder's axis (see below for more about Bp).
  /// @param[in] radius radius of cylinder (meters).
  /// @param[in] length length of cylinder in unit_vector direction (meters).
  /// @param[in] unit_vector unit vector parallel to the axis of the cylinder
  /// and directed from Bp to Bcm (B's center of mass), expressed in a frame E.
  /// @retval G_BBp_E B's unit inertia about Bp expressed in the same frame E
  /// as the unit_vector is expressed.
  /// @note The position from Bp to Bcm is p_BpBcm = length / 2 * unit_vector.
  /// @note B's unit inertia about Bp is axially symmetric, meaning B has an
  /// equal moment of inertia about any line that both passes through Bp and
  /// is perpendicular to unit_vector.
  /// @throws std::exception if radius or length is negative or if
  /// ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  static UnitInertia<T> SolidCylinderAboutEnd(const T& radius, const T& length,
                                              const Vector3<T>& unit_vector);

  /// Creates a unit inertia for a unit-mass uniform density solid tetrahedron B
  /// about a point A, from which position vectors to B's 4 vertices B0, B1, B2,
  /// B3 are measured (position vectors are all expressed in a common frame E).
  /// @param[in] p0 position vector p_AB0_E from point A to B0, expressed in E.
  /// @param[in] p1 position vector p_AB1_E from point A to B1, expressed in E.
  /// @param[in] p2 position vector p_AB2_E from point A to B2, expressed in E.
  /// @param[in] p3 position vector p_AB3_E from point A to B3, expressed in E.
  /// @retval G_BA_E B's unit inertia about point A, expressed in E.
  /// @see SolidTetrahedronAboutVertex() to efficiently calculate a unit inertia
  /// about a vertex of B.
  static UnitInertia<T> SolidTetrahedronAboutPoint(const Vector3<T>& p0,
                                                   const Vector3<T>& p1,
                                                   const Vector3<T>& p2,
                                                   const Vector3<T>& p3);

  /// (Advanced) Creates a unit inertia for a unit-mass uniform density solid
  /// tetrahedron B about its vertex B0, from which position vectors to B's
  /// other 3 vertices B1, B2, B3 are measured (vectors are all expressed
  /// in a common frame E).
  /// @param[in] p1 position vector p_B0B1_E from B0 to B1, expressed in E.
  /// @param[in] p2 position vector p_B0B2_E from B0 to B2, expressed in E.
  /// @param[in] p3 position vector p_B0B3_E from B0 to B3, expressed in E.
  /// @retval G_BB0_E B's unit inertia about its vertex B0, expressed in E.
  /// @see SolidTetrahedronAboutPoint() to calculate a unit inertia about an
  /// arbitrary point.
  static UnitInertia<T> SolidTetrahedronAboutVertex(const Vector3<T>& p1,
                                                    const Vector3<T>& p2,
                                                    const Vector3<T>& p3);

  /// Returns the unit inertia for a body B for which there exists an axis L
  /// passing through the body's center of mass Bcm having the property that
  /// B's moment of inertia about all lines perpendicular to L are equal.
  /// Bodies with this "axially symmetric inertia" property include axisymmetric
  /// cylinders or cones and propellers with 3⁺ evenly spaced blades.
  /// For a body B with axially symmetric inertia, B's unit inertia about a
  /// point Bp on axis L can be written in terms of a unit_vector parallel to L;
  /// the parallel moment of inertia J about L; and the perpendicular moment of
  /// inertia K about any line perpendicular to axis L; as:
  /// <pre>
  ///   G = K * Identity + (J - K) * unit_vector ⊗ unit_vector
  /// </pre>
  /// where `Identity` is the identity matrix and ⊗ denotes the tensor product
  /// operator. See Mitiguy, P., 2016. Advanced Dynamics & Motion Simulation.
  /// @param[in] moment_parallel (J) B's unit moment of inertia about axis L.
  /// @param[in] moment_perpendicular (K) B's unit moment of inertia about Bp
  /// for any line perpendicular to unit_vector.
  /// @param[in] unit_vector unit vector parallel to axis L, expressed in a
  /// frame E.
  /// @retval G_BBp_E B's unit inertia about point Bp on B's symmetry axis,
  /// expressed in the same frame E as the unit_vector is expressed.
  /// @pre Points Bp and Bcm are both on B's symmetry axis. The actual location
  /// of these points is not known by this function. However, the value of
  /// moment_perpendicular (K) is associated with point Bp.
  /// @note B's unit inertia about Bp is axially symmetric, meaning B has an
  /// equal moment of inertia about any line that both passes through Bp and
  /// is perpendicular to unit_vector.
  /// @throws std::exception if moment_parallel (J) or moment_perpendicular (K)
  /// is negative or if J > 2 K (violates the triangle inequality, see
  /// CouldBePhysicallyValid()) or ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  static UnitInertia<T> AxiallySymmetric(const T& moment_parallel,
                                         const T& moment_perpendicular,
                                         const Vector3<T>& unit_vector);

  /// Creates a unit inertia for a straight line segment B about a point Bp on
  /// the line segment.
  /// @param[in] moment_perpendicular Unit moment of inertia about any axis
  /// that passes through Bp and is perpendicular to the line segment.
  /// @param[in] unit_vector unit vector defining the line segment's direction,
  /// expressed in a frame E.
  /// @retval G_BBp_E B's unit inertia about Bp, expressed in the same frame E
  /// that the unit_vector is expressed.
  /// @note B's unit inertia about Bp is axially symmetric, meaning B has an
  /// equal moment of inertia about any line that both passes through Bp and
  /// is perpendicular to unit_vector.
  /// @throws std::exception if moment_perpendicular is not positive or if
  /// ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  /// @note B's axial moment of inertia (along the line segment) is zero.
  /// @see ThinRod() is an example of an object that is axially symmetric and
  /// that has a zero moment of inertia about Bp in the unit_vector direction.
  static UnitInertia<T> StraightLine(const T& moment_perpendicular,
                                     const Vector3<T>& unit_vector);

  /// Creates a unit inertia for a uniform density thin rod B about its
  /// center of mass Bcm (which is coincident with B's geometric center Bo).
  /// @param[in] length length of rod in unit_vector direction (meters).
  /// @param[in] unit_vector unit vector defining the rod's axial direction,
  /// expressed in a frame E.
  /// @retval G_BBcm_E B's unit inertia about Bcm expressed in the same frame E
  /// that the unit_vector is expressed.
  /// @note B's unit inertia about Bcm is axially symmetric, meaning B has an
  /// equal moment of inertia about any line that both passes through Bcm and
  /// is perpendicular to unit_vector.
  /// @throws std::exception if length is not positive or if
  /// ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  /// @note B's axial moment of inertia (along the rod) is zero..
  static UnitInertia<T> ThinRod(const T& length, const Vector3<T>& unit_vector);

  /// Constructs a unit inertia with equal moments of inertia along its
  /// diagonal and with each product of inertia set to zero. This factory
  /// is useful for the unit inertia of a uniform-density sphere or cube.
  /// In debug builds, throws std::exception if I_triaxial is negative/NaN.
  /// @see UnitInertia::SolidSphere() and UnitInertia::SolidCube() for examples.
  // TODO(mitiguy) Per issue #6139  Update to ConstructTriaxiallySymmetric.
  static UnitInertia<T> TriaxiallySymmetric(const T& I_triaxial) {
    return UnitInertia<T>(
        RotationalInertia<T>::TriaxiallySymmetric(I_triaxial));
  }
  // End of Doxygen group
  //@}

#ifndef DRAKE_DOXYGEN_CXX
  // (Internal use only)
  // @see SpatialInertia::CalcPrincipalHalfLengthsAndPoseForEquivalentShape()
  // for documentation, formulas, and details on @p inertia_shape_factor.
  // @returns 3 principal ½-lengths [lmax lmed lmin] sorted in descending order
  // (lmax ≥ lmed ≥ lmin) and their associated principal directions [Ax Ay Az]
  // stored in columns of the returned rotation matrix R_EA.
  // @throws std::exception if the elements of `this` unit inertia cannot
  // be converted to a real finite double. For example, an exception is thrown
  // if `this` contains an erroneous NaN or if scalar type T is symbolic.
  // @throws std::exception if inertia_shape_factor ≤ 0 or > 1.
  // See @ref spatial_inertia_equivalent_shapes
  // "Spatial inertia equivalent shapes" for more details.
  std::pair<Vector3<double>, math::RotationMatrix<double>>
  CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
      double inertia_shape_factor) const;
#endif

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

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::UnitInertia);
