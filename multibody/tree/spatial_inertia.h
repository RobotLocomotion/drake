#pragma once

#include <algorithm>
#include <exception>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt_ostream.h"
#include "drake/math/cross_product.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/tree/rotational_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"

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
/// @note Several methods in this class throw a std::exception for invalid
/// rotational inertia operations in debug releases only.  This provides speed
/// in a release build while facilitating debugging in debug builds.
/// In addition, these validity tests are only performed for scalar types for
/// which drake::scalar_predicate<T>::is_bool is `true`. For instance, validity
/// checks are not performed when T is symbolic::Expression.
///
/// @note The methods of this class satisfy the "basic exception guarantee": if
/// an exception is thrown, the program will still be in a valid
/// state. Specifically, no resources are leaked, and all objects' invariants
/// are intact. Be aware that SpatialInertia objects may contain invalid
/// inertia data in cases where input checking is skipped.
/// @see https://en.cppreference.com/w/cpp/language/exceptions
///
/// @see To create a spatial inertia of a mesh, see
/// @ref CalcSpatialInertia(const geometry::TriangleSurfaceMesh<double>& mesh,
/// double density). <!--# NOLINT-->
///
/// @see To create spatial inertia from most of geometry::Shape, see
/// @ref CalcSpatialInertia(const geometry::Shape& shape, double density).
///
/// @see To create spatial inertia for a set of bodies, see
/// @ref MultibodyPlant::CalcSpatialInertia().
///
/// - [Jain 2010]  Jain, A., 2010. Robot and multibody dynamics: analysis and
///                algorithms. Springer Science & Business Media.
///
/// @tparam_default_scalar
template <typename T>
class SpatialInertia {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialInertia);

  /// Creates a spatial inertia for a physical body or composite body S about a
  /// point P from a given mass, center of mass, and central rotational inertia.
  /// For example, this method creates a body's SpatialInertia about its body
  /// origin Bo from the body's mass, position vector from Bo to the body's
  /// center of mass, and rotational inertia about the body's center of mass.
  ///
  /// This method checks for the physical validity of the resulting
  /// %SpatialInertia with IsPhysicallyValid() and throws a std::runtime_error
  /// exception in the event the provided input parameters lead to a
  /// non-physically viable spatial inertia.
  ///
  /// @param[in] mass The mass of the body or composite body S.
  /// @param[in] p_PScm_E The position vector from point P to point `Scm`
  ///                     (S's center of mass), expressed in a frame E.
  /// @param[in] I_SScm_E S's RotationalInertia about Scm, expressed in frame E.
  /// @retval M_SP_E S's spatial inertia about point P, expressed in frame E.
  static SpatialInertia<T> MakeFromCentralInertia(
      const T& mass, const Vector3<T>& p_PScm_E,
      const RotationalInertia<T>& I_SScm_E);

  /// (Internal use only) Creates a spatial inertia whose mass is 1, position
  /// vector to center of mass is zero, and whose rotational inertia has
  /// moments of inertia of 1 and products of inertia of 0.
  static SpatialInertia<T> MakeUnitary();

  /// Creates the spatial inertia for a particle Q of mass m about a point P.
  /// @param[in] mass mass of the single particle (units of kg).
  /// @param[in] position vector from point P to Q, expressed in a frame B.
  /// @retval M_QP_B particle Q's spatial inertia about P, expressed in frame B.
  /// @throws std::exception if mass is not positive and finite.
  static SpatialInertia<T> PointMass(const T& mass, const Vector3<T>& position);

  /// Creates a spatial inertia for a uniform density solid box B about its
  /// geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] density mass per volume (kg/m³).
  /// @param[in] lx length of the box in the Bx direction (meters).
  /// @param[in] ly length of the box in the By direction (meters).
  /// @param[in] lz length of the box in the Bz direction (meters).
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B.
  /// @throws std::exception if density, lx, ly, or lz is not positive and
  /// finite.
  static SpatialInertia<T> SolidBoxWithDensity(const T& density, const T& lx,
                                               const T& ly, const T& lz);

  /// Creates a spatial inertia for a uniform density solid box B about its
  /// geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] mass mass of the solid box (kg).
  /// @param[in] lx length of the box in the Bx direction (meters).
  /// @param[in] ly length of the box in the By direction (meters).
  /// @param[in] lz length of the box in the Bz direction (meters).
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B.
  /// @throws std::exception if mass, lx, ly, or lz is not positive and finite.
  static SpatialInertia<T> SolidBoxWithMass(const T& mass, const T& lx,
                                            const T& ly, const T& lz);

  /// Creates a spatial inertia for a uniform density solid cube B about its
  /// geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] density mass per volume (kg/m³).
  /// @param[in] length The length of each of the cube's sides (meters).
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B.  Since B's
  /// rotational inertia is triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo
  /// expressed in frame B is equal to M_BBo expressed in an arbitrary frame E.
  /// @note B's rotational inertia about Bo is triaxially symmetric, meaning
  /// B has an equal moment of inertia about any line passing through Bo.
  /// @throws std::exception if density or length is not positive and finite.
  static SpatialInertia<T> SolidCubeWithDensity(const T& density,
                                                const T& length);

  /// Creates a spatial inertia for a uniform density solid cube B about its
  /// geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] mass mass of the solid cube (kg).
  /// @param[in] length The length of each of the cube's sides (meters).
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B.  Since B's
  /// rotational inertia is triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo
  /// expressed in frame B is equal to M_BBo expressed in an arbitrary frame E.
  /// @note B's rotational inertia about Bo is triaxially symmetric, meaning
  /// B has an equal moment of inertia about any line passing through Bo.
  /// @throws std::exception if mass or length is not positive and finite.
  static SpatialInertia<T> SolidCubeWithMass(const T& mass, const T& length);

  /// Creates a spatial inertia for a uniform density solid capsule B about
  /// its geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] density mass per volume (kg/m³).
  /// @param[in] radius radius of the cylinder/half-sphere parts of the capsule.
  /// @param[in] length length of the cylindrical part of the capsule.
  /// @param[in] unit_vector unit vector defining the axial direction of the
  /// cylindrical part of the capsule, expressed in B.
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B.
  /// @note B's rotational inertia about Bo is axially symmetric, meaning B has
  /// an equal moment of inertia about any line that both passes through Bo
  /// and is perpendicular to unit_vector.
  /// @throws std::exception if density, radius, or length is not positive and
  /// finite or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  static SpatialInertia<T> SolidCapsuleWithDensity(
      const T& density, const T& radius, const T& length,
      const Vector3<T>& unit_vector);

  /// Creates a spatial inertia for a uniform density solid capsule B about
  /// its geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] mass mass of the solid capsule (kg).
  /// @param[in] radius radius of the cylinder/half-sphere parts of the capsule.
  /// @param[in] length length of the cylindrical part of the capsule.
  /// @param[in] unit_vector unit vector defining the axial direction of the
  /// cylindrical part of the capsule, expressed in B.
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B.
  /// @note B's rotational inertia about Bo is axially symmetric, meaning B has
  /// an equal moment of inertia about any line that both passes through Bo
  /// and is perpendicular to unit_vector.
  /// @throws std::exception if mass, radius, or length is not positive and
  /// finite or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  static SpatialInertia<T> SolidCapsuleWithMass(const T& mass, const T& radius,
                                                const T& length,
                                                const Vector3<T>& unit_vector);

  /// Creates a spatial inertia for a uniform density solid cylinder B about
  /// its geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] density mass per volume (kg/m³).
  /// @param[in] radius radius of the cylinder (meters).
  /// @param[in] length length of cylinder in unit_vector direction (meters).
  /// @param[in] unit_vector unit vector defining the axial direction of the
  /// cylinder, expressed in B.
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B.
  /// @note B's rotational inertia about Bo is axially symmetric, meaning B has
  /// an equal moment of inertia about any line that both passes through Bo
  /// and is perpendicular to unit_vector.
  /// @throws std::exception if density, radius, or length is not positive and
  /// finite or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  /// @see SolidCylinderWithDensityAboutEnd() to calculate M_BBp_B, B's spatial
  /// inertia about Bp (at the center of one of the cylinder's circular ends).
  static SpatialInertia<T> SolidCylinderWithDensity(
      const T& density, const T& radius, const T& length,
      const Vector3<T>& unit_vector);

  /// Creates a spatial inertia for a uniform density solid cylinder B about
  /// its geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] mass mass of the solid cylinder (kg).
  /// @param[in] radius radius of the cylinder (meters).
  /// @param[in] length length of cylinder in unit_vector direction (meters).
  /// @param[in] unit_vector unit vector defining the axial direction of the
  /// cylinder, expressed in B.
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B.
  /// @note B's rotational inertia about Bo is axially symmetric, meaning B has
  /// an equal moment of inertia about any line that both passes through Bo
  /// and is perpendicular to unit_vector.
  /// @throws std::exception if mass, radius, or length is not positive and
  /// finite or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  /// @see SolidCylinderWithMassAboutEnd() to calculate M_BBp_B, B's spatial
  /// inertia about Bp (at the center of one of the cylinder's circular ends).
  static SpatialInertia<T> SolidCylinderWithMass(const T& mass, const T& radius,
                                                 const T& length,
                                                 const Vector3<T>& unit_vector);

  /// Creates a spatial inertia for a uniform-density solid cylinder B about an
  /// end-point Bp of the cylinder's axis (see below for more about Bp).
  /// @param[in] density mass per volume (kg/m³).
  /// @param[in] radius radius of cylinder (meters).
  /// @param[in] length length of cylinder in unit_vector direction (meters).
  /// @param[in] unit_vector unit vector parallel to the axis of the cylinder
  /// and directed from Bp to Bcm (B's center of mass), expressed in B.
  /// @retval M_BBp_B B's spatial inertia about Bp, expressed in B.
  /// @note The position from Bp to Bcm is p_BpBcm = length / 2 * unit_vector.
  /// @note B's rotational inertia about Bp is axially symmetric, meaning B has
  /// an equal moment of inertia about any line that both passes through Bp
  /// and is perpendicular to unit_vector.
  /// @throws std::exception if density, radius, or length is not positive and
  /// finite or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  /// @see SolidCylinderWithDensity() to calculate M_BBcm_B, B's spatial
  /// inertia about Bcm (B's center of mass).
  static SpatialInertia<T> SolidCylinderWithDensityAboutEnd(
      const T& density, const T& radius, const T& length,
      const Vector3<T>& unit_vector);

  /// Creates a spatial inertia for a uniform-density solid cylinder B about an
  /// end-point Bp of the cylinder's axis (see below for more about Bp).
  /// @param[in] mass mass of the solid cylinder (kg).
  /// @param[in] radius radius of cylinder (meters).
  /// @param[in] length length of cylinder in unit_vector direction (meters).
  /// @param[in] unit_vector unit vector parallel to the axis of the cylinder
  /// and directed from Bp to Bcm (B's center of mass), expressed in B.
  /// @retval M_BBp_B B's spatial inertia about Bp, expressed in B.
  /// @note The position from Bp to Bcm is p_BpBcm = length / 2 * unit_vector.
  /// @note B's rotational inertia about Bp is axially symmetric, meaning B has
  /// an equal moment of inertia about any line that both passes through Bp
  /// and is perpendicular to unit_vector.
  /// @throws std::exception if density, radius, or length is not positive and
  /// finite or if ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  /// @see SolidCylinderWithMass() to calculate M_BBcm_B, B's spatial inertia
  /// about Bcm (B's center of mass).
  static SpatialInertia<T> SolidCylinderWithMassAboutEnd(
      const T& mass, const T& radius, const T& length,
      const Vector3<T>& unit_vector);

  /// Creates a spatial inertia for a uniform-density thin rod B about its
  /// center of mass Bcm.
  /// @param[in] mass mass of the rod (units of kg).
  /// @param[in] length length of the rod (units of meters).
  /// @param[in] unit_vector unit vector defining the rod's axial direction,
  /// expressed in B.
  /// @retval M_BBcm_B B's spatial inertia about Bcm, expressed in B.
  /// @note B's rotational inertia about Bcm is axially symmetric, meaning B has
  /// an equal moment of inertia about any line that both passes through Bcm and
  /// is perpendicular to unit_vector. B has no (zero) rotational inertia about
  /// the line that passes through Bcm and is parallel to unit_vector.
  /// @throws std::exception if mass or length is not positive and finite or
  /// if ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  /// @see ThinRodWithMassAboutEnd() to calculate M_BBp_B, B's spatial inertia
  /// about Bp (one of the ends of rod B).
  static SpatialInertia<T> ThinRodWithMass(const T& mass, const T& length,
                                           const Vector3<T>& unit_vector);

  /// Creates a spatial inertia for a uniform-density thin rod B about one of
  /// its ends.
  /// @param[in] mass mass of the rod (units of kg).
  /// @param[in] length length of the rod (units of meters).
  /// @param[in] unit_vector unit vector defining the rod's axial direction,
  /// expressed in B.
  /// @retval M_BBp_B B's spatial inertia about Bp, expressed in B.
  /// @note The position from Bp to Bcm is length / 2 * unit_vector.
  /// @note B's rotational inertia about Bp is axially symmetric, meaning B has
  /// an equal moment of inertia about any line that both passes through Bp and
  /// is perpendicular to unit_vector. B has no (zero) rotational inertia about
  /// the line that passes through Bp and is parallel to unit_vector.
  /// @throws std::exception if mass or length is not positive and finite or
  /// if ‖unit_vector‖ is not within 1.0E-14 of 1.0.
  /// @see ThinRodWithMass() to calculate M_BBcm_B, B's spatial inertia about
  /// Bcm (B's center of mass).
  static SpatialInertia<T> ThinRodWithMassAboutEnd(
      const T& mass, const T& length, const Vector3<T>& unit_vector);

  /// Creates a spatial inertia for a uniform density solid ellipsoid B about
  /// its geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] density mass per volume (kg/m³).
  /// @param[in] a length of ellipsoid semi-axis in the ellipsoid Bx direction.
  /// @param[in] b length of ellipsoid semi-axis in the ellipsoid By direction.
  /// @param[in] c length of ellipsoid semi-axis in the ellipsoid Bz direction.
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B.
  /// @throws std::exception if density, a, b, or c is not positive and finite.
  static SpatialInertia<T> SolidEllipsoidWithDensity(const T& density,
                                                     const T& a, const T& b,
                                                     const T& c);

  /// Creates a spatial inertia for a uniform density solid ellipsoid B about
  /// its geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] mass mass of the solid ellipsoid (kg).
  /// @param[in] a length of ellipsoid semi-axis in the ellipsoid Bx direction.
  /// @param[in] b length of ellipsoid semi-axis in the ellipsoid By direction.
  /// @param[in] c length of ellipsoid semi-axis in the ellipsoid Bz direction.
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B.
  /// @throws std::exception if mass, a, b, or c is not positive and finite.
  static SpatialInertia<T> SolidEllipsoidWithMass(const T& mass, const T& a,
                                                  const T& b, const T& c);

  /// Creates a spatial inertia for a uniform density solid sphere B about its
  /// geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] density mass per volume (kg/m³).
  /// @param[in] radius sphere's radius (meters).
  /// @retval M_BBo B's spatial inertia about Bo. Since B's rotational inertia
  /// is triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo expressed in
  /// frame B is equal to M_BBo expressed in an arbitrary frame E.
  /// @note B's rotational inertia about Bo is triaxially symmetric, meaning
  /// B has an equal moment of inertia about any line passing through Bo.
  /// @throws std::exception if density or radius is not positive and finite.
  static SpatialInertia<T> SolidSphereWithDensity(const T& density,
                                                  const T& radius);

  /// Creates a spatial inertia for a uniform density solid sphere B about its
  /// geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] mass mass of the solid sphere (kg).
  /// @param[in] radius sphere's radius (meters).
  /// @retval M_BBo B's spatial inertia about Bo. Since B's rotational inertia
  /// is triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo expressed in
  /// frame B is equal to M_BBo expressed in an arbitrary frame E.
  /// @note B's rotational inertia about Bo is triaxially symmetric, meaning
  /// B has an equal moment of inertia about any line passing through Bo.
  /// @throws std::exception if mass or radius is not positive and finite.
  static SpatialInertia<T> SolidSphereWithMass(const T& mass, const T& radius);

  /// Creates a spatial inertia for a uniform density thin hollow sphere B about
  /// its geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] area_density mass per unit area (kg/m²).
  /// @param[in] radius sphere's radius in meters (the hollow sphere is regarded
  /// as an infinitesimally thin shell of uniform density).
  /// @retval M_BBo_B B's spatial inertia about Bo, expressed in B. Since B's
  /// rotational inertia is triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo
  /// expressed in frame B is equal to M_BBo expressed in an arbitrary frame E.
  /// @note B's rotational inertia about Bo is triaxially symmetric, meaning
  /// B has an equal moment of inertia about any line passing through Bo.
  /// @throws std::exception if area_density or radius is not positive and
  /// finite.
  static SpatialInertia<T> HollowSphereWithDensity(const T& area_density,
                                                   const T& radius);

  /// Creates a spatial inertia for a uniform density hollow sphere B about its
  /// geometric center Bo (which is coincident with B's center of mass Bcm).
  /// @param[in] mass mass of the hollow sphere (kg).
  /// @param[in] radius sphere's radius in meters (the hollow sphere is regarded
  /// as an infinitesimally thin shell of uniform density).
  /// @retval M_BBo B's spatial inertia about Bo. Since B's rotational inertia
  /// is triaxially symmetric, M_BBo_B = M_BBo_E, i.e., M_BBo expressed in
  /// frame B is equal to M_BBo expressed in an arbitrary frame E.
  /// @note B's rotational inertia about Bo is triaxially symmetric, meaning
  /// B has an equal moment of inertia about any line passing through Bo.
  /// @throws std::exception if mass or radius is not positive and finite.
  static SpatialInertia<T> HollowSphereWithMass(const T& mass, const T& radius);

  /// Creates a spatial inertia for a uniform density solid tetrahedron B about
  /// a point A, from which position vectors to B's 4 vertices B0, B1, B2, B3
  /// are measured (position vectors are all expressed in a common frame E).
  /// @param[in] density mass per volume (kg/m³).
  /// @param[in] p0 position vector p_AB0_E from point A to B0, expressed in E.
  /// @param[in] p1 position vector p_AB1_E from point A to B1, expressed in E.
  /// @param[in] p2 position vector p_AB2_E from point A to B2, expressed in E.
  /// @param[in] p3 position vector p_AB3_E from point A to B3, expressed in E.
  /// @retval M_BA_E B's spatial inertia about point A, expressed in E.
  /// @note In the common case, point A is Eo (the origin of the expressed-in
  /// frame E). The example below has point A as Wo (origin of world frame W).
  /// @throws std::exception if density is not positive and finite.
  /// @code{.cc}
  /// double density = 1000;
  /// Vector3<double> p_WoB0_W(1, 0, 0);
  /// Vector3<double> p_WoB1_W(2, 0, 0);
  /// Vector3<double> p_WoB2_W(1, 1, 0);
  /// Vector3<double> p_WoB3_W(1, 0, 1);
  /// SpatialInertia<double> M_BWo_W =
  ///     SpatialInertia<double>::SolidTetrahedronAboutPointWithDensity(
  ///         density, p_WoB0_W, p_WoB1_W, p_WoB2_W, p_WoB3_W);
  /// @endcode
  /// @see SolidTetrahedronAboutVertexWithDensity() to efficiently calculate a
  /// spatial inertia about a vertex of B.
  static SpatialInertia<T> SolidTetrahedronAboutPointWithDensity(
      const T& density, const Vector3<T>& p0, const Vector3<T>& p1,
      const Vector3<T>& p2, const Vector3<T>& p3);

  /// (Advanced) Creates a spatial inertia for a uniform density solid
  /// tetrahedron B about its vertex B0, from which position vectors to B's
  /// other 3 vertices B1, B2, B3 are measured (position vectors are all
  /// expressed in a common frame E).
  /// @param[in] density mass per volume (kg/m³).
  /// @param[in] p1 position vector p_B0B1_E from B0 to B1, expressed in E.
  /// @param[in] p2 position vector p_B0B2_E from B0 to B2, expressed in E.
  /// @param[in] p3 position vector p_B0B3_E from B0 to B3, expressed in E.
  /// @retval M_BB0_E B's spatial inertia about its vertex B0, expressed in E.
  /// @throws std::exception if density is not positive and finite.
  /// @see SolidTetrahedronAboutPointWithDensity() to calculate a spatial
  /// inertia about an arbitrary point.
  static SpatialInertia<T> SolidTetrahedronAboutVertexWithDensity(
      const T& density, const Vector3<T>& p1, const Vector3<T>& p2,
      const Vector3<T>& p3);

  /// Initializes mass, center of mass and rotational inertia to zero.
  static SpatialInertia Zero() {
    return SpatialInertia(0, Vector3<T>::Zero(), UnitInertia<T>(0, 0, 0));
  }

  /// Initializes mass, center of mass and rotational inertia to invalid NaN's
  /// for a quick detection of uninitialized values.
  static SpatialInertia NaN() { return SpatialInertia(); }

  /// Constructs a spatial inertia for a physical body or composite body S about
  /// a point P from a given mass, center of mass and rotational inertia. The
  /// center of mass is specified by the position vector `p_PScm_E` from point P
  /// to the center of mass point `Scm`, expressed in a frame E.
  /// The rotational inertia is provided as the UnitInertia `G_SP_E` of the body
  /// or composite body S computed about point P and expressed in frame E.
  ///
  /// @note The third argument of this constructor is unusual in that it is an
  /// UnitInertia (not a traditional RotationalInertia) and its inertia is about
  /// the arbitrary point P (not Scm -- S's center of mass).
  /// @see MakeFromCentralInertia a factory method with traditional utility.
  ///
  /// This constructor checks for the physical validity of the resulting
  /// %SpatialInertia with IsPhysicallyValid() and throws a std::runtime_error
  /// exception in the event the provided input parameters lead to
  /// non-physically viable spatial inertia. Since this check has non-negligable
  /// runtime costs, it can be disabled by setting the optional argument
  /// `skip_validity_check` to `true`.
  ///
  /// @param[in] mass The mass of the body or composite body S.
  /// @param[in] p_PScm_E The position vector from point P to the center of mass
  ///                     of body or composite body S expressed in frame E.
  /// @param[in] G_SP_E UnitInertia of the body or composite body S computed
  ///                   about origin point P and expressed in frame E.
  /// @param[in] skip_validity_check If true, skips the validity check described
  ///                                above. Defaults to false.
  SpatialInertia(const T& mass, const Vector3<T>& p_PScm_E,
                 const UnitInertia<T>& G_SP_E,
                 const bool skip_validity_check = false)
      : mass_(mass), p_PScm_E_(p_PScm_E), G_SP_E_(G_SP_E) {
    if (!skip_validity_check) {
      ThrowIfNotPhysicallyValid();
    }
  }

  /// Returns a new %SpatialInertia object templated on `Scalar` initialized
  /// from the value of `this` spatial inertia.
  ///
  /// @tparam Scalar The scalar type on which the new spatial inertia will
  /// be templated.
  ///
  /// @note `SpatialInertia<From>::cast<To>()` creates a new
  /// `SpatialInertia<To>` from a `SpatialInertia<From>` but only if
  /// type `To` is constructible from type `From`.
  /// This cast method works in accordance with Eigen's cast method for Eigen's
  /// objects that underlie this %SpatialInertia.  For example, Eigen
  /// currently allows cast from type double to AutoDiffXd, but not vice-versa.
  template <typename Scalar>
  SpatialInertia<Scalar> cast() const {
    return SpatialInertia<Scalar>(
        get_mass(), get_com().template cast<Scalar>(),
        get_unit_inertia().template cast<Scalar>(),
        true);  // Skip validity check since this inertia is already valid.
  }

  /// Get a constant reference to the mass of this spatial inertia.
  const T& get_mass() const { return mass_; }

  /// Get a constant reference to the position vector `p_PScm_E` from the
  /// _about point_ P to the center of mass `Scm` of the body or composite body
  /// S, expressed in frame E. See the documentation of this class for details.
  const Vector3<T>& get_com() const { return p_PScm_E_; }

  /// Computes the center of mass moment vector `mass * p_PScm_E` given the
  /// position vector `p_PScm_E` from the _about point_ P to the center of mass
  /// `Scm` of the body or composite body S, expressed in frame E. See the
  /// documentation of this class for details.
  Vector3<T> CalcComMoment() const { return mass_ * p_PScm_E_; }

  /// Get a constant reference to the unit inertia `G_SP_E` of this
  /// spatial inertia, computed about point P and expressed in frame E. See the
  /// documentation of this class for details.
  const UnitInertia<T>& get_unit_inertia() const { return G_SP_E_; }

  /// Computes the rotational inertia `I_SP_E = mass * G_SP_E` of this
  /// spatial inertia, computed about point P and expressed in frame E. See the
  /// documentation of this class for details.
  RotationalInertia<T> CalcRotationalInertia() const { return mass_ * G_SP_E_; }

  /// Returns `true` if any of the elements in this spatial inertia is NaN
  /// and `false` otherwise.
  boolean<T> IsNaN() const {
    using std::isnan;
    return isnan(mass_) || G_SP_E_.IsNaN() ||
           any_of(p_PScm_E_, [](const auto& x) {
             return isnan(x);
           });
  }

  /// Returns `true` if all of the elements in this spatial inertia are zero
  /// and `false` otherwise.
  boolean<T> IsZero() const {
    return (mass_ == 0.0) && G_SP_E_.IsZero() &&
           all_of(p_PScm_E_, [](const auto& x) {
             return (x == 0.0);
           });
  }

  /// Performs a number of checks to verify that this is a physically valid
  /// spatial inertia. The checks performed include:
  ///
  /// - No NaN entries.
  /// - Non-negative mass.
  /// - Non-negative principal moments about the center of mass.
  /// - Principal moments about the center of mass must satisfy the triangle
  ///   inequality:
  ///   - `Ixx + Iyy >= Izz`
  ///   - `Ixx + Izz >= Iyy`
  ///   - `Iyy + Izz >= Ixx`
  ///
  /// These are the tests performed by
  /// RotationalInertia::CouldBePhysicallyValid() which become a sufficient
  /// condition when performed on a rotational inertia about a body's center of
  /// mass.
  /// @see RotationalInertia::CouldBePhysicallyValid().
  boolean<T> IsPhysicallyValid() const;

  /// (Internal use only). Returns an optional string if this SpatialInertia is
  /// invalid, otherwise returns an empty optional.
  std::optional<std::string> CreateInvalidityReport() const;

  /// @anchor spatial_inertia_equivalent_shapes
  /// @name Spatial inertia equivalent shapes
  /// Calculates principal semi-diameters (half-lengths), principal axes
  /// orientations, and the position of a uniform-density object whose spatial
  /// inertia is equal to `this` spatial inertia.
  /// These functions are useful for visualization or physical interpretation
  /// of the geometric extents of `this` spatial inertia for a given shape.
  /// These functions return 3 principal semi-diameters (half-lengths) [a b c]
  /// sorted in descending order (a ≥ b ≥ c) which are measured from Scm (the
  /// center of mass of `this` spatial inertia). They also return the pose of
  /// the uniform density object that represents `this` spatial inertia.
  ///
  /// Example: Consider an oddly-shaped rigid body B with a known spatial
  /// inertia M_BBo_B about B's origin Bo, expressed in frame B. These functions
  /// return an easily visualized simple shape whose spatial inertial is equal
  /// to M_BBo_B. The simple shape is defined by a frame A with origin Ao at Bcm
  /// (B's center of mass), has principal dimensions [a b c], and has unit
  /// vectors Ax, Ay, Az parallel to the simple shape's principal directions.
  /// When the simple shape is a uniform-density solid ellipsoid, proceed as
  /// follows to form [a b c], the rotation matrix R_BA describing Ax, Ay, Az,
  /// and the position vector p_BoAo_B from Bo to Ao (ellipsoid center of mass).
  /// @code{.cpp}
  ///   const SpatialInertia<double>& M_BBo_B = B.default_spatial_inertia();
  ///   auto [abc, X_BA] =
  ///     M_BBo_B.CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid();
  /// @endcode
  /// @throws std::exception if the elements of `this` spatial inertia cannot
  /// be converted to a real finite double. For example, an exception is thrown
  /// if `this` contains an erroneous NaN or if scalar type T is symbolic.
  /// @see RotationalInertia::CalcPrincipalMomentsAndAxesOfInertia() to form
  /// principal moments of inertia and their associated principal directions.
  ///@{

  /// Returns 3 principal semi-diameters [lmax lmed lmin] sorted in descending
  /// order (lmax ≥ lmed ≥ lmin), orientation, and position of a solid ellipsoid
  /// whose spatial inertia is equal to `this` spatial inertia.
  /// See @ref spatial_inertia_equivalent_shapes
  /// "Spatial inertia equivalent shapes" for more details.
  /// @throws std::exception if the elements of `this` spatial inertia cannot
  /// be converted to a real finite double. For example, an exception is thrown
  /// if `this` contains an erroneous NaN or if scalar type T is symbolic.
  std::pair<Vector3<double>, drake::math::RigidTransform<double>>
  CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid() const {
    constexpr double inertia_shape_factor = 1.0 / 5.0;
    return CalcPrincipalHalfLengthsAndPoseForEquivalentShape(
        inertia_shape_factor);
  }

  /// Returns three ½-lengths [lmax lmed lmin] sorted in descending order
  /// (lmax ≥ lmed ≥ lmin), orientation, and position of a solid box
  /// whose spatial inertia is equal to `this` spatial inertia.
  /// See @ref spatial_inertia_equivalent_shapes
  /// "Spatial inertia equivalent shapes" for more details.
  /// @throws std::exception if the elements of `this` spatial inertia cannot
  /// be converted to a real finite double. For example, an exception is thrown
  /// if `this` contains an erroneous NaN or if scalar type T is symbolic.
  std::pair<Vector3<double>, drake::math::RigidTransform<double>>
  CalcPrincipalHalfLengthsAndPoseForSolidBox() const {
    constexpr double inertia_shape_factor = 1.0 / 3.0;
    return CalcPrincipalHalfLengthsAndPoseForEquivalentShape(
        inertia_shape_factor);
  }

  /// Returns three ½-lengths [lmax lmed lmin] sorted in descending order
  /// (lmax ≥ lmed ≥ lmin), orientation, and position of a box whose mass is
  /// concentrated in 8 particles at the box's corners and whose spatial inertia
  /// is equal to `this` spatial inertia. The physical geometry of the actual
  /// underlying object must be larger than this box, as this box is the minimum
  /// bounding box for the actual geometry.
  /// See @ref spatial_inertia_equivalent_shapes
  /// "Spatial inertia equivalent shapes" for more details.
  /// @throws std::exception if the elements of `this` spatial inertia cannot
  /// be converted to a real finite double. For example, an exception is thrown
  /// if `this` contains an erroneous NaN or if scalar type T is symbolic.
  std::pair<Vector3<double>, drake::math::RigidTransform<double>>
  CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox() const {
    constexpr double inertia_shape_factor = 1.0;
    return CalcPrincipalHalfLengthsAndPoseForEquivalentShape(
        inertia_shape_factor);
  }

  /// Returns the minimum possible length for the physical extent of the massive
  /// object that underlies this spatial inertia. In other words, the underlying
  /// physical object must have at least two particles whose distance between
  /// each other is greater than or equal to the minimum possible length.
  /// @note The minimum possible length is equal to the space-diagonal of the
  /// minimum bounding box for `this` spatial inertia, which happens to be equal
  /// to √(2 * trace of the central unit inertia associated with `this`).
  /// @note Minimum possible length can be used to detect erroneous inertias
  /// associated with absurdly large objects or to detect errors when the
  /// minimum possible length is larger than the real physical geometry that
  /// underlies `this` spatial inertia (maybe due to inertia conversion errors,
  /// e.g., factor of 10⁷ from kg m² to g cm² or 10⁹ from kg m² to g mm²).
  /// To assess whether the minimum possible length is reasonable, it helps to
  /// have comparable sizes, e.g., the world's largest aircraft carrier has a
  /// space-diagonal ≈ 355 m (length ≈ 337 m, width ≈ 78 m, height ≈ 76 m), the
  /// largest land vehicle (Bagger bucket-wheel excavator) is ≈ 224 m long, the
  /// largest human object in space (International Space Station) is 109 m long
  /// and 75 m wide, the USA space shuttle is ≈ 37 m long and can carry a 15.2 m
  /// Canadarm, the world's largest humanoid robot (Mononofu) is ≈ 8.5 m tall.
  /// Also, minimum possible length can be compared to known physical geometry
  /// (e.g., realistic collision geometry, visual geometry, or physical extents
  /// associated with body connectivity data and topology), and this comparison
  /// can be used to warn that a spatial inertia may be physically impossible
  /// (e.g., underlying geometry is smaller than the minimum possible length).
  T CalcMinimumPhysicalLength() const;
  ///@}

  /// Copy to a full 6x6 matrix representation.
  Matrix6<T> CopyToFullMatrix6() const {
    using drake::math::VectorToSkewSymmetric;
    Matrix6<T> M;
    M.template block<3, 3>(0, 0) = mass_ * G_SP_E_.CopyToFullMatrix3();
    M.template block<3, 3>(0, 3) = mass_ * VectorToSkewSymmetric(p_PScm_E_);
    M.template block<3, 3>(3, 0) = -M.template block<3, 3>(0, 3);
    M.template block<3, 3>(3, 3) = mass_ * Matrix3<T>::Identity();
    return M;  // 33 flops and 36 copies
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
  /// @note Given that the composition of spatial inertias is not well defined
  /// for massless bodies, this composition of the spatial inertias performs the
  /// arithmetic average of the center of mass position vector (get_com()) and
  /// unit inertia (get_unit_inertia()) when the two spatial inertias have zero
  /// mass (get_mass()). This is only valid in the limit to zero mass for two
  /// bodies with the same mass. This special case allows the composition of
  /// spatial inertias in the common case of a kinematic chain of massless
  /// bodies.
  ///
  /// @warning This operation is only valid if both spatial inertias are
  /// computed about the same point P and expressed in the same frame E.
  /// Considering `this` spatial inertia to be `M_SP_E` for some body or
  /// composite body S, about some point P, the supplied spatial inertia
  /// `M_BP_E` must be for some other body or composite body B about the _same_
  /// point P; B's inertia is then included in S.
  SpatialInertia<T>& operator+=(const SpatialInertia<T>& M_BP_E);

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// taken about a point P and expressed in frame E, this method computes the
  /// same inertia re-expressed in another frame A.
  /// This operation is performed in-place modifying the original object.
  /// On return, `this` is now re-expressed in frame A, that is, `M_SP_A`.
  /// @param[in] R_AE Rotation matrix from frame E to frame A.
  void ReExpressInPlace(const math::RotationMatrix<T>& R_AE) {
    p_PScm_E_ = R_AE * p_PScm_E_;    // Now p_PScm_A, 15 flops
    G_SP_E_.ReExpressInPlace(R_AE);  // Now I_SP_A, 57 flops
    // Now M_SP_A, total 72 flops
  }

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// taken about a point P and expressed in frame E, this method computes the
  /// same inertia re-expressed in another frame A.
  /// @param[in] R_AE RotationMatrix relating frames A and E.
  /// @retval M_SP_A The same spatial inertia of S about P but now
  ///                re-expressed in frame A.
  /// @see ReExpressInPlace() for details.
  SpatialInertia<T> ReExpress(const math::RotationMatrix<T>& R_AE) const {
    SpatialInertia result(*this);
    result.ReExpressInPlace(R_AE);
    return result;
  }

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// computed about point P, and expressed in frame E, this method uses
  /// the _Parallel Axis Theorem_ for spatial inertias to compute the same
  /// spatial inertia about a new point Q. The result still is expressed in
  /// frame E.
  /// This operation is performed in-place modifying the original object.
  /// On return, `this` is now computed about a new point Q.
  /// @see Shift() which does not modify this object.
  ///
  /// For details see Section 2.1.2, p. 20 of [Jain 2010].
  ///
  /// @param[in] p_PQ_E position vector from the original about-point P to the
  ///                   new about-point Q, expressed in the same frame E that
  ///                   `this` spatial inertia is expressed in.
  void ShiftInPlace(const Vector3<T>& p_PQ_E) {
    const Vector3<T> p_QScm_E = p_PScm_E_ - p_PQ_E;  // 3 flops
    // Use the shift theorem (parallel axis theorem) to first shift
    // G_SP to G_SScm and then shift G_SScm to G_SQ.
    G_SP_E_.ShiftToCenterOfMassInPlace(p_PScm_E_);   // 17 flops
    G_SP_E_.ShiftFromCenterOfMassInPlace(p_QScm_E);  // 17 flops
    p_PScm_E_ = p_QScm_E;
    // Note: It would be an implementation bug if a shift starts with a valid
    // spatial inertia and the shift produces an invalid spatial inertia.
    // Hence, no need to use DRAKE_ASSERT_VOID(CheckInvariants()).
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
  /// @retval M_SQ_E    This same spatial inertia for body or composite body S
  ///                   but computed about a new point Q.
  SpatialInertia<T> Shift(const Vector3<T>& p_PQ_E) const {
    SpatialInertia result(*this);
    result.ShiftInPlace(p_PQ_E);  // 37 flops
    return result;
  }

  /// Multiplies `this` spatial inertia `M_Bo_E` of a body B about its frame
  /// origin `Bo` by the spatial acceleration of the body frame B in a frame W.
  /// Mathematically: <pre>
  ///   F_Bo_E = M_Bo_E * A_WB_E
  /// </pre>
  /// or, in terms of its rotational and translational components (see this
  /// class's documentation for the block form of a rotational inertia): <pre>
  ///   t_Bo = I_Bo * alpha_WB + m * p_BoBcm x a_WBo
  ///   f_Bo = -m * p_BoBcm x alpha_WB + m * a_WBo
  /// </pre>
  /// where `alpha_WB` and `a_WBo` are the rotational and translational
  /// components of the spatial acceleration `A_WB`, respectively.
  ///
  /// @note
  /// The term `F_Bo_E` computed by this operator appears in the equations of
  /// motion for a rigid body which, when written about the origin `Bo` of the
  /// body frame B (which does not necessarily need to coincide with the body's
  /// center of mass), read as: <pre>
  ///   Ftot_BBo = M_Bo_W * A_WB + b_Bo
  /// </pre>
  /// where `Ftot_BBo` is the total spatial force applied on body B at `Bo`
  /// that corresponds to the body spatial acceleration `A_WB` and `b_Bo`
  /// contains the velocity dependent gyroscopic terms (see Eq. 2.26, p. 27,
  /// in A. Jain's book).
  SpatialForce<T> operator*(const SpatialAcceleration<T>& A_WB_E) const;

  /// Multiplies `this` spatial inertia `M_BP_E` of a body B about a point P
  /// by the spatial velocity `V_WBp`, in a frame W, of the body frame B shifted
  /// to point P. Mathematically: <pre>
  ///   L_WBp_E = M_BP_E * V_WBp_E
  /// </pre>
  /// or, in terms of its rotational and translational components (see this
  /// class's documentation for the block form of a rotational inertia): <pre>
  ///   h_WB  = I_Bp * w_WB + m * p_BoBcm x v_WP
  ///   l_WBp = -m * p_BoBcm x w_WB + m * v_WP
  /// </pre>
  /// where `w_WB` and `v_WP` are the rotational and translational components of
  /// the spatial velocity `V_WBp`, respectively and, `h_WB` and `l_WBp` are the
  /// angular and linear components of the spatial momentum `L_WBp`,
  /// respectively.
  ///
  /// @note
  /// It is possible to show that `M_BP_E.Shift(p_PQ_E) * V_WBp_E.Shift(p_PQ_E)`
  /// exactly equals `L_WBp_E.Shift(p_PQ_E)`.
  SpatialMomentum<T> operator*(const SpatialVelocity<T>& V_WBp_E) const;

  /// Multiplies `this` spatial inertia by a set of spatial vectors in M⁶ stored
  /// as columns of input matrix `Mmatrix`. The top three rows of Mmatrix are
  /// expected to store the rotational components while the bottom three rows
  /// are expected to store the translational components.
  /// The output matrix is of the same size as `Mmatrix` and each j-th column
  /// stores the spatial vector in F⁶ result of multiplying `this` spatial
  /// inertia with the j-th column of `Mmatrix`.
  template <typename Derived>
  Eigen::Matrix<T, 6, Derived::ColsAtCompileTime, 0, 6,
                Derived::MaxColsAtCompileTime>
  operator*(const Eigen::MatrixBase<Derived>& Mmatrix) const {
    static_assert(is_eigen_scalar_same<Derived, T>::value,
                  "Derived must be templated on the same scalar type as this "
                  "spatial inertia.");
    if (Mmatrix.rows() != 6) {
      throw std::logic_error("Mmatrix must hold spatial vectors in M⁶.");
    }
    const auto& Vrotational = Mmatrix.template topRows<3>();
    const auto& Vtranslational = Mmatrix.template bottomRows<3>();
    const Vector3<T>& mp_BoBcm_E = CalcComMoment();  // = m * p_BoBcm
    const Matrix3<T> I_SP_E = CalcRotationalInertia().CopyToFullMatrix3();

    Eigen::Matrix<T, 6, Derived::ColsAtCompileTime, 0, 6,
                  Derived::MaxColsAtCompileTime>
        F_Bo_E(6, Mmatrix.cols());

    // Rotational component.
    F_Bo_E.template topRows<3>() =
        I_SP_E * Vrotational - Vtranslational.colwise().cross(mp_BoBcm_E);

    // Translational component.
    F_Bo_E.template bottomRows<3>() =
        Vrotational.colwise().cross(mp_BoBcm_E) + get_mass() * Vtranslational;
    return F_Bo_E;
  }

 private:
  // Constructs an all-NaN inertia.
  SpatialInertia() = default;

  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // If type T is Symbolic, validity is not checked and no exception is thrown.
  void ThrowIfNotPhysicallyValid() {
    if constexpr (scalar_predicate<T>::is_bool) {
      ThrowIfNotPhysicallyValidImpl();
    }
  }

  // Throw an exception if CreateInvalidityReport() returns an error string.
  void ThrowIfNotPhysicallyValidImpl() const;

  // Mass of the body or composite body.
  T mass_{nan()};
  // Position vector from point P to the center of mass of body or composite
  // body S, expressed in a frame E.
  Vector3<T> p_PScm_E_{Vector3<T>::Constant(nan())};
  // Rotational inertia of body or composite body S computed about point P and
  // expressed in a frame E.
  UnitInertia<T> G_SP_E_{};  // Defaults to NaN initialized inertia.

  // Appends text to an existing string with information about a SpatialInertia.
  // If the position vector p_PBcm from about-point P to Bcm (body B's center of
  // mass) is non-zero, appends I_BBcm (body B's rotational inertia about Bcm)
  // to `message`. In all cases, the central principal moments of inertia are
  // appended to `message`, e.g., to help identify a rotational inertia that
  // violates the "triangle inequality".
  void WriteExtraCentralInertiaProperties(std::string* message) const;

  // Shifts `this` spatial inertia for a body (or composite body) S from
  // about-point Scm (S's center of mass) to about-point P. In other words,
  // shifts `M_SScm_E` to `M_SP_E` (both are expressed-in frame E).
  // @param[in] p_ScmP_E Position vector from Scm to P, expressed-in frame E.
  // @pre On entry, the about-point for `this` SpatialInertia is Scm. Hence, on
  // entry the position vector p_PScm underlying `this` is the zero vector.
  void ShiftFromCenterOfMassInPlace(const Vector3<T>& p_ScmP_E) {
    DRAKE_ASSERT(p_PScm_E_ == Vector3<T>::Zero());
    G_SP_E_.ShiftFromCenterOfMassInPlace(p_ScmP_E);
    p_PScm_E_ = -p_ScmP_E;
  }

  // Calculates the spatial inertia that results from shifting `this` spatial
  // inertia for a body (or composite body) S from about-point Scm (S's center
  // of mass) to about-point P. In other words, shifts `M_SScm_E` to `M_SP_E`
  // (both are expressed-in frame E).
  // @param[in] p_ScmP_E Position vector from Scm to P, expressed-in frame E.
  // @retval M_SP_E S's spatial inertia about-point P expressed-in frame E.
  // @pre On entry, the about-point for `this` SpatialInertia is Scm. Hence, on
  // entry the position vector p_PScm underlying `this` is the zero vector.
  [[nodiscard]] SpatialInertia<T> ShiftFromCenterOfMass(
      const Vector3<T>& p_ScmP_E) const {
    SpatialInertia result(*this);
    result.ShiftFromCenterOfMassInPlace(p_ScmP_E);
    return result;
  }

  // Shifts `this` spatial inertia for a body (or composite body) S from
  // about-point P to about-point Scm (S's center of mass). In other words,
  // shifts `M_SP_E` to `M_SScm_E` (both are expressed-in frame E).
  // @note On return, the about-point for `this` SpatialInertia is Scm. Hence,
  // on return the position vector p_PScm underlying `this` is the zero vector.
  // @see SpatialInertia::ShiftToCenterOfMass(), ShiftFromCenterOfMassInPlace().
  void ShiftToCenterOfMassInPlace() {
    G_SP_E_.ShiftToCenterOfMassInPlace(p_PScm_E_);
    p_PScm_E_ = Vector3<T>::Zero();
  }

  // Calculates the spatial inertia that results from shifting `this` spatial
  // inertia for a body (or composite body) S from about-point P to
  // about-point Scm (S's center of mass). In other words, shifts
  // `M_SP_E` to `M_SScm_E` (both are expressed-in frame E).
  // @retval M_SScm_E S's spatial inertia about-point Scm expressed-in frame E.
  // @note On return, the about-point for `this` SpatialInertia is Scm. Hence,
  // on return the position vector p_PScm underlying `this` is the zero vector.
  // @see SpatialInertia::ShiftToCenterOfMassInPlace(), ShiftFromCenterOfMass().
  [[nodiscard]] SpatialInertia<T> ShiftToCenterOfMass() const {
    SpatialInertia<T> result(*this);
    result.ShiftToCenterOfMassInPlace();
    return result;
  }

  // Returns principal semi-diameters (half-lengths), associated principal axes
  // orientations, and the position of a simple uniform-density body D whose
  // shape is specified by @p inertia_shape_factor and whose spatial inertia is
  // equal to `this` spatial inertia.
  // @param[in] inertia_shape_factor real positive number in the range
  // 0 < inertia_shape_factor ≤ 1 associated with unit moment of inertia
  // (Gxx, Gyy, Gzz) formulas for G_DDcm_A, where Dcm is D's center of mass and
  // frame A contains right-handed orthogonal unit vectors Ax, Ay, Az that are
  // aligned with D's principal inertia axes.
  //-----------------------------------------|----------------------------------
  // Solid ellipsoid with semi-axes a, b, c  | Solid box with ½ lengths a, b, c
  // Gxx = 1/5 (b² + c²)                     | Gxx = 1/3 (b² + c²)
  // Gyy = 1/5 (a² + c²)                     | Gyy = 1/3 (a² + c²)
  // Gzz = 1/5 (a² + b²)                     | Gzz = 1/3 (a² + b²)
  // shape_factor = 1/5                      | shape_factor = 1/3
  //-----------------------------------------|----------------------------------
  // Ellipsoid with semi-axes a, b, c        | Box with ½ lengths a, b, c
  // Mass concentrated in 6 particles on     | Mass concentrated in 8 particles
  // the ellipsoid axes at ±a, ±b, ±c.       | at the corners of the box.
  // Gxx = 1/3 (b² + c²)                     | Gxx = 1.0 (b² + c²)
  // Gyy = 1/3 (a² + c²)                     | Gyy = 1.0 (a² + c²)
  // Gzz = 1/3 (a² + b²)                     | Gzz = 1.0 (a² + b²)
  // shape_factor = 1/3                      | shape_factor = 1
  // See @ref spatial_inertia_equivalent_shapes
  // "Spatial inertia equivalent shapes" for more details.
  // @see RotationalInertia::CalcPrincipalMomentsAndAxesOfInertia() to calculate
  // principal moments of inertia and their associated principal directions.
  // @see UnitInertia::CalcPrincipalHalfLengthsAndAxesForEquivalentShape for
  // thrown exceptions.
  // TODO(Mitiguy) Calculate the shape factor for hollow box.
  std::pair<Vector3<double>, drake::math::RigidTransform<double>>
  CalcPrincipalHalfLengthsAndPoseForEquivalentShape(
      double inertia_shape_factor) const;
};

/// Writes an instance of SpatialInertia into a std::ostream.
/// @relates SpatialInertia
template <typename T>
std::ostream& operator<<(std::ostream& out, const SpatialInertia<T>& M);

}  // namespace multibody
}  // namespace drake

// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <typename T>
struct formatter<drake::multibody::SpatialInertia<T>>
    : drake::ostream_formatter {};
}  // namespace fmt

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::SpatialInertia);
