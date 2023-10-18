#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh_field.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Sets up the equilibrium plane between two linear functions where they
 are equal.

 The first linear function f₀:ℝ³→ ℝ is specified by the tetrahedron index
 `element0` into the piecewise-linear field `field0_M`, expressed in frame M.
 The second function f₁:ℝ³→ ℝ is specified by the tetrahedron index
 `element1` into the piecewise-linear field `field1_N`, expressed in frame N.

 @param[in] X_MN Relative pose of frame N in frame M.

 @param[out] plane_M  Equilibrium plane expressed in frame M if it exists.
 It is not set if the gradient of the two field are deemed identical.
 Its unit normal vector n̂ will point *out of* f₁ and *into* f₀, i.e.,
 n̂ points in the direction of increasing f₀ and decreasing f₁:

                 n̂ = (∇f₀−∇f₁)/‖∇f₀−∇f₁‖

 @retval true if the equilibrium plane exists. The plane may not exist if the
  two functions have the same gradient vector, which means they are equal
  everywhere, or they are nowhere equal. We use an internal threshold to
  decide.

 @note The equilibrium plane may or may not intersect the tetrahedra.

 @throw std::runtime_error if `field0_M` or `field1_N` has no gradient
 calculated.

 @tparam T A valid Eigen scalar.
 */
template <typename T>
bool CalcEquilibriumPlane(int element0,
                          const VolumeMeshFieldLinear<double, double>& field0_M,
                          int element1,
                          const VolumeMeshFieldLinear<double, double>& field1_N,
                          const math::RigidTransform<T>& X_MN,
                          Plane<T>* plane_M);

/* Computes the intersecting polygon of two tetrahedra and a plane.

 @param[in] element0  The index of the tetrahedron in the first mesh.
 @param[in] mesh0_M   The first tetrahedral mesh expressed in frame M.
 @param[in] element1  The index of the tetrahedron in the second mesh.
 @param[in] mesh1_N   The second tetrahedral mesh expressed in frame N.
 @param[in] X_MN      Pose of frame N in frame M.
 @param[in] plane_M   The plane between the two tetrahedra, expressed in
                      frame M.

 @retval `polygon_M`  A sequence of vertex positions of the intersecting
                      polygon expressed in frame M. Its unit normal vector
                      in CCW winding order convention is in the same direction
                      as the plane's normal vector.

 @tparam T A valid Eigen scalar.
 */
template <typename T>
std::vector<Vector3<T>> IntersectTetrahedra(
    int element0, const VolumeMesh<double>& mesh0_M, int element1,
    const VolumeMesh<double>& mesh1_N, const math::RigidTransform<T>& X_MN,
    const Plane<T>& equilibrium_plane_M);

// TODO(DamrongGuoy): Move IsPlaneNormalAlongPressureGradient() into
//  contact_surface_utility for code reuse if and when we work on compliant
//  mesh vs. compliant halfspace in hydroelastic contact model. At that time,
//  we might support dual frames. Right now it's all in single frame M.

/* Determines if the indicated normal vector of a plane is "in the direction"
 of the field gradient in a given linear-field tetrahedron.

 @param nhat_M       The normal to test against, expressed in the same frame M
                     of the piecewise linear field.
 @param tetrahedron  The index of the tetrahedron in the mesh of the field.
 @param field_M      The piecewise linear field expressed in frame M.
 @pre  `nhat_M` is unit length.
 @return `true` if the angle between `nhat_M` and the field gradient vector
         lies within a hard-coded tolerance.
 */
template <typename T>
bool IsPlaneNormalAlongPressureGradient(
    const Vector3<T>& nhat_M, int tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M);

/* Creates the mesh and the pressure field of the contact surface between two
 tetrahedral meshes with pressure fields. The output surface mesh is posed in
 frame M of the first tetrahedral mesh.

 @param[in] field0_M  The first compliant geometry represented as a tetrahedral
                      mesh with pressure field, expressed in frame M.
 @param[in] bvh0_M    The bounding volume hierarchy built on the tetrahedral
                      mesh of `field0_M`.
 @param[in] field1_N  The second compliant geometry represented as a
                      tetrahedral mesh with pressure field, expressed in
                      frame N.
 @param[in] bvh1_N    The bounding volume hierarchy built on the tetrahedral
                      mesh of `field1_N`.
 @param[in] X_MN      The pose of frame N in frame M.
 @param[in] builder   The builder of the output mesh and the contact pressure
                      field.
 @param[out] surface_01_M  The output mesh of the contact surface between
                      the two compliant geometries of `field0_M` and
                      `field1_N`, expressed in frame M.
 @param[out] e_01_M   The pressure field on the contact surface, expressed in
                      frame M.
 @param[out] grad_e0_Ms  The pressure gradients of `field0` on the mesh of
                     the contact surface, expressed in frame M (one sample
                     per face in `surface_01_M`).
 @param[out] grad_e1_Ms  The pressure gradients of `field1` on the mesh of
                     the contact surface, expressed in frame M (one sample
                     per face in `surface_01_M`).
 @note  The output surface mesh may have duplicate vertices.
 @tparam MeshType    Type of output surface mesh: TriangleSurfaceMesh<T> or
                     PolygonSurfaceMesh<T>, where T is double or AutoDiffXd.
 @tparam MeshBuilder Type of the output mesh builder: TriMeshBuilder<T> or
                     PolyMeshBuilder<T>, where T is double or AutoDiffXd.
                     (See the documentation in contact_surface_utility.h
                     for details.)
 @pre The MeshType and the MeshBuilder must be consistent.
 */
template <class MeshType, class MeshBuilder,
          typename T = typename MeshType::ScalarType,
          class FieldType = MeshFieldLinear<T, MeshType>>
void IntersectFields(const VolumeMeshFieldLinear<double, double>& field0_M,
                     const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
                     const VolumeMeshFieldLinear<double, double>& field1_N,
                     const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
                     const math::RigidTransform<T>& X_MN,
                     std::unique_ptr<MeshType>* surface_01_M,
                     std::unique_ptr<FieldType>* e_01_M,
                     std::vector<Vector3<T>>* grad_e0_Ms,
                     std::vector<Vector3<T>>* grad_e1_Ms);

/* Computes the contact surface between two compliant hydroelastic geometries
 given a specific mesh-builder instance. The output contact surface is posed
 in World frame.

 @param[in] id0        ID of the first geometry.
 @param[in] field0_F   Pressure field on the tetrahedral mesh of the first
                       geometry, expressed in frame F.
 @param[in] bvh0_F     A bounding volume hierarchy built on the mesh of
                       `field0_F`.
 @param[in] X_WF       The pose of the first geometry in World.
 @param[in] id1        ID of the second geometry.
 @param[in] field1_G   Pressure field on the tetrahedral mesh of the second
                       geometry, expressed in frame G.
 @param[in] bvh1_G     A bounding volume hierarchy built on the mesh of
                       `field1_G`.
 @param[in] X_WG       The pose of the second geometry in World.
 @param[in] builder   The builder of the output mesh and the contact pressure
                      field.
 @returns The contact surface, whose type (e.g., triangles or polygons) depends
          on the given MeshBuilder. It is expressed in World frame.
          If there is no contact, nullptr is returned.

 @tparam MeshType    Type of output surface mesh: TriangleSurfaceMesh<T> or
                     PolygonSurfaceMesh<T>, where T is double or AutoDiffXd.
 @tparam MeshBuilder Type of the output mesh builder: TriMeshBuilder<T> or
                     PolyMeshBuilder<T>, where T is double or AutoDiffXd.
 @pre The MeshType and the MeshBuilder must be consistent.
 */
template <class MeshType, class MeshBuilder,
          typename T = typename MeshType::ScalarType,
          class FieldType = MeshFieldLinear<T, MeshType>>
std::unique_ptr<ContactSurface<T>> IntersectCompliantVolumes(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<T>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<T>& X_WG);

/* Computes the contact surface between two compliant hydroelastic geometries
 with the requested representation. The output contact surface is posed
 in World frame.

 @param[in] id0        Id of the first geometry.
 @param[in] field0_F   Pressure field on the tetrahedral mesh of the first
                       geometry, expressed in frame F.
 @param[in] bvh0_F     A bounding volume hierarchy built on the mesh of
                       `field0_F`.
 @param[in] X_WF       The pose of the first geometry in World.
 @param[in] id1        Id of the second geometry.
 @param[in] field1_G   Pressure field on the tetrahedral mesh of the second
                       geometry, expressed in frame G.
 @param[in] bvh1_G     A bounding volume hierarchy built on the mesh of
                       `field1_G`.
 @param[in] X_WG       The pose of the second geometry in World.
 @param[in] representation  The preferred representation of each contact
                            polygon.

 @returns the contact surface between the two geometries (see ContactSurface)
          in the requested representation. It is expressed in World frame.
          If there is no contact, nullptr is returned.

 @tparam_nonsymbolic_scalar

 @note This definition of the function assumes that the meshes involved are
 *double* valued -- in other words, they are constant parameters in the
 calculation. If derivatives are to be found, the point of injection is through
 the definition of the relative position of the two meshes.
*/
template <typename T>
std::unique_ptr<ContactSurface<T>> ComputeContactSurfaceFromCompliantVolumes(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<T>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<T>& X_WG,
    HydroelasticContactRepresentation representation);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
