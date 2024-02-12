#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/plane.h"
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

/* %VolumeIntersector performs an intersection algorithm between two
 tetrahedral meshes with scalar fields to get a contact surface, on which
 the two fields have equal values. Hydroelastics and deformables share this
 code.

 @tparam MeshBuilder
   The type of mesh-output builder of the contact surface. It can be
   TriMeshBuilder<T> or PolyMeshBuilder<T> for T = double or AutoDiffXd.
   See contact_surface_utility.h for details.

 @tparam BvType
   The type of bounding volumes for the tetrahedra. It can be Obb
   for hydroelastics or Aabb for deformables.  */
template <class MeshBuilder, class BvType>
class VolumeIntersector {
 public:
  // T is double or AutoDiffXd.
  using T = typename MeshBuilder::ScalarType;
  // Type of output surface mesh. It could be TriangleSurfaceMesh<T> or
  // PolygonSurfaceMesh<T>.
  using MeshType = typename MeshBuilder::MeshType;
  // Type of output surface mesh field. It could be
  // TriangleSurfaceMeshFieldLinear<T, T> or PolygonSurfaceMeshFieldLinear<T,T>.
  using FieldType = typename MeshBuilder::FieldType;

  VolumeIntersector() {}

  /* Creates the mesh and the scalar field on the contact surface between two
   tetrahedral meshes with scalar fields. The output surface mesh is posed in
   frame M of the first tetrahedral mesh.

   Hydroelastics and deformables use this function when the scalar fields
   are pressure fields and signed distance fields, respectively.

   @param[in] field0_M  The first geometry represented as a tetrahedral
                        mesh with scalar field, expressed in frame M.
   @param[in] bvh0_M    The bounding volume hierarchy built on the tetrahedral
                        mesh of `field0_M`.
   @param[in] field1_N  The second geometry represented as a
                        tetrahedral mesh with scalar field, expressed in
                        frame N.
   @param[in] bvh1_N    The bounding volume hierarchy built on the tetrahedral
                        mesh of `field1_N`.
   @param[in] X_MN      The pose of frame N in frame M.
   @param[out] surface_01_M  The output mesh of the contact surface between
                        the two geometries expressed in frame M. The surface
                        normal is in the direction of increasing the difference
                        field0 - field1. Usually it is the direction
                        of increasing field0 and decreasing field1.
   @param[out] e_01_M   The scalar field on the contact surface, expressed in
                        frame M.
   @note  The output surface mesh may have duplicate vertices.
   */
  void IntersectFields(const VolumeMeshFieldLinear<double, double>& field0_M,
                       const Bvh<BvType, VolumeMesh<double>>& bvh0_M,
                       const VolumeMeshFieldLinear<double, double>& field1_N,
                       const Bvh<BvType, VolumeMesh<double>>& bvh1_N,
                       const math::RigidTransform<T>& X_MN,
                       std::unique_ptr<MeshType>* surface_01_M,
                       std::unique_ptr<FieldType>* e_01_M);

  /* Returns the index of tetrahedron in the first mesh containing the
   i-th contact polygon.
   @pre IntersectFields() was called already.  */
  int tet0_of_polygon(int i) const { return tet0_of_contact_polygon_[i]; }
  /* Returns the index of tetrahedron in the second mesh containing the
   i-th contact polygon.
   @pre IntersectFields() was called already.  */
  int tet1_of_polygon(int i) const { return tet1_of_contact_polygon_[i]; }

 private:
  /* Internal function to process a possible contact between two tetrahedra
   in the meshes with scalar fields.

   @param[in] field0_M  The first tetrahedral mesh with scalar field, expressed
                        in frame M.
   @param[in] field1_N  The second tetrahedral mesh with scalar field,
                        expressed in frame N.
   @param[in] X_MN      The pose of frame N in frame M.
   @param[in] R_NM      The rotation matrix of frame N in frame M. It is the
                        same as X_MN.rotation().inverse(). This is a redundant
                        parameter for performance reason.
   @param[in] tet0      Index of the first tetrahedron in the first mesh.
   @param[in] tet1      Index of the second tetrahedron in the second mesh.
   @param[in, out] builder_M   If there is a non-empty intersection, the
                          intersecting polygon (contact polygon) and its
                          field values are added into builder_M. The added
                          polygon is in frame M.  */
  void CalcContactPolygon(const VolumeMeshFieldLinear<double, double>& field0_M,
                          const VolumeMeshFieldLinear<double, double>& field1_N,
                          const math::RigidTransform<T>& X_MN,
                          const math::RotationMatrix<T>& R_NM, int tet0,
                          int tet1, MeshBuilder* builder_M);

  // List of tetrahedron indices in the meshes of field0 and field1. One
  // index per contact polygon;
  std::vector<int> tet0_of_contact_polygon_{};
  std::vector<int> tet1_of_contact_polygon_{};
};

/* %HydroelasticVolumeIntersector performs an intersection algorithm between
 two compliant hydroelastic geometries represented as tetrahedral meshes
 with pressure fields to get a contact surface, on which the two fields
 have equal values.

 @tparam MeshBuilder
   The type of mesh-output builder of the contact surface. It can be
   TriMeshBuilder<T> or PolyMeshBuilder<T> for T = double or AutoDiffXd.
   See contact_surface_utility.h for details.  */
template <class MeshBuilder>
class HydroelasticVolumeIntersector {
 public:
  // T is double or AutoDiffXd.
  using T = typename MeshBuilder::ScalarType;

  /* Computes the contact surface between two compliant hydroelastic
   geometries. The output contact surface is posed in World frame. It is a
   helper of ComputeContactSurfaceFromCompliantVolumes().

   @param[in] id0        ID of the first geometry.
   @param[in] field0_M   Pressure field on the tetrahedral mesh of the first
                         geometry, expressed in frame M.
   @param[in] bvh0_M     A bounding volume hierarchy built on the mesh of
                         `field0_M`.
   @param[in] X_WM       The pose of the first geometry in World.
   @param[in] id1        ID of the second geometry.
   @param[in] field1_N   Pressure field on the tetrahedral mesh of the second
                         geometry, expressed in frame N.
   @param[in] bvh1_N     A bounding volume hierarchy built on the mesh of
                         `field1_N`.
   @param[in] X_WN       The pose of the second geometry in World.
   @param[out] contact_surface_W   The contact surface, whose type (e.g.,
                         triangles or polygons) depends on the type parameter
                         MeshBuilder. It is expressed in World frame.
                         If there is no contact, nullptr is returned.  */
  void IntersectCompliantVolumes(
      GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_M,
      const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
      const math::RigidTransform<T>& X_WM, GeometryId id1,
      const VolumeMeshFieldLinear<double, double>& field1_N,
      const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
      const math::RigidTransform<T>& X_WN,
      std::unique_ptr<ContactSurface<T>>* contact_surface_W);
};

/* Computes the contact surface between two compliant hydroelastic geometries
 with the requested representation. The output contact surface is posed
 in World frame.

 @param[in] id0        Id of the first geometry.
 @param[in] field0_M   Pressure field on the tetrahedral mesh of the first
                       geometry, expressed in frame M.
 @param[in] bvh0_M     A bounding volume hierarchy built on the mesh of
                       `field0_M`.
 @param[in] X_WM       The pose of the first geometry in World.
 @param[in] id1        Id of the second geometry.
 @param[in] field1_N   Pressure field on the tetrahedral mesh of the second
                       geometry, expressed in frame N.
 @param[in] bvh1_N     A bounding volume hierarchy built on the mesh of
                       `field1_N`.
 @param[in] X_WN       The pose of the second geometry in World.
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
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_M,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
    const math::RigidTransform<T>& X_WM, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
    const math::RigidTransform<T>& X_WN,
    HydroelasticContactRepresentation representation);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
