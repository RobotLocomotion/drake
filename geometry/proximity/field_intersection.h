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

/* Calculates the intersecting polygon between a plane and a tetrahedron */
template <typename T>
void IntersectPlaneTetrahedron(
    int tetrahedron, const VolumeMesh<double>& mesh_M,
    const Plane<T>& plane_M, std::vector<Vector3<T>>* polygon_M);

/*
 @retval `polygon_M` contains the locations of vertices of the contact polygon.
         `polygon_nhat_M` is its unit normal vector pointing out of element1
         and into element0.
 */
template <typename T>
std::vector<Vector3<T>> IntersectTetrahedra(
    int element0, const VolumeMeshFieldLinear<double, double>& field0_M,
    int element1, const VolumeMeshFieldLinear<double, double>& field1_N,
    const math::RigidTransform<T>& X_MN, const Plane<T>& equilibrium_plane_M);

bool IsPlaneNormalAlongPressureGradient(
    const Vector3<double>& nhat_M, int tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M);

/*
 @tparam MeshType    Type of output surface mesh: TriangleSurfaceMesh<T> or
                     PolygonSurfaceMesh<T>, where T is double or AutoDiffXd.
 @tparam MeshBuilder Type of the output mesh builder: TriMeshBuilder<T> or
                     PolyMeshBuilder<T>, where T is double or AutoDiffXd.
 @pre The MeshType and the MeshBuilder must correspond.
 */
template <class MeshType, class MeshBuilder,
    typename T = typename MeshType::ScalarType,
    class FieldType = MeshFieldLinear<T, MeshType>>
void FieldIntersection(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
    const math::RigidTransform<double>& X_MN,
    MeshBuilder builder,
    std::unique_ptr<MeshType>* surface_MN_M,
    std::unique_ptr<FieldType>* e_01_M,
    std::vector<Vector3<double>>* grad_eM_Ms,
    std::vector<Vector3<double>>* grad_eN_Ms);

/*
 @tparam MeshType    Type of output surface mesh: TriangleSurfaceMesh<T> or
                     PolygonSurfaceMesh<T>, where T is double or AutoDiffXd.
 @tparam MeshBuilder Type of the output mesh builder: TriMeshBuilder<T> or
                     PolyMeshBuilder<T>, where T is double or AutoDiffXd.
 @pre The MeshType and the MeshBuilder must correspond.
 */
template <class MeshType, class MeshBuilder,
          typename T = typename MeshType::ScalarType,
          class FieldType = MeshFieldLinear<T, MeshType>>
std::unique_ptr<ContactSurface<double>> IntersectCompliantVolumes(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<double>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<double>& X_WG, MeshBuilder builder);

std::unique_ptr<ContactSurface<double>>
ComputeContactSurfaceFromCompliantVolumes(
  GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
  const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
  const math::RigidTransform<double>& X_WF,
  GeometryId id1, const VolumeMeshFieldLinear<double, double>& field1_G,
  const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
  const math::RigidTransform<double>& X_WG,
  HydroelasticContactRepresentation representation);

std::unique_ptr<ContactSurface<AutoDiffXd>>
ComputeContactSurfaceFromCompliantVolumes(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<AutoDiffXd>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<AutoDiffXd>& X_WG,
    HydroelasticContactRepresentation representation);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
