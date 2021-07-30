#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/surface_mesh_field.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/** Sets up the equilibrium plane between the two linear functions from the
 two tetrahedra. The normal vector of the plane will point *out* of field1
 and *into* field0.

 @retval true if the equilibrium plane exists. The plane may not exist, for
         example, if the two fields have the same gradient vectors, which
         means they have equal values everywhere in space or nowhere at all.

 @pre For each field, the field value increases from the boundary into the
      interior.
 */
template <typename T>
bool CalcEquilibriumPlane(VolumeElementIndex element0,
                          const VolumeMeshFieldLinear<double, double>& field0_M,
                          VolumeElementIndex element1,
                          const VolumeMeshFieldLinear<double, double>& field1_N,
                          const math::RigidTransform<T>& X_MN,
                          Plane<T>* plane_M);
template <typename T>
void IntersectPlaneTetrahedron(
    VolumeElementIndex tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M,
    const Plane<T>& plane_M, std::vector<Vector3<T>>* polygon_M);

/*
 @retval `polygon_M` contains the locations of vertices of the contact polygon.
         `polygon_nhat_M` is its unit normal vector pointing out of element1
         and into element0.
 */
template <typename T>
std::vector<Vector3<T>> IntersectTetrahedra(
    VolumeElementIndex element0,
    const VolumeMeshFieldLinear<double, double>& field0_M,
    VolumeElementIndex element1,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const math::RigidTransform<T>& X_MN,
    const Plane<T>& equilibrium_plane_M);

bool IsPlaneNormalAlongPressureGradient(
    const Vector3<double>& nhat_M, VolumeElementIndex tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M);

void FieldIntersection(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
    const math::RigidTransform<double>& X_MN,
    ContactPolygonRepresentation representation,
    std::unique_ptr<SurfaceMesh<double>>* surface_MN_M,
    std::unique_ptr<SurfaceMeshFieldLinear<double, double>>* e_MN_M,
    std::vector<Vector3<double>>* grad_eM_Ms,
    std::vector<Vector3<double>>* grad_eN_Ms);

std::unique_ptr<ContactSurface<double>>
ComputeContactSurfaceFromCompliantVolumes(
  GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
  const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
  const math::RigidTransform<double>& X_WF,
  GeometryId id1, const VolumeMeshFieldLinear<double, double>& field1_G,
  const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
  const math::RigidTransform<double>& X_WG,
  ContactPolygonRepresentation representation);

std::unique_ptr<ContactSurface<AutoDiffXd>>
ComputeContactSurfaceFromCompliantVolumes(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<AutoDiffXd>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<AutoDiffXd>& X_WG,
    ContactPolygonRepresentation representation);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
