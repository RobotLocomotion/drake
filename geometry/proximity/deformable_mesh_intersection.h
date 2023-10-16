#pragma once

#include <memory>
#include <vector>

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/query_results/deformable_contact.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Computes the contact surface between a deformable geometry and a rigid
 geometry and appends it to existing data.
 @param[in] deformable_sdf_D
     The approximated signed distance field for the deformable geometry in the
     deformable geometry's frame D.
 @param[in] deformable_mesh_D
     The deformable geometry's mesh expressed in the deformable geometry's frame
     D.
 @param[in] deformable_id
     Id of the deformable geometry.
 @param[in] rigid_id
     Id of the rigid geometry.
 @param[in] rigid_mesh_R
     The rigid geometry is represented as a surface mesh, whose vertices are
     measusured and expressed in frame R. We assume that triangles are oriented
     outward.
 @param[in] rigid_bvh_R
     A bounding volume hierarchy built on the geometry contained in
     rigid_mesh_R.
 @param[in] X_DR
     The pose of the rigid geometry's frame R in deformable geometry's frame D.
 @param[in, out] deformable_contact
     The deformable contact data to be appended to.
 @pre deformable_contact != nullptr. */
void AddDeformableRigidContactSurface(
    const VolumeMeshFieldLinear<double, double>& deformable_sdf,
    const DeformableVolumeMesh<double>& deformable_mesh,
    GeometryId deformable_id, GeometryId rigid_id,
    const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_DR,
    DeformableContact<double>* deformable_contact);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
