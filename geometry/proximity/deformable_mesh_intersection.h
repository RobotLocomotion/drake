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
 @param[in] pressure_field_R
     The pressure field in the rigid geometry's frame R.
 @param[in] deformable_mesh_D
     The deformable geometry's surface mesh expressed in the deformable
     geometry's frame D. We assume that triangles are oriented outward.
 @param[in] surface_index_to_volume_index
     A mapping from the deformable surface mesh's vertex index to the deformable
     volume mesh's vertex index.
 @param[in] deformable_id
     Id of the deformable geometry.
 @param[in] rigid_id
     Id of the rigid geometry.
 @param[in] rigid_mesh_R
     The rigid geometry is represented as a volume mesh, whose vertices are
     measusured and expressed in frame R.
 @param[in] rigid_bvh_R
     A bounding volume hierarchy built on the geometry contained in
     rigid_mesh_R.
 @param[in] X_RD
     The pose of the deformable geometry's frame D in rigid geometry's frame R.
 @param[in, out] deformable_contact
     The deformable contact data to be appended to.
 @pre deformable_contact != nullptr. */
void AddDeformableRigidContactSurface(
    const VolumeMeshFieldLinear<double, double>& pressure_field_R,
    const DeformableSurfaceMeshWithBvh<double>& deformable_mesh_D,
    const std::vector<int>& surface_index_to_volume_index,
    GeometryId deformable_id, GeometryId rigid_id,
    const VolumeMesh<double>& rigid_mesh_R,
    const Bvh<Obb, VolumeMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_RD,
    DeformableContact<double>* deformable_contact);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
