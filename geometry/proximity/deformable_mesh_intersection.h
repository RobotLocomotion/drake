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
 geometry and appends it to existing data. The resulting contact surface lies on
 the surface of the deformable geometry (possibly with additional subdivisions
 due to the intersection with the volume of the rigid geometry). This choice of
 contact surface representation (as opposed to making the contact surface lie on
 the surface of the rigid geometry) is made because:

 1. it often in practice reduces the number of contact points and the number of
    participating vertices (i.e. the vertices incident to the deformable
    tetrahedra containing at least one contact point),
 2. it prevents contact constraints that only involve internal deformable
    vertices, and
 3. It mitigates the “sticking” artifact we observe when a deformable body
    is in contact a rigid shape with sharp features (i.e., discontinuous
    normals), preventing unintended adhesion and ensuring the two bodies cleanly
    separate.

 @param[in] deformable_mesh_D
     The deformable geometry's surface mesh expressed in the deformable
     geometry's frame D. We assume that triangles are oriented outward.
 @param[in] deformable_volume_mesh_D
     The deformable geometry's volume mesh expressed in the deformable
     geometry's frame D.
 @param[in] surface_index_to_volume_index
     A mapping from the deformable surface mesh's vertex index to the deformable
     volume mesh's vertex index.
 @param[in] surface_tri_to_volume_tet
     A mapping from the deformable surface mesh's triangle index to the
     deformable volume mesh's tetrahedron index.
 @param[in] deformable_id
     Id of the deformable geometry.
 @param[in] rigid_id
     Id of the rigid geometry.
 @param[in] pressure_field_R
     The pressure field in the rigid geometry's frame R.
 @param[in] rigid_bvh_R
     A bounding volume hierarchy built on the geometry contained in
     rigid_mesh_R.
 @param[in] X_RD
     The pose of the deformable geometry's frame D in rigid geometry's frame R.
 @param[in, out] deformable_contact
     The deformable contact data to be appended to.
 @pre deformable_contact != nullptr. */
void AddDeformableRigidContactSurface(
    const DeformableSurfaceMeshWithBvh<double>& deformable_mesh_D,
    const DeformableVolumeMeshWithBvh<double>& deformable_volume_mesh_D,
    const std::vector<int>& surface_index_to_volume_index,
    const std::vector<int>& surface_tri_to_volume_tet, GeometryId deformable_id,
    GeometryId rigid_id,
    const VolumeMeshFieldLinear<double, double>& pressure_field_R,
    const Bvh<Obb, VolumeMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_RD,
    DeformableContact<double>* deformable_contact);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
