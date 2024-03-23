#pragma once

#include <memory>
#include <unordered_set>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/query_results/deformable_contact.h"

namespace drake {
namespace geometry {
namespace internal {

/* Computes the contact surface between two deformable geometries and appends
 it to the existing data.
 @param[in] deformable0_sdf_W
     The approximated signed distance field for the first deformable geometry
     in World frame.
 @param[in] deformable0_mesh_W
     The first deformable geometry's mesh expressed in World frame.
 @param[in] deformable0_id
     Id of the first deformable geometry.
 @param[in] deformable1_sdf_W
     The approximated signed distance field for the second deformable
     geometry in World frame.
 @param[in] deformable1_mesh_W
     The second deformable geometry's mesh expressed in World frame.
 @param[in] deformable1_id
     Id of the second deformable geometry.
 @param[in, out] deformable_contact
     The deformable contact data to be appended to.

 @pre deformable0_sdf_W.mesh() refers to deformable0_mesh_W.mesh().
 @pre deformable1_sdf_W.mesh() refers to deformable1_mesh_W.mesh().
 @pre deformable_contact != nullptr.
 @pre deformable1_id < deformable0_id  */
void AddDeformableDeformableContactSurface(
    const VolumeMeshFieldLinear<double, double>& deformable0_sdf_W,
    const DeformableVolumeMeshWithBvh<double>& deformable0_mesh_W,
    GeometryId deformable0_id,
    const VolumeMeshFieldLinear<double, double>& deformable1_sdf_W,
    const DeformableVolumeMeshWithBvh<double>& deformable1_mesh_W,
    GeometryId deformable1_id, DeformableContact<double>* deformable_contact);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
