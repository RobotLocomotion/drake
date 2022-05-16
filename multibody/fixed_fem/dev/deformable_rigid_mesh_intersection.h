#pragma once

#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/deformable_contact_surface.h"
#include "drake/geometry/proximity/deformable_volume_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace fem {

/** Computes the contact between a deformable tet mesh and a rigid tri mesh. The
 contact is characterized by a collection of per-polygon quantities (see
 DeformableContactSurface) associated with an implicit underlying polygon mesh.

 If there is no contact, the result will be empty.

 @param tet_mesh_D   The deformable tetrahedral mesh, with vertex positions
                     measured and expressed in the *deformable* mesh's frame D.
 @param tri_mesh_R   The triangle mesh, with vertex positions measured and
                     expressed in the *rigid* mesh's frame R.
 @param bvh_R        The bounding volume hierarchy for the triangle mesh,
                     measured and expressed in the *rigid* mesh's frame R.
 @param X_DR         The pose of the triangle mesh in the volume mesh's frame.
 @returns The collection of contact data associated with an implicit contact
          surface formed by the intersection of the volume and surface meshes.
          If there is no intersection, the resulting surface will report as
          "empty".  */
template <typename T>
geometry::internal::DeformableContactSurface<T> ComputeTetMeshTriMeshContact(
    const geometry::internal::DeformableVolumeMesh<T>& tet_mesh_D,
    const geometry::TriangleSurfaceMesh<double>& tri_mesh_R,
    const geometry::internal::Bvh<geometry::internal::Obb,
                                  geometry::TriangleSurfaceMesh<double>>& bvh_R,
    const math::RigidTransform<T>& X_DR);
}  // namespace fem
}  // namespace multibody
}  // namespace drake
