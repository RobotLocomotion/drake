#pragma once

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {

/* Characterization of the contact between a deformable geometry and a rigid
 (non-deformable) geometry. The characterization includes:
   - The geometry ids for the deformable and rigid geometries.
   - A polygonal "contact" mesh representing the contact interface manifold. In
     fact it is the portion of the surface of the rigid geometry enclosed by the
     deformable mesh.
   - The *abstract* concept of contact "points". We get one point for each
     polygon in the contact mesh. The point is the polygon's centroid. We have
     the following data associated with each point:
       - The approximate penetration distance into the deformable mesh
         (non-positive).
       - A basis for a contact frame.
       - The index of the tetrahedron containing the contact point and the
         Barycentric coordinate of the contact point in that tetrahedron. Note:
         we don't explicitly expose the Cartesian coordinates of the point.
 @tparam_double_only */
template <typename T>
struct DeformableRigidContactSurface {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableRigidContactSurface)

  /* Creates a contact surface between the deformable geometry with
   `deformable_id` and the rigid (non-deformable) geometry with `rigid_id`.
   @param[in] contact_surface_mesh_W
      The contact surface expressed in World frame. The normal of the contact
      surface points out of the rigid geometry.
   @param[in] signed_distance_field
      An _approximate_ signed distance field of penetation on
      `contact_surface_mesh_W`. The values in the field are non-positive.
   @param[in] tetrahedron_indices
      Each contact polygon in `contact_surface` is completely contained within
      one tetrahedron of the deformable mesh. For the i-th contact polygon in
      the contact surface, tetrahedron_indices[i] contains the index of the
      containing tetrahedron.
   @param[in] barycentric_centroids
      Barycentric coordinates of centroids of contact polygons with respect to
      their containing tetrahedra with the same index semantics as
      `tetrahedron_indices`.
   @param[in] rigid_id
      The GeometryId of the rigid geometry.
   @param[in] deformable_id
      The GeometryId of the deformable geometry. */
  DeformableRigidContactSurface(PolygonSurfaceMesh<T>&& contact_surface_mesh_W,
                                std::vector<T>&& penetration_distances,
                                std::vector<int>&& tetrahedron_indices,
                                std::vector<Vector4<T>>&& barycentric_centroids,
                                GeometryId rigid_id, GeometryId deformable_id);

  int num_contact_points() const { return contact_surface_mesh_W_.num_faces(); }

  /* Releases all the per-contact-point data stored in this
   DeformableRigidContactSurface. After this is called,
   DeformableRigidContactSurface gives up ownership of the data, and should
   therefore be discarded. */
  std::tuple<PolygonSurfaceMesh<T>, std::vector<T>, std::vector<int>,
             std::vector<Vector4<T>>, std::vector<math::RotationMatrix<T>>>
  release_data();

  GeometryId deformable_id() const { return deformable_id_; }
  GeometryId rigid_id() const { return rigid_id_; }

  PolygonSurfaceMesh<T> contact_surface_mesh_W_;
  /* The approximated penetration distance at each contact point. */
  std::vector<T> penetration_distances_;
  std::vector<int> tetrahedron_indices_;
  std::vector<Vector4<T>> barycentric_centroids_;
  /* The id of the rigid geometry in contact. */
  GeometryId rigid_id_;
  /* The id of deformable geometry in contact. */
  GeometryId deformable_id_;
  /* The *inverses* of the bases for the per-point contact frames
   expressed in the world frame. The contact frame C has its origin at the
   contact point (polygon centroid) and is oriented with Cz in the polygon's
   outward-facing normal direction. Cx and Cy are arbitrarily oriented. The iᵗʰ
   contact point is associated with frame Cᵢ (with basis R_WCᵢ). Its
   inverse R⁻¹_WCᵢ = R_CᵢW can be found in R_CWs[i]. */
  std::vector<math::RotationMatrix<T>> R_CWs_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
