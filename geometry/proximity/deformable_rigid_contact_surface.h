#pragma once

#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {

// Forward declaration of Tester class, so we can grant friend access.
template <typename T>
class DeformableRigidContactSurfaceTester;

/* DeformableRigidContactSurface reports the necessary information regarding the
 contact between a single deformable geometry and a single rigid
 (non-deformable) geometry as required by the contact solver. That includes
   - The deformable geometry's id.
   - The rigid geometry's id.
   - Per contact point data
      - Penetration distance (non-positive).
      - Basis of the contact frame.
      - The index of the tetrahedron containing the contact point and the
        barycentric coordinate of the contact point in that tetrahedron.
   - The polygon mesh of the contact surface.
 @tparam_double_only */
template <typename T>
class DeformableRigidContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableRigidContactSurface)

  /* Creates a contact surface between the deformable geometry with
   `deformable_id` and the rigid (non-deformable) geometry with `rigid_id`.
   @param[in] contact_surface
      The contact surface expressed in World frame.
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
      The GeometryId of the deformable geometry.
   @pre `rigid_id` and `deformable_id` must be consistent with the GeometryIds
   in `contact_surface`. */
  DeformableRigidContactSurface(const ContactSurface<T>& contact_surface,
                                std::vector<int> tetrahedron_indices,
                                std::vector<Vector4<T>> barycentric_centroids,
                                geometry::GeometryId rigid_id,
                                geometry::GeometryId deformable_id);

  GeometryId rigid_id() const { return rigid_id_; }

  GeometryId deformable_id() const { return deformable_id_; }

  /* Returns the tetrahedron indices provided at construction. */
  const std::vector<int>& tetrahedron_indices() const {
    return tetrahedron_indices_;
  }

  /* Returns the barycentric coordinates of contact polygon centroids provided
   at construction. */
  const std::vector<Vector4<T>>& barycentric_centroids() const {
    return barycentric_centroids_;
  }

  /* Returns the number of contact points between the rigid and deformable
   geometry. We have one contact point at the centroid of each contact polygon
   in the contact surface. */
  int num_contact_points() const { return num_contact_points_; }

  /* Returns the approximated penetration distance at the e-th contact point.
   @throws exception if e >= num_contact_points() or e < 0. */
  T EvaluatePenetrationDistance(int e) const;

  /* Returns the contact surface as a polygon mesh in the World frame. */
  const PolygonSurfaceMesh<T>& contact_surface_mesh() const {
     return contact_surface_mesh_;
  }

  /* Returns the rotation matrices mapping world frame quantities into contact
   frames at each contact point. The i-th contact point has its own contact
   frame Cᵢ, where R_CᵢW = R_CWs[i]. The basis vector Cᵢz is along the contact
   surface's normal at that contact point, and the origin of the contact frame
   Cᵢ is at the i-th contact point. */
  const std::vector<math::RotationMatrix<T>>& R_CWs() const { return R_CWs_; }

 private:
  friend class DeformableRigidContactSurfaceTester<T>;

  int num_contact_points_{};
  std::vector<int> tetrahedron_indices_;
  std::vector<Vector4<T>> barycentric_centroids_;
  std::vector<T> penetration_distances_;
  PolygonSurfaceMesh<T> contact_surface_mesh_;

  geometry::GeometryId rigid_id_;  // The id of the rigid geometry in contact.
  // The id of deformable geometry in contact.
  geometry::GeometryId deformable_id_;
  std::vector<math::RotationMatrix<T>> R_CWs_;  // See R_CWs().
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
