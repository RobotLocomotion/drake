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

/* A wrapper around ContactSurface that provides additional information for
 contact between a deformable geometry and a rigid (non-deformable) geometry.
 @tparam_double_only */
template <typename T>
class DeformableRigidContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableRigidContactSurface)
  /* Creates a contact surface between the deformable geometry with
   `deofrmable_id` and the rigid (non-deformable) geometry with `rigid_id`.
   @param[in] contact_surface
      The contact surface expressed in World frame.
   @param[out] tetrahedron_indices
      Each contact polygon in `contact_surface` is completely contained within
      one tetrahedron of the deformable mesh. For the i-th contact polygon in
      the contact surface, tetrahedron_indices[i] contains the index of the
      containing tetrahedron.
   @param[out] barycentric_centroids
      Barycentric coordinates of centroids of contact polygons with respect to
      their containing tetrahedra with the same index semantics as
      `tetrahedron_indices`. */
  DeformableRigidContactSurface(ContactSurface<T> contact_surface,
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

  /* Returns the number of contact points between the rigid and deformable body.
   We have one contact point at the centroid of each contact polygon in the
   contact surface. */
  int num_contact_points() const { return contact_surface_.num_faces(); }

  /* Returns the approximated penetration distance at the e-th contact point.
   @throws exception if e >= num_contact_points() or e < 0. */
  T EvaluatePenetrationDistance(int e) const;

  /* Returns the rotation matrices mapping world frame quantities into contact
   frames at each contact point. The i-th contact point has its own contact
   frame Cᵢ, where R_CᵢW = R_CWs[i]. The basis vector Cᵢz is along the contact
   surface's normal at that contact point. */
  const std::vector<math::RotationMatrix<T>>& R_CWs() const { return R_CWs_; }

 private:
  friend class DeformableRigidContactSurfaceTester<T>;

  ContactSurface<T> contact_surface_;
  std::vector<int> tetrahedron_indices_;
  std::vector<Vector4<T>> barycentric_centroids_;

  geometry::GeometryId rigid_id_;  // The id of the rigid geometry in contact.
  geometry::GeometryId deformable_id_;  // The id of deformable body in contact.
  std::vector<math::RotationMatrix<T>> R_CWs_;  // See R_CWs().
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
