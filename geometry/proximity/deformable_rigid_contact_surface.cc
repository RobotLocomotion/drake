#include "drake/geometry/proximity/deformable_rigid_contact_surface.h"

#include <iostream>

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
DeformableRigidContactSurface<T>::DeformableRigidContactSurface(
    std::unique_ptr<PolygonSurfaceMesh<T>> contact_surface_mesh_W,
    const PolygonSurfaceMeshFieldLinear<T, T>& signed_distance_field,
    std::vector<int> tetrahedron_indices,
    std::vector<Vector4<T>> barycentric_centroids,
    geometry::GeometryId rigid_id, geometry::GeometryId deformable_id)
    : contact_surface_mesh_W_(std::move(contact_surface_mesh_W)),
      // Make a copy of the number of contact points since we might release the
      // mesh.
      num_contact_points_(contact_surface_mesh_W_->num_faces()),
      tetrahedron_indices_(std::move(tetrahedron_indices)),
      barycentric_centroids_(std::move(barycentric_centroids)),
      rigid_id_(rigid_id),
      deformable_id_(deformable_id),
      R_CWs_(contact_surface_mesh_W_->num_faces()) {
  penetration_distances_.reserve(contact_surface_mesh_W_->num_faces());
  for (int i = 0; i < contact_surface_mesh_W_->num_faces(); ++i) {
    /* The normal of the face lies in the direction of the contact frame's
     z-axis. */
    const Vector3<T>& Cz_W = contact_surface_mesh_W_->face_normal(i);
    constexpr int kZAxis = 2;
    auto R_WC = math::RotationMatrix<T>::MakeFromOneUnitVector(Cz_W, kZAxis);
    R_CWs_[i] = R_WC.transpose();
    /* Compute the penetration distance from the signed distance field. */
    penetration_distances_[i] = signed_distance_field.EvaluateCartesian(
        i, contact_surface_mesh_W_->element_centroid(i));
  }
}

template <typename T>
const T& DeformableRigidContactSurface<T>::penetration_distance(int e) const {
  DRAKE_DEMAND(0 <= e && e < num_contact_points());
  return penetration_distances_[e];
}

template class DeformableRigidContactSurface<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
