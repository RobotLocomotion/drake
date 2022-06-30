#include "drake/geometry/proximity/deformable_rigid_contact_surface.h"

#include <utility>

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
DeformableRigidContactSurface<T>::DeformableRigidContactSurface(
    const ContactSurface<T>& contact_surface,
    std::vector<int> tetrahedron_indices,
    std::vector<Vector4<T>> barycentric_centroids,
    geometry::GeometryId rigid_id, geometry::GeometryId deformable_id)
    : num_contact_points_(contact_surface.num_faces()),
      tetrahedron_indices_(std::move(tetrahedron_indices)),
      barycentric_centroids_(std::move(barycentric_centroids)),
      penetration_distances_(num_contact_points_),
      contact_surface_mesh_(contact_surface.poly_mesh_W()),
      rigid_id_(rigid_id),
      deformable_id_(deformable_id),
      R_CWs_(contact_surface.num_faces()) {
  for (int i = 0; i < contact_surface.num_faces(); ++i) {
    /* The normal of the face lies in the direction of the contact frame's
     z-axis. */
    const Vector3<T>& Cz_W = contact_surface.face_normal(i);
    constexpr int kZAxis = 2;
    auto R_WC = math::RotationMatrix<T>::MakeFromOneUnitVector(Cz_W, kZAxis);
    R_CWs_[i] = R_WC.transpose();
    /* Compute the penetration distance from the distance field in
     * `contact_surface`. */
    penetration_distances_[i] =
        contact_surface.poly_e_MN().template EvaluateCartesian(
            i, contact_surface.centroid(i));
  }
}

template <typename T>
T DeformableRigidContactSurface<T>::EvaluatePenetrationDistance(int e) const {
  DRAKE_THROW_UNLESS(0 <= e && e < num_contact_points());
  return penetration_distances_[e];
}

template class DeformableRigidContactSurface<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
