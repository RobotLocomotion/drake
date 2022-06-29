#include "drake/geometry/proximity/deformable_rigid_contact_surface.h"

#include <utility>

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
DeformableRigidContactSurface<T>::DeformableRigidContactSurface(
    ContactSurface<T> contact_surface_W, std::vector<int> tetrahedron_indices,
    std::vector<Vector4<T>> barycentric_centroids,
    geometry::GeometryId rigid_id, geometry::GeometryId deformable_id)
    : contact_surface_W_(std::move(contact_surface_W)),
      tetrahedron_indices_(std::move(tetrahedron_indices)),
      barycentric_centroids_(std::move(barycentric_centroids)),
      rigid_id_(rigid_id),
      deformable_id_(deformable_id),
      R_CWs_(contact_surface_W_.num_faces()) {
  for (int i = 0; i < contact_surface_W_.num_faces(); ++i) {
    /* The normal of the face lies in the direction of the contact frame's
     z-axis. */
    const Vector3<T>& Cz_W = contact_surface_W_.face_normal(i);
    constexpr int kZAxis = 2;
    auto R_WC = math::RotationMatrix<T>::MakeFromOneUnitVector(Cz_W, kZAxis);
    R_CWs_[i] = R_WC.transpose();
  }
}

template <typename T>
T DeformableRigidContactSurface<T>::EvaluatePenetrationDistance(int e) const {
  DRAKE_THROW_UNLESS(0 <= e && e < num_contact_points());
  return contact_surface_W_.poly_e_MN().template EvaluateCartesian(
      e, contact_surface_W_.centroid(e));
}

template class DeformableRigidContactSurface<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
