#include "drake/geometry/proximity/deformable_rigid_contact_surface.h"

#include <utility>

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
DeformableRigidContactSurface<T>::DeformableRigidContactSurface(
    ContactSurface<T> contact_surface, std::vector<int> tetrahedron_indices,
    std::vector<Vector4<T>> barycentric_centroids,
    geometry::GeometryId rigid_id, geometry::GeometryId deformable_id)
    : contact_surface_(std::move(contact_surface)),
      tetrahedron_indices_(std::move(tetrahedron_indices)),
      barycentric_centroids_(std::move(barycentric_centroids)),
      rigid_id_(rigid_id),
      deformable_id_(deformable_id),
      R_CWs_(contact_surface_.num_faces()) {
  for (int ic = 0; ic < contact_surface_.num_faces(); ++ic) {
    const Vector3<T>& nhat_W = contact_surface_.face_normal(ic);
    constexpr int axis = 2;
    auto R_WC = math::RotationMatrix<T>::MakeFromOneUnitVector(nhat_W, axis);
    R_CWs_[ic] = R_WC.transpose();
  }
}

template <typename T>
T DeformableRigidContactSurface<T>::EvaluatePenetrationDistance(int e) const {
  DRAKE_THROW_UNLESS(0 <= e && e < num_contact_points());
  return contact_surface_.poly_e_MN().template EvaluateCartesian(
      e, contact_surface_.centroid(e));
}

template class DeformableRigidContactSurface<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
