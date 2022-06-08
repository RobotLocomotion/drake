#include "drake/geometry/proximity/deformable_rigid_contact_surface.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
DeformableRigidContactSurface<T>::DeformableRigidContactSurface(
    PolygonSurfaceMesh<T>&& contact_surface_mesh_W,
    std::vector<T>&& penetration_distances,
    std::vector<int>&& tetrahedron_indices,
    std::vector<Vector4<T>>&& barycentric_centroids,
    geometry::GeometryId rigid_id, geometry::GeometryId deformable_id)
    : contact_surface_mesh_W_(std::move(contact_surface_mesh_W)),
      penetration_distances_(std::move(penetration_distances)),
      tetrahedron_indices_(std::move(tetrahedron_indices)),
      barycentric_centroids_(std::move(barycentric_centroids)),
      rigid_id_(rigid_id),
      deformable_id_(deformable_id),
      R_CWs_(contact_surface_mesh_W_.num_faces()) {
  DRAKE_DEMAND(R_CWs_.size() == penetration_distances_.size());
  DRAKE_DEMAND(R_CWs_.size() == tetrahedron_indices_.size());
  DRAKE_DEMAND(R_CWs_.size() == barycentric_centroids_.size());
  for (int i = 0; i < contact_surface_mesh_W_.num_faces(); ++i) {
    /* The normal of the face lies in the direction of the contact frame's
     z-axis. */
    const Vector3<T>& Cz_W = contact_surface_mesh_W_.face_normal(i);
    constexpr int kZAxis = 2;
    auto R_WC = math::RotationMatrix<T>::MakeFromOneUnitVector(Cz_W, kZAxis);
    R_CWs_[i] = R_WC.transpose();
  }
}

template <typename T>
std::tuple<PolygonSurfaceMesh<T>, std::vector<T>, std::vector<int>,
           std::vector<Vector4<T>>, std::vector<math::RotationMatrix<T>>>
DeformableRigidContactSurface<T>::release_data() {
  return {std::move(contact_surface_mesh_W_), std::move(penetration_distances_),
          std::move(tetrahedron_indices_), std::move(barycentric_centroids_),
          std::move(R_CWs_)};
}

template struct DeformableRigidContactSurface<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
