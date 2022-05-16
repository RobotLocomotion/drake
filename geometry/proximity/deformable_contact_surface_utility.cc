#include "drake/geometry/proximity/deformable_contact_surface_utility.h"

#include "drake/geometry/proximity/contact_surface_utility.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
int DeformableContactBuilder<T>::num_vertices() const {
  return poly_mesh_builder_.num_vertices();
}

template <typename T>
int DeformableContactBuilder<T>::num_faces() const {
  return poly_mesh_builder_.num_faces();
}

template <typename T>
int DeformableContactBuilder<T>::AddVertex(const Vector3<T>& p_BV,
                                           const T& field_value) {
  return poly_mesh_builder_.AddVertex(p_BV, field_value);
}

template <typename T>
int DeformableContactBuilder<T>::AddPolygon(
    const std::vector<int>& polygon_vertices, const Vector3<T>& nhat_B,
    const Vector3<T>& grad_e_MN_B, const int tetrahedron_index) {
  poly_mesh_builder_.AddPolygon(polygon_vertices, nhat_B, grad_e_MN_B);
  tetrahedron_index_of_polygons_.push_back(tetrahedron_index);
  return 1;
}

template <typename T>
std::pair<std::unique_ptr<PolygonSurfaceMesh<T>>,
          std::unique_ptr<PolygonSurfaceMeshFieldLinear<T, T>>>
DeformableContactBuilder<T>::MakeMeshAndField() {
  return poly_mesh_builder_.MakeMeshAndField();
}

template class DeformableContactBuilder<double>;
}  // namespace internal
}  // namespace geometry
}  // namespace drake
