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
          std::unique_ptr<std::vector<ContactPolygonData<T>>>>
DeformableContactBuilder<T>::MakeMeshAndField() {
  std::unique_ptr<PolygonSurfaceMesh<T>> polygon_mesh;
  std::unique_ptr<PolygonSurfaceMeshFieldLinear<T, T>> polygon_field;
  std::tie(polygon_mesh, polygon_field) = poly_mesh_builder_.MakeMeshAndField();

  const int num_polygons = tetrahedron_index_of_polygons_.size();
  DRAKE_DEMAND(num_polygons ==  polygon_mesh->num_faces());

  std::vector<ContactPolygonData<T>> polygon_data_B;
  for (int polygon_index = 0; polygon_index < num_polygons; ++polygon_index) {
    polygon_data_B.push_back(
        {polygon_mesh->area(polygon_index),
         polygon_mesh->face_normal(polygon_index),
         polygon_mesh->element_centroid(polygon_index),
         volume_mesh_B_->CalcBarycentric(
             polygon_mesh->element_centroid(polygon_index),
             tetrahedron_index_of_polygons_[polygon_index]),
         tetrahedron_index_of_polygons_[polygon_index]});
  }

  return {std::move(polygon_mesh),
          std::make_unique<std::vector<ContactPolygonData<T>>>(polygon_data_B)};
}

template class DeformableContactBuilder<double>;
}  // namespace internal
}  // namespace geometry
}  // namespace drake
