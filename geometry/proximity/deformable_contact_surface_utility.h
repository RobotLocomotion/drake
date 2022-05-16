#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/deformable_contact_surface.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
class DeformableContactBuilder {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContactBuilder);

  using ScalarType = T;
  using MeshType = PolygonSurfaceMesh<T>;
  using FieldType = std::vector<ContactPolygonData<T>>;

  // Assume the given `volume_mesh_B` is alive during the lifetime of
  // this object.
  explicit DeformableContactBuilder(const VolumeMesh<T>& volume_mesh_B)
      : volume_mesh_B_(&volume_mesh_B) {}

  int AddVertex(const Vector3<T>& p_BV, const T& field_value);

  int AddPolygon(const std::vector<int>& polygon_vertices,
                 const Vector3<T>& nhat_B, const Vector3<T>& grad_e_MN_B,
                 int tetrahedron_index);

  int num_vertices() const;
  int num_faces() const;

  /* Create a mesh and field from the mesh data that has been aggregated by
   this builder. */
  std::pair<std::unique_ptr<MeshType>, std::unique_ptr<FieldType>>
  MakeMeshAndField();

 private:
  const VolumeMesh<T>* volume_mesh_B_;
  PolyMeshBuilder<T> poly_mesh_builder_;
  std::vector<int> tetrahedron_index_of_polygons_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
