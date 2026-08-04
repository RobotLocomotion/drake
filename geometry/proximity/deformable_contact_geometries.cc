#include "drake/geometry/proximity/deformable_contact_geometries.h"

#include <algorithm>

#include "drake/geometry/proximity/calc_distance_to_surface_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {
namespace {

using std::make_unique;
using std::vector;

/* Returns an approximation of the signed distance field inside the given
 `mesh`. The distance field samples on the mesh vertices are exact (to the
 accuracy of the distance query algorithm). All values in the field are
 non-positive.

 @warning The resultant field aliases the given `mesh` and the mesh must be
 kept alive at least as long as the returned field.
 @pre mesh != nullptr. */
std::unique_ptr<VolumeMeshFieldLinear<double, double>>
ApproximateSignedDistanceField(const VolumeMesh<double>* mesh) {
  DRAKE_DEMAND(mesh != nullptr);
  const int num_vertices = mesh->num_vertices();
  vector<double> signed_distance;
  signed_distance.reserve(num_vertices);
  std::vector<int> boundary_vertices;
  const TriangleSurfaceMesh<double> surface_mesh =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(*mesh, &boundary_vertices);
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_of_surface(surface_mesh);
  auto boundary_iter = boundary_vertices.begin();
  for (int v = 0; v < num_vertices; ++v) {
    if (boundary_iter != boundary_vertices.end() && *boundary_iter == v) {
      ++boundary_iter;
      signed_distance.push_back(0);
      continue;
    }
    signed_distance.emplace_back(-CalcDistanceToSurfaceMesh(
        mesh->vertex(v), surface_mesh, bvh_of_surface));
  }
  return make_unique<VolumeMeshFieldLinear<double, double>>(
      std::move(signed_distance), mesh);
}

}  // namespace

DeformableGeometry::DeformableGeometry(const DeformableGeometry& other) {
  *this = other;
}

DeformableGeometry& DeformableGeometry::operator=(
    const DeformableGeometry& other) {
  if (this == &other) return *this;

  deformable_volume_ = std::make_unique<DeformableVolumeMeshWithBvh<double>>(
      *other.deformable_volume_);
  deformable_surface_ = std::make_unique<DeformableSurfaceMeshWithBvh<double>>(
      *other.deformable_surface_);
  surface_index_to_volume_index_ = other.surface_index_to_volume_index_;
  surface_tri_to_volume_tet_ = other.surface_tri_to_volume_tet_;
  // We can't simply copy the field; the copy must contain a pointer to
  // the new mesh. So, we use CloneAndSetMesh() instead.
  signed_distance_field_ = other.signed_distance_field_->CloneAndSetMesh(
      &deformable_volume_->mesh());
  return *this;
}

DeformableGeometry::DeformableGeometry(
    VolumeMesh<double> volume_mesh, TriangleSurfaceMesh<double> surface_mesh,
    std::vector<int> surface_index_to_volume_index,
    std::vector<int> surface_tri_to_volume_tet)
    : deformable_volume_(std::make_unique<DeformableVolumeMeshWithBvh<double>>(
          std::move(volume_mesh))),
      deformable_surface_(
          std::make_unique<DeformableSurfaceMeshWithBvh<double>>(
              std::move(surface_mesh))),
      surface_index_to_volume_index_(std::move(surface_index_to_volume_index)),
      surface_tri_to_volume_tet_(std::move(surface_tri_to_volume_tet)),
      signed_distance_field_(
          ApproximateSignedDistanceField(&deformable_volume_->mesh())) {}

const VolumeMeshFieldLinear<double, double>&
DeformableGeometry::CalcSignedDistanceField() const {
  std::vector<double> values = signed_distance_field_->values();
  *signed_distance_field_ = VolumeMeshFieldLinear<double, double>(
      std::move(values), &deformable_volume_->mesh());
  return *signed_distance_field_;
}

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
