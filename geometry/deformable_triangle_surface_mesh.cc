#include "drake/geometry/deformable_triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// Note: MeshDeformer does not have copy/move semantics. So,
// we can't default those semantics on DeformableTriangleSurfaceMesh. The
// deformer_ is configured to *always* point to the instance's *members* mesh_.
// That never changes during the entire lifetime of a
// DeformableTriangleSurfaceMesh instance. So, the assignment operators only
// have to worry about setting the member mesh to the assigned data; we don't
// have to (and can't) make any modifications to the deformer.
DeformableTriangleSurfaceMesh::DeformableTriangleSurfaceMesh(
    const DeformableTriangleSurfaceMesh& other)
    : DeformableTriangleSurfaceMesh(other.mesh_) {}

DeformableTriangleSurfaceMesh& DeformableTriangleSurfaceMesh::operator=(
    const DeformableTriangleSurfaceMesh& other) {
  if (this == &other) return *this;
  mesh_ = other.mesh();
  return *this;
}

DeformableTriangleSurfaceMesh::DeformableTriangleSurfaceMesh(
    DeformableTriangleSurfaceMesh&& other)
    : DeformableTriangleSurfaceMesh(std::move(other.mesh_)) {}

DeformableTriangleSurfaceMesh& DeformableTriangleSurfaceMesh::operator=(
    DeformableTriangleSurfaceMesh&& other) {
  if (this == &other) return *this;
  mesh_ = std::move(other.mesh_);
  return *this;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
