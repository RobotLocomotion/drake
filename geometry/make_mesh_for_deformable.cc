#include "drake/geometry/make_mesh_for_deformable.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

std::unique_ptr<VolumeMesh<double>> MeshBuilderForDeformable::Build(
    const Shape& shape, double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0.0);
  ReifyData data{resolution_hint, nullptr};
  shape.Reify(this, &data);
  return std::move(data.mesh);
}

void MeshBuilderForDeformable::ImplementGeometry(const Sphere& sphere,
                                                 void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  DRAKE_DEMAND(data.resolution_hint > 0);
  // Relying on move construction from r-value return from MakeSphereVolumeMesh.
  data.mesh = std::make_unique<VolumeMesh<double>>(MakeSphereVolumeMesh<double>(
      sphere, data.resolution_hint,
      TessellationStrategy::kDenseInteriorVertices));
}

void MeshBuilderForDeformable::HandleUnsupportedGeometry(
    const std::string& shape_name) {
  throw std::logic_error(
      fmt::format("MeshBuilderForDeformable: We don't yet generate deformable "
                  "meshes from {}.",
                  shape_name));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
