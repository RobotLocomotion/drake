#include "drake/geometry/make_mesh_for_deformable.h"

#include "drake/common/drake_assert.h"
#include "drake/common/overloaded.h"
#include "drake/geometry/proximity/make_mesh_from_vtk.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

std::unique_ptr<VolumeMesh<double>> MakeMeshForDeformable(
    const Shape& shape, double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0.0);
  return shape.Visit(overloaded{
      [](const Mesh& mesh) {
        return std::make_unique<VolumeMesh<double>>(
            MakeVolumeMeshFromVtk<double>(mesh));
      },
      [resolution_hint](const Sphere& sphere) {
        return std::make_unique<VolumeMesh<double>>(
            MakeSphereVolumeMesh<double>(
                sphere, resolution_hint,
                TessellationStrategy::kDenseInteriorVertices));
      },
      // TODO(xuchenhan-tri): As other shapes get supported, include their
      //  specific overrides here.
      [](const auto& unsupported) -> std::unique_ptr<VolumeMesh<double>> {
        throw std::logic_error(fmt::format(
            "MakeMeshForDeformable: We don't yet generate deformable meshes "
            "for {}.",
            unsupported));
      }});
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
