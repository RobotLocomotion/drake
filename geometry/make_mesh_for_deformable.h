#pragma once

#include <memory>
#include <string>

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* See Build() function. */
class MeshBuilderForDeformable : public ShapeReifier {
 public:
  MeshBuilderForDeformable() = default;

  /* Returns a volume mesh suitable for deformable body simulation that
   discretizes the shape with the resolution provided at construction.
   @pre resolution_hint > 0.
   @throws std::exception if `shape` doesn't support a mesh discretization. */
  std::unique_ptr<VolumeMesh<double>> Build(const Shape& shape,
                                            double resolution_hint);

 private:
  /* Data to be used during reification. It is passed as the `user_data`
   parameter in the ImplementGeometry API. */
  struct ReifyData {
    double resolution_hint{};
    std::unique_ptr<VolumeMesh<double>> mesh;
  };

  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Sphere& sphere, void* user_data) override;
  void ImplementGeometry(const Mesh& mesh_spec, void* user_data) override;
  // TODO(xuchenhan-tri): As other shapes get supported, include their specific
  //  overrides here.

  void ThrowUnsupportedGeometry(const std::string& shape_name) override;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
