#pragma once

#include <string>

#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

void WriteSurfaceMeshToObj(const std::string& file_name,
                           const SurfaceMesh<double>& mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
