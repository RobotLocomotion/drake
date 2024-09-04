#include "drake/geometry/in_memory_mesh.h"

#include <fmt/format.h>
#include <fmt/ranges.h>

namespace drake {
namespace geometry {

std::string InMemoryMesh::to_string() const {
  return fmt::format(
      "InMemoryMesh(mesh_file={}{})", mesh_file,
      supporting_files.empty()
          ? std::string{}
          : fmt::format(", supporting_files={}", supporting_files));
}
}  // namespace geometry
}  // namespace drake
