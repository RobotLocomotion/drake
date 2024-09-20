#include "drake/geometry/in_memory_mesh.h"

#include <fmt/format.h>

namespace drake {
namespace geometry {

std::string InMemoryMesh::to_string() const {
  // Note: This would not be appropriate to use in the python __repr__
  // implementation because MemoryFile doesn't properly escape the appropriate
  // characters in the strings.
  std::string supporting_str;
  if (supporting_files.size() > 0) {
    supporting_str = ", supporting_files={";
    bool first = true;
    for (const auto& [name, source] : supporting_files) {
        if (!first) {
          supporting_str += ", ";
        }
        supporting_str +=
            fmt::format("{{'{}', {}}}", name, drake::to_string(source));
        first = false;
    }
    supporting_str += "}";
  }
  return fmt::format("InMemoryMesh(mesh_file={}{})", mesh_file.to_string(),
                     supporting_str);
}
}  // namespace geometry
}  // namespace drake
