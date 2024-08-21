#include "drake/geometry/in_memory_mesh.h"

#include <utility>

namespace drake {
namespace geometry {

InMemoryMesh::InMemoryMesh() = default;

InMemoryMesh::InMemoryMesh(MemoryFile mesh_file)
    : mesh_file_(std::move(mesh_file)) {}

InMemoryMesh::~InMemoryMesh() = default;

bool InMemoryMesh::empty() const {
  return mesh_file_.contents().empty();
}

}  // namespace geometry
}  // namespace drake
