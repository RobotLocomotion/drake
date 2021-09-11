#include "drake/geometry/proximity/surface_mesh_to_obj.h"

#include <fstream>
#include <iostream>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {
void WriteObj(std::ofstream& out, const SurfaceMesh<double>& mesh) {
  // Example:
  // "v  1.0 -1.0 -1.0\n"
  // "v  1.0 -1.0  1.0\n"
  // "v -1.0 -1.0  1.0\n"
  // "f 1 2 3\n"}
  const int num_vertices = mesh.num_vertices();
  for (SurfaceMesh<double>::VertexIndex v(0); v < num_vertices; ++v) {
    const Vector3<double>& vertex = mesh.vertex(v).r_MV();
    out << fmt::format("v {:12.8f} {:12.8f} {:12.8f}\n", vertex[0], vertex[1],
                       vertex[2]);
  }
  out << std::endl;

  // Example:
  // "f 1 2 3\n"}
  const int num_faces = mesh.num_faces();
  for (SurfaceMesh<double>::ElementIndex f(0); f < num_faces; ++f) {
    const SurfaceFace& face = mesh.element(f);
    // Ours use 0-based indexing. Obj files use 1-based indexing.
    int v1 = face.vertex(0) + 1;
    int v2 = face.vertex(1) + 1;
    int v3 = face.vertex(2) + 1;
    out << fmt::format("f {:6d} {:6d} {:6d}\n", v1, v2, v3);
  }
  out << std::endl;
}

}  // namespace

void WriteSurfaceMeshToObj(const std::string& file_name,
                           const SurfaceMesh<double>& mesh) {
  std::ofstream file(file_name);
  if (file.fail()) {
    throw std::runtime_error(fmt::format("Cannot create file: {}.", file_name));
  }
  WriteObj(file, mesh);
  file.close();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
