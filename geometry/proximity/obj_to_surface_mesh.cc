#include "drake/geometry/proximity/obj_to_surface_mesh.h"

#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/read_obj.h"

namespace drake {
namespace geometry {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
using Eigen::Vector3d;

namespace internal {

std::optional<TriangleSurfaceMesh<double>> DoReadObjToSurfaceMesh(
    const MeshSource& mesh_source, double scale,
    const DiagnosticPolicy& diagnostic) {
  std::shared_ptr<std::vector<Eigen::Vector3d>> vertices;
  std::shared_ptr<std::vector<int>> face_data;
  int num_tris{};
  std::tie(vertices, face_data, num_tris) =
      ReadObj(mesh_source, scale, /* triangulate = */ true,
              /* vertices_only = */ false, diagnostic);
  if (vertices == nullptr) {
    // ReadObjStream() reported errors to diagnostic; we can simply return no
    // mesh.
    return std::nullopt;
  }

  DRAKE_DEMAND(ssize(*face_data) == num_tris * 4);
  std::vector<SurfaceTriangle> triangles;
  triangles.reserve(num_tris);
  int i = 0;
  for (int f = 0; f < num_tris; ++f) {
    DRAKE_DEMAND((*face_data)[i] == 3);  // Each face has three vertices.
    triangles.emplace_back((*face_data)[i + 1], (*face_data)[i + 2],
                           (*face_data)[i + 3]);
    i += 4;
  }

  return TriangleSurfaceMesh<double>(std::move(triangles),
                                     std::move(*vertices));
}

}  // namespace internal

namespace {

DiagnosticPolicy MakePolicy(std::function<void(std::string_view)> on_warning) {
  DiagnosticPolicy policy;
  if (on_warning != nullptr) {
    policy.SetActionForWarnings([&on_warning](const DiagnosticDetail& detail) {
      on_warning(detail.FormatWarning());
    });
  }
  return policy;
}

}  // namespace

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const std::filesystem::path& filename, const double scale,
    std::function<void(std::string_view)> on_warning) {
  // We will either throw or return a mesh here.
  return *internal::DoReadObjToSurfaceMesh(filename, scale,
                                           MakePolicy(on_warning));
}

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    std::istream* input_stream, const double scale,
    std::function<void(std::string_view)> on_warning,
    std::string_view description) {
  DRAKE_THROW_UNLESS(input_stream != nullptr);
  DRAKE_THROW_UNLESS(input_stream->good());
  std::stringstream content;
  content << input_stream->rdbuf();

  // We will either throw or return a mesh here.
  return *internal::DoReadObjToSurfaceMesh(
      InMemoryMesh(MemoryFile(std::move(content).str(), ".obj",
                              std::string(description))),
      scale, MakePolicy(on_warning));
}

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const MeshSource& mesh_source, double scale,
    std::function<void(std::string_view)> on_warning) {
  // We will either throw or return a mesh here.
  return *internal::DoReadObjToSurfaceMesh(mesh_source, scale,
                                           MakePolicy(on_warning));
}

}  // namespace geometry
}  // namespace drake
