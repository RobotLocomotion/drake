#include "drake/geometry/proximity/obj_to_surface_mesh.h"

#include <filesystem>
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
    const MeshSource& source, double scale,
    const DiagnosticPolicy& diagnostic) {
  std::shared_ptr<std::vector<Eigen::Vector3d>> vertices;
  std::shared_ptr<std::vector<int>> face_data;
  int num_tris{};
  try {
    std::tie(vertices, face_data, num_tris) =
        ReadObj(source, scale, /* triangulate = */ true);
  } catch (const std::exception& e) {
    diagnostic.Error(e.what());
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
    const std::string& filename, const double scale,
    std::function<void(std::string_view)> on_warning) {
  // We will either throw or return a mesh here.
  return *internal::DoReadObjToSurfaceMesh(filename, scale,
                                           MakePolicy(on_warning));
}

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    std::istream* input_stream, const double scale,
    std::function<void(std::string_view)> on_warning) {
  DRAKE_THROW_UNLESS(input_stream != nullptr);
  DRAKE_THROW_UNLESS(input_stream->good());
  std::stringstream content;
  content << input_stream->rdbuf();

  InMemoryMesh mesh{.mesh_file = MemoryFile(std::move(content).str(), ".obj",
                                            "in_memory.obj")};

  // We will either throw or return a mesh here.
  return *internal::DoReadObjToSurfaceMesh(mesh, scale, MakePolicy(on_warning));
}

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const MeshSource& source, double scale,
    std::function<void(std::string_view)> on_warning) {
  // We will either throw or return a mesh here.
  return *internal::DoReadObjToSurfaceMesh(source, scale,
                                           MakePolicy(on_warning));
}

}  // namespace geometry
}  // namespace drake
