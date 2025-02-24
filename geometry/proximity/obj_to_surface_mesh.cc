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
    const MeshSource& mesh_source, const Vector3d& scale,
    const DiagnosticPolicy& diagnostic) {
  std::shared_ptr<std::vector<Eigen::Vector3d>> vertices;
  std::shared_ptr<std::vector<int>> face_data;
  int num_tris{};
  // Note: we rely on ReadObj to do the "right" thing in case the scale factor
  // would cause the mesh to get flipped inside out; we assume the mesh data
  // returned has winding consistent with its inside and outside.
  std::tie(vertices, face_data, num_tris) =
      ReadObj(mesh_source, scale, /* triangulate = */ true,
              /* vertices_only = */ false, diagnostic);
  if (vertices == nullptr) {
    // ReadObj() only returns nullptr when there's an error. When nullptr
    // is returned, an error is reported to the diagnostic policy which we can
    // rely on to throw. Returning nullopt satisfies the compiler, but will
    // never be reached outside of unit tests.
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

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const std::filesystem::path& filename, const Eigen::Vector3d& scale3,
    std::function<void(std::string_view)> on_warning) {
  return ReadObjToTriangleSurfaceMesh(MeshSource(filename), scale3, on_warning);
}

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const std::filesystem::path& filename, const double scale,
    std::function<void(std::string_view)> on_warning) {
  return ReadObjToTriangleSurfaceMesh(
      filename, Eigen::Vector3d::Constant(scale), on_warning);
}

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const MeshSource& mesh_source, const Eigen::Vector3d& scale3,
    std::function<void(std::string_view)> on_warning) {
  DiagnosticPolicy policy;
  if (on_warning != nullptr) {
    policy.SetActionForWarnings([&on_warning](const DiagnosticDetail& detail) {
      on_warning(detail.FormatWarning());
    });
  }
  // We will either throw or return a mesh here (courtesy of ReadObj).
  return *internal::DoReadObjToSurfaceMesh(mesh_source, scale3, policy);
}

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const MeshSource& mesh_source, double scale,
    std::function<void(std::string_view)> on_warning) {
  return ReadObjToTriangleSurfaceMesh(
      mesh_source, Eigen::Vector3d::Constant(scale), on_warning);
}

}  // namespace geometry
}  // namespace drake
