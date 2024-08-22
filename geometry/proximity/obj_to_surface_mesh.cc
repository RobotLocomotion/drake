#include "drake/geometry/proximity/obj_to_surface_mesh.h"

#include <fstream>
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
    std::istream* input_stream, const double scale,
    const drake::internal::DiagnosticPolicy& diagnostic,
    std::string_view description) {
  std::shared_ptr<std::vector<Eigen::Vector3d>> vertices;
  std::shared_ptr<std::vector<int>> face_data;
  int num_tris{};
  try {
    std::tie(vertices, face_data, num_tris) = ReadObjStream(
        input_stream, scale, /* triangulate = */ true, description);
  } catch (const std::exception& e) {
    // We're using the diagnostic policy to report errors. So, any errors that
    // ReadObjStream() chose to communicate via exception, we're capturing here
    // to place in the diagnostic error, deferring to the diagnostic policy on
    // how errors get handled. By returning an undefined surface mesh we allow
    // the caller to proceed in a sane manner, even if the policy doesn't throw
    // errors.
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

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const std::string& filename, const double scale,
    std::function<void(std::string_view)> on_warning) {
  std::ifstream input_stream(filename);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Cannot open file '" + filename + "'");
  }

  DiagnosticPolicy policy;
  if (on_warning != nullptr) {
    policy.SetActionForWarnings([&on_warning](const DiagnosticDetail& detail) {
      on_warning(detail.FormatWarning());
    });
  }

  // We will either throw or return a mesh here.
  return *internal::DoReadObjToSurfaceMesh(&input_stream, scale, policy,
                                           filename);
}

TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    std::istream* input_stream, const double scale,
    std::function<void(std::string_view)> on_warning,
    std::string_view description) {
  DRAKE_THROW_UNLESS(input_stream != nullptr);

  DiagnosticPolicy policy;
  if (on_warning != nullptr) {
    policy.SetActionForWarnings([&on_warning](const DiagnosticDetail& detail) {
      on_warning(detail.FormatWarning());
    });
  }

  // We will either throw or return a mesh here.
  return *internal::DoReadObjToSurfaceMesh(input_stream, scale, policy,
                                           description);
}

}  // namespace geometry
}  // namespace drake
