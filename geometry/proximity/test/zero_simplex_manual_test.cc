#include <filesystem>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/proximity/detect_zero_simplex.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

DEFINE_bool(expect_fail, false,
            "expect to find zero simplices; reverses the success/fail logic");

namespace drake {
namespace geometry {
namespace internal {
namespace {

int do_main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "[INPUT-VTK-FILE]\n"
      "Run zero simplex checks; summarize zero simplices;"
      " fail if any are found unless --expect_fail is set");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 2) {
    drake::log()->error("missing input filename");
    return 1;
  }

  auto mesh = ReadVtkToVolumeMesh(std::filesystem::path(argv[1]));
  auto bad_tets = DetectTetrahedronWithAllBoundaryVertices(mesh);
  auto bad_triangles = DetectInteriorTriangleWithAllBoundaryVertices(mesh);
  auto bad_edges = DetectInteriorEdgeWithAllBoundaryVertices(mesh);

  drake::log()->info(
      "Found {} bad tetrahedra, {} bad triangles, and {} bad "
      "edges from total {} tetrahedra.",
      bad_tets.size(), bad_triangles.size(), bad_edges.size(),
      mesh.num_elements());
  bool found_bad =
      (!bad_tets.empty() || !bad_triangles.empty() || !bad_edges.empty());
  return found_bad ^ FLAGS_expect_fail;
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::geometry::internal::do_main(argc, argv);
}
