#include <filesystem>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/proximity/detect_zero_simplex.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"
#include "drake/geometry/proximity/volume_mesh_refiner.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

DEFINE_bool(scene_graph, true, "include/exclude scene graph");
DEFINE_bool(strict, false, "enable strict parsing");

namespace drake {
namespace geometry {
namespace internal {
namespace {

int do_main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "[INPUT-FILE] [OUTPUT-FILE]\n"
      "Run mesh refiner");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 2) {
    drake::log()->error("missing input filename");
    return 1;
  }
  if (argc < 3) {
    drake::log()->error("missing output filename");
    return 1;
  }

  auto mesh = ReadVtkToVolumeMesh(argv[1]);
  auto bad_tets = DetectTetrahedronWithAllBoundaryVertices(mesh);
  auto bad_triangles = DetectInteriorTriangleWithAllBoundaryVertices(mesh);
  auto bad_edges = DetectInteriorEdgeWithAllBoundaryVertices(mesh);

  drake::log()->info(
      "Found {} bad tets, {} bad triangles, and {} bad edges."
      "The mesh has {} tets and {} vertices.",
      bad_tets.size(), bad_triangles.size(), bad_edges.size(),
      mesh.tetrahedra().size(), mesh.vertices().size());
  bool found_bad =
      (!bad_tets.empty() || !bad_triangles.empty() || !bad_edges.empty());
  if (!found_bad) {
    drake::log()->info(
        "No zero simplices found in input mesh '{}';"
        " not writing an output mesh.",
        argv[1]);
    return 0;
  }

  auto refiner = VolumeMeshRefiner(mesh);
  auto refined_mesh = refiner.Refine();

  std::filesystem::path outfile(argv[2]);
  const char* test_dir = ::getenv("TEST_TMPDIR");
  if (test_dir != nullptr) {
    // Redirect the output when run as a bazel test.
    outfile = std::filesystem::path(test_dir) / outfile;
  }

  WriteVolumeMeshToVtk(outfile.string(), refined_mesh,
                       "refined by mesh_refiner_manual_test");
  drake::log()->info(
      "wrote refined mesh to file '{}' with {} tets and {} "
      "vertices.",
      outfile.string(), refined_mesh.tetrahedra().size(),
      refined_mesh.vertices().size());
  return 0;
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::geometry::internal::do_main(argc, argv);
}
