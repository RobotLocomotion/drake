#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/proximity/detect_zero_simplex.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"
#include "drake/geometry/proximity/volume_mesh_refiner.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

namespace drake {
namespace geometry {
namespace {

int do_main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "[INPUT-FILE] [OUTPUT-FILE]\n"
      "\n"
      "Refine a tetrahedral mesh of a compliant-hydroelastic geometry in\n"
      "the input VTK file to eliminate problematic tetrahedra,\n"
      "internal triangles, and internal edges with all boundary vertices.\n"
      "They define zero pressure in the interior volume creating a void\n"
      "space of no hydroelastic contact.\n"
      "\n"
      "If the input has no problems, no output file is written.\n"
      "\n"
      "After building this tool from source; for example:\n"
      " drake $ bazel build //geometry/proximity:refine_mesh\n"
      "Run it in your data directory; for example:\n"
      " data $ /path/to/drake/bazel-bin/geometry/proximity/refine_mesh "
      "input.vtk output.vtk");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 2) {
    drake::log()->error("missing input filename");
    return 1;
  }
  if (argc < 3) {
    drake::log()->error("missing output filename");
    return 1;
  }

  // Make cwd be what the user expected, not the runfiles tree.
  if (const char* path = std::getenv("BUILD_WORKING_DIRECTORY")) {
    const int error = ::chdir(path);
    if (error != 0) {
      log()->warn("Could not chdir to '{}'", path);
    }
  }

  const VolumeMesh<double> input_mesh =
      internal::ReadVtkToVolumeMesh(std::filesystem::path(argv[1]));

  // Log statistics.
  // TODO(nepfaff): Computing the statistics is redundant and only
  // here for printing the diagnostics message. Change the call to
  // RefineVolumeMesh() so that it returns this diagnostic information.
  VolumeMesh<double> mesh =
      internal::ReadVtkToVolumeMesh(std::filesystem::path(argv[1]));
  std::vector<int> bad_tets =
      internal::DetectTetrahedronWithAllBoundaryVertices(mesh);
  std::vector<internal::SortedTriplet<int>> bad_triangles =
      internal::DetectInteriorTriangleWithAllBoundaryVertices(mesh);
  std::vector<SortedPair<int>> bad_edges =
      internal::DetectInteriorEdgeWithAllBoundaryVertices(mesh);
  drake::log()->info(
      "Found {} bad tets, {} bad triangles, and {} bad edges."
      "The mesh has {} tets and {} vertices.",
      bad_tets.size(), bad_triangles.size(), bad_edges.size(),
      mesh.tetrahedra().size(), mesh.vertices().size());

  // Refine the mesh.
  const VolumeMesh<double> refined_mesh = RefineVolumeMesh(input_mesh);

  // If no refinement was needed, exit early.
  if (refined_mesh.Equal(input_mesh)) {
    drake::log()->info(
        "No problems found in input mesh '{}';"
        " not writing an output mesh.",
        argv[1]);
    return 0;
  }

  std::filesystem::path outfile(argv[2]);
  internal::WriteVolumeMeshToVtk(outfile.string(), refined_mesh,
                                 "refined by //geometry/proximity:refine_mesh");
  drake::log()->info(
      "wrote refined mesh to file '{}' with {} tets and {} "
      "vertices.",
      outfile.string(), refined_mesh.tetrahedra().size(),
      refined_mesh.vertices().size());
  return 0;
}

}  // namespace
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::geometry::do_main(argc, argv);
}
