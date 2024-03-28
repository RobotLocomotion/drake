#include <filesystem>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/proximity/detect_zero_simplex.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"
#include "drake/geometry/proximity/volume_mesh_refiner.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;

struct TetrahedralVolume {
  double volume{std::numeric_limits<double>::infinity()};
  int tet_index{-1};
  Vector3d centroid{0, 0, 0};
};

TetrahedralVolume CalcMinVolumeTetrahedron(const VolumeMesh<double>& mesh) {
  TetrahedralVolume min_tet;
  for (int tet = 0; tet < mesh.num_elements(); ++tet) {
    const double tet_volume = mesh.CalcTetrahedronVolume(tet);
    if (tet_volume < min_tet.volume) {
      min_tet.volume = tet_volume;
      min_tet.tet_index = tet;
    }
  }
  min_tet.centroid =
      (mesh.vertex(mesh.tetrahedra()[min_tet.tet_index].vertex(0)) +
       mesh.vertex(mesh.tetrahedra()[min_tet.tet_index].vertex(1)) +
       mesh.vertex(mesh.tetrahedra()[min_tet.tet_index].vertex(2)) +
       mesh.vertex(mesh.tetrahedra()[min_tet.tet_index].vertex(3))) /
      4.0;

  return min_tet;
}

void ReportMinVolumeTetrahedron(const VolumeMesh<double>& mesh) {
  TetrahedralVolume min_tet = CalcMinVolumeTetrahedron(mesh);
  if (min_tet.volume < 1e-14) {
    drake::log()->warn(
        "The {}-th tetrahedron (0-based index) has very low minimum "
        "volume {} m^3 less than 1e-14 m^3. Its centroid is at ({}, {}, {}).",
        min_tet.tet_index, min_tet.volume, min_tet.centroid.x(),
        min_tet.centroid.y(), min_tet.centroid.z());
  }
}

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

  VolumeMesh<double> mesh = internal::ReadVtkToVolumeMesh(argv[1]);
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
  ReportMinVolumeTetrahedron(mesh);
  bool found_bad =
      (!bad_tets.empty() || !bad_triangles.empty() || !bad_edges.empty());
  if (!found_bad) {
    drake::log()->info(
        "No problems found in input mesh '{}';"
        " not writing an output mesh.",
        argv[1]);
    return 0;
  }

  VolumeMesh<double> refined_mesh = internal::VolumeMeshRefiner(mesh).Refine();

  std::filesystem::path outfile(argv[2]);
  internal::WriteVolumeMeshToVtk(outfile.string(), refined_mesh,
                                 "refined by //geometry/proximity:refine_mesh");
  drake::log()->info(
      "wrote refined mesh to file '{}' with {} tets and {} "
      "vertices.",
      outfile.string(), refined_mesh.tetrahedra().size(),
      refined_mesh.vertices().size());
  ReportMinVolumeTetrahedron(refined_mesh);

  return 0;
}

}  // namespace
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::geometry::do_main(argc, argv);
}
