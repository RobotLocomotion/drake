#include "drake/geometry/proximity/make_obb_mesh_impl.h"

#include <set>
#include <string>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

Obb MakeObb(const MeshSource& mesh_source, const Vector3d& scale) {
  if (mesh_source.extension() == ".obj") {
    // For OBJ files, create a TriangleSurfaceMesh and use ObbMaker.
    TriangleSurfaceMesh<double> surface_mesh =
        ReadObjToTriangleSurfaceMesh(mesh_source, scale);

    // Create a set containing all vertex indices
    std::set<int> all_vertices;
    for (int i = 0; i < surface_mesh.num_vertices(); ++i) {
      all_vertices.insert(i);
    }

    // Use ObbMaker to compute the OBB
    ObbMaker<TriangleSurfaceMesh<double>> obb_maker(surface_mesh, all_vertices);
    return obb_maker.Compute();
  } else if (mesh_source.extension() == ".vtk") {
    // For VTK files, create a VolumeMesh and use ObbMaker.
    const VolumeMesh<double> volume_mesh =
        ReadVtkToVolumeMesh(mesh_source, scale);

    // Create a set containing all vertex indices
    std::set<int> all_vertices;
    for (int i = 0; i < volume_mesh.num_vertices(); ++i) {
      all_vertices.insert(i);
    }

    // Use ObbMaker to compute the OBB
    ObbMaker<VolumeMesh<double>> obb_maker(volume_mesh, all_vertices);
    return obb_maker.Compute();
  } else if (mesh_source.extension() == ".gltf") {
    // For glTF files, we don't currently have a direct mesh creation method
    // that works with ObbMaker, so we throw an exception.
    throw std::runtime_error(
        fmt::format("MakeObb does not currently support glTF files. "
                    "Unsupported extension '{}' for geometry data: {}.",
                    mesh_source.extension(), mesh_source.description()));
  } else {
    throw std::runtime_error(
        fmt::format("MakeObb only applies to .obj and .vtk meshes; "
                    "unsupported extension '{}' for geometry data: {}.",
                    mesh_source.extension(), mesh_source.description()));
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
