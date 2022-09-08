#include "drake/geometry/proximity/make_nonconvex_mesh.h"

#include <string>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
VolumeMesh<T> MakeNonConvexVolumeMeshFromVtk(const Mesh& mesh_spec) {
  const std::string& vtk_file_name = mesh_spec.filename();
  VolumeMesh<double> read_mesh =
      ReadVtkToVolumeMesh(vtk_file_name, mesh_spec.scale());

  for (int e = 0; e < read_mesh.num_elements(); ++e) {
    if (read_mesh.CalcTetrahedronVolume(e) <= 0.) {
      throw std::runtime_error(fmt::format(
          "MakeNonConvexVolumeMeshFromVtk: "
          "The {}-th tetrahedron (index start at 0) with "
          "vertices {}, {}, {}, {} has non-positive volume, "
          "so you might want to switch two consecutive vertices.",
          e, read_mesh.element(e).vertex(0), read_mesh.element(e).vertex(1),
          read_mesh.element(e).vertex(2), read_mesh.element(e).vertex(3)));
    }
  }

  if constexpr (std::is_same_v<T, double>) {
    return read_mesh;
  } else {
    // Scalar conversion from double to T, which is supposedly AutoDiffXd.
    std::vector<Vector3<T>> mesh_vertices;
    mesh_vertices.reserve(read_mesh.num_vertices());
    for (const Vector3<double>& p_MV : read_mesh.vertices()) {
      mesh_vertices.emplace_back(p_MV);
    }
    std::vector<VolumeElement> mesh_elements = read_mesh.tetrahedra();
    return {std::move(mesh_elements), std::move(mesh_vertices)};
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &MakeNonConvexVolumeMeshFromVtk<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
