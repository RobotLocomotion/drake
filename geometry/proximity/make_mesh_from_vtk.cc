#include "drake/geometry/proximity/make_mesh_from_vtk.h"

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
VolumeMesh<T> MakeVolumeMeshFromVtk(const Mesh& mesh) {
  if (mesh.extension() != ".vtk") {
    throw std::runtime_error(fmt::format(
        "MakeVolumeMeshFromVtk() called on a Mesh specification with the wrong "
        "extension type. Requires '.vtk', got '{}' for mesh data {}.",
        mesh.extension(), mesh.source().description()));
  }

  const Eigen::Vector3d scale = mesh.scale3();

  VolumeMesh<double> read_mesh = ReadVtkToVolumeMesh(mesh.source(), scale);

  for (int e = 0; e < read_mesh.num_elements(); ++e) {
    if (read_mesh.CalcTetrahedronVolume(e) <= 0.) {
      throw std::runtime_error(fmt::format(
          "MakeVolumeMeshFromVtk('{}') with scale [{}]: "
          "The {}-th tetrahedron (index start at 0) with "
          "vertices {}, {}, {}, {} has non-positive volume, "
          "so you might want to switch two consecutive vertices.",
          mesh.source().description(), fmt_eigen(scale.transpose()), e,
          read_mesh.element(e).vertex(0), read_mesh.element(e).vertex(1),
          read_mesh.element(e).vertex(2), read_mesh.element(e).vertex(3)));
    }
  }

  if constexpr (std::is_same_v<T, double>) {
    return read_mesh;
  } else {
    static_assert(std::is_same_v<T, AutoDiffXd>);
    std::vector<Vector3<T>> mesh_vertices;
    mesh_vertices.reserve(read_mesh.num_vertices());
    for (const Vector3<double>& p_MV : read_mesh.vertices()) {
      mesh_vertices.emplace_back(p_MV);
    }
    std::vector<VolumeElement> mesh_elements = read_mesh.tetrahedra();
    return {std::move(mesh_elements), std::move(mesh_vertices)};
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&MakeVolumeMeshFromVtk<T>));

}  // namespace internal
}  // namespace geometry
}  // namespace drake
