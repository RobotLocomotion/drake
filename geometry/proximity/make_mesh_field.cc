#include "drake/geometry/proximity/make_mesh_field.h"

#include <cmath>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/geometry/proximity/calc_distance_to_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

template <typename T>
TriangleSurfaceMesh<double> ConvertVolumeToSurfaceMeshDouble(
    const VolumeMesh<T>& volume_mesh) {
  TriangleSurfaceMesh<T> surface =
      ConvertVolumeToSurfaceMesh(volume_mesh);
  if constexpr (std::is_same_v<T, double>) {
    return surface;
  } else {
    // Scalar conversion from T (supposedly AutoDiffXd) to double.
    std::vector<Vector3<double>> vertices_double;
    vertices_double.reserve(surface.vertices().size());
    for (const Vector3<T>& p_MV : surface.vertices()) {
      vertices_double.emplace_back(ExtractDoubleOrThrow(p_MV));
    }
    std::vector<SurfaceTriangle> triangles = surface.triangles();
    return {std::move(triangles), std::move(vertices_double)};
  }
}

}  // namespace

template <typename T>
VolumeMeshFieldLinear<T, T> MakeVolumeMeshPressureField(
    const VolumeMesh<T>* mesh_M, const T& hydroelastic_modulus) {
  DRAKE_DEMAND(hydroelastic_modulus > T(0));
  DRAKE_DEMAND(mesh_M != nullptr);
  // The subscript _d is for the scalar type double.
  TriangleSurfaceMesh<double> surface_d =
      ConvertVolumeToSurfaceMeshDouble(*mesh_M);

  std::vector<T> pressure_values;
  // First round, it's actually unsigned distance, not pressure values yet.
  for (const Vector3<T>& p_MV : mesh_M->vertices()) {
    Vector3<double> p_MV_d = ExtractDoubleOrThrow(p_MV);
    pressure_values.emplace_back(
      std::abs(internal::CalcDistanceToSurfaceMesh(p_MV_d, surface_d)));
  }
  const T max_value =
      *std::max_element(pressure_values.begin(), pressure_values.end());
  for (T& p : pressure_values) {
    p = hydroelastic_modulus * p / max_value;
  }

  return {std::move(pressure_values), mesh_M, true};
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &MakeVolumeMeshPressureField<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
