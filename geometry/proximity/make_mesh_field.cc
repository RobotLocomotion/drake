#include "drake/geometry/proximity/make_mesh_field.h"

#include <cmath>
#include <limits>
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

  // TODO(DamrongGuoy): Check whether there could be numerical roundings that
  //  cause a vertex on the boundary to have a non-zero value. Consider
  //  initializing pressure_values to zeros and skip the computation for
  //  boundary vertices.
  std::vector<T> pressure_values;
  T max_value(std::numeric_limits<double>::lowest());
  // First round, it's actually unsigned distance, not pressure values yet.
  for (const Vector3<T>& p_MV : mesh_M->vertices()) {
    Vector3<double> p_MV_d = ExtractDoubleOrThrow(p_MV);
    T pressure(internal::CalcDistanceToSurfaceMesh(p_MV_d, surface_d));
    pressure_values.emplace_back(pressure);
    if (max_value < pressure) {
      max_value = pressure;
    }
  }

  if (max_value <= T(0)) {
    throw std::runtime_error(
        "MakeVolumeMeshPressureField(): "
        "the computed max distance to boundary among "
        "all mesh vertices is non-positive. Perhaps "
        "the mesh lacks interior vertices.");
  }
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
