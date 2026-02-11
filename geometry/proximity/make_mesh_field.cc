#include "drake/geometry/proximity/make_mesh_field.h"

#include <algorithm>
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
    const VolumeMesh<T>& volume_mesh, std::vector<int>* boundary_vertices) {
  TriangleSurfaceMesh<T> surface =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(volume_mesh,
                                                     boundary_vertices);
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
    const VolumeMesh<T>* mesh_M, const T& hydroelastic_modulus, double margin) {
  DRAKE_DEMAND(hydroelastic_modulus > T(0));
  DRAKE_DEMAND(mesh_M != nullptr);
  using std::max;

  std::vector<int> boundary_vertices;
  // The subscript _d is for the scalar type double.
  TriangleSurfaceMesh<double> surface_d =
      ConvertVolumeToSurfaceMeshDouble(*mesh_M, &boundary_vertices);

  // TODO(DamrongGuoy): Check whether there could be numerical roundings that
  //  cause a vertex on the boundary to have a non-zero value. Consider
  //  initializing pressure_values to zeros and skip the computation for
  //  boundary vertices.
  std::vector<T> values;
  T max_value(std::numeric_limits<double>::lowest());
  // First round, it's actually unsigned distance, not pressure values yet.
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh(surface_d);
  auto boundary_iter = boundary_vertices.begin();
  for (int v = 0; v < ssize(mesh_M->vertices()); ++v) {
    if (boundary_iter != boundary_vertices.end() && *boundary_iter == v) {
      ++boundary_iter;
      values.push_back(0);
      continue;
    }
    const Vector3<T>& p_MV = mesh_M->vertex(v);
    const Vector3<double> p_MV_d = ExtractDoubleOrThrow(p_MV);
    // N.B. For small margin values, we can approximate the distance to the
    // inflated surface as the distance to the original surface plus the margin.
    // This correction only applies to interior vertices.
    const T distance =
        internal::CalcDistanceToSurfaceMesh(p_MV_d, surface_d, bvh) + margin;
    values.push_back(distance);
    max_value = max(distance, max_value);
  }

  if (max_value <= T(0)) {
    throw std::runtime_error(
        "MakeVolumeMeshPressureField(): "
        "the computed max distance to boundary among "
        "all mesh vertices is non-positive. Perhaps "
        "the mesh lacks interior vertices.");
  }

  DRAKE_DEMAND(max_value > margin);

  for (T& p : values) {
    p = hydroelastic_modulus * (p - margin) / (max_value - margin);
  }

  return {std::move(values), mesh_M, MeshGradientMode::kOkOrThrow};
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&MakeVolumeMeshPressureField<T>));

}  // namespace internal
}  // namespace geometry
}  // namespace drake
