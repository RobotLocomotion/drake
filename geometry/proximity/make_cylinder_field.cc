#include "drake/geometry/proximity/make_cylinder_field.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
VolumeMeshFieldLinear<T, T> MakeCylinderPressureField(
    const Cylinder& cylinder, const VolumeMesh<T>* mesh_C,
    const T hydroelastic_modulus) {
  DRAKE_DEMAND(hydroelastic_modulus > T(0));
  const double radius = cylinder.radius();
  const double length = cylinder.length();
  const double min_half_size = std::min(radius, length / 2.0);

  // TODO(DamrongGuoy): Switch to a better implementation in the future. The
  //  current implementation has a number of limitations:
  //  1. For simplicity, we use a scaling of distance to boundary, which is
  //     not differentiable at points equally far from two or more boundary
  //     points of the cylinder.
  //  2. Implicitly we impose the rigid core on the medial axis (or medial
  //     surfaces for short cylinders).  In the future, we will consider a
  //     rigid core as an offset of the cylinder or other shapes.
  //  3. We do not have a mechanism to define a pressure field using barrier
  //     functions.  One possibility is to generate the mesh in offset
  //     layers, and define linear pressure fields in each offset with
  //     different elastic modulus.

  std::vector<T> pressure_values;
  pressure_values.reserve(mesh_C->num_vertices());
  // TODO(DamrongGuoy): The following three const variables (unused_id,
  //  identity, and fcl_cylinder) are needed for applying DistanceToPoint
  //  functor in the for loop. They are awkward to use here and introduce an
  //  unnecessary dependency on FCL. In the future, we should either refactor
  //  the distance-to-boundary calculation or use a different calculation (for
  //  example, offset-based distance).
  const GeometryId unused_id;
  const auto identity = math::RigidTransform<T>::Identity();
  const fcl::Cylinderd fcl_cylinder(radius, length);
  for (const Vector3<T>& vertex : mesh_C->vertices()) {
    // V is a vertex of the cylinder mesh with frame C.
    const Vector3<T>& r_CV = vertex;
    point_distance::DistanceToPoint<T> signed_distance_functor(
        unused_id, identity, r_CV);
    const T signed_distance = signed_distance_functor(fcl_cylinder).distance;
    // Map signed_distance ∈ [-min_half_size, 0] to extent e ∈ [0, 1],
    // -min_half_size ⇝ 1, 0 ⇝ 0.
    const T extent = -signed_distance / T(min_half_size);
    using std::min;
    // Bound the pressure values in [0, E], where E is the elastic modulus.
    pressure_values.push_back(
        min(hydroelastic_modulus * extent, hydroelastic_modulus));
  }

  // Make sure the boundary vertices have zero pressure. Numerical rounding
  // can cause the boundary vertices to be slightly off the boundary surface
  // of the cylinder.
  std::vector<int> boundary_vertices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh_C->tetrahedra()));
  for (int bv : boundary_vertices) {
    pressure_values[bv] = T(0.);
  }

  return VolumeMeshFieldLinear<T, T>(std::move(pressure_values), mesh_C);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &MakeCylinderPressureField<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
