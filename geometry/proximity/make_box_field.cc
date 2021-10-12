#include "drake/geometry/proximity/make_box_field.h"

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
VolumeMeshFieldLinear<T, T> MakeBoxPressureField(
    const Box& box, const VolumeMesh<T>* mesh_B,
    const T hydroelastic_modulus) {
  DRAKE_DEMAND(hydroelastic_modulus > T(0));
  const Vector3<double> half_size = box.size() / 2.0;
  const double min_half_size = half_size.minCoeff();

  // TODO(DamrongGuoy): Switch to a better implementation in the future. The
  //  current implementation has a number of limitations:
  //  1. For simplicity, we use a scaling of distance to boundary, which is
  //     not differentiable at points equally far from two or more faces of
  //     the box.
  //  2. Implicitly we impose the rigid core on the medial axis. In the
  //     future, we will consider a rigid core as an offset of the box or
  //     other shapes.
  //  3. We do not have a mechanism to define a pressure field using barrier
  //     functions.  One possibility is to generate the mesh in offset
  //     layers, and define linear pressure fields in each offset with
  //     different elastic modulus.

  std::vector<T> pressure_values;
  pressure_values.reserve(mesh_B->num_vertices());
  for (const Vector3<T>& r_BV : mesh_B->vertices()) {
    // N is for the nearest point of V on the boundary of the box,
    // and grad_B is the gradient vector of the signed distance function
    // of the box at V, expressed in frame B.
    const auto [r_BN, grad_B, is_V_on_edge_or_vertex] =
        point_distance::DistanceToPoint<T>::ComputeDistanceToBox(half_size,
                                                                 r_BV);
    unused(is_V_on_edge_or_vertex);
    T signed_distance = grad_B.dot(r_BV - r_BN);
    // Map signed_distance ∈ [-min_half_size, 0] to extent e ∈ [0, 1],
    // -min_half_size ⇝ 1, 0 ⇝ 0.
    T extent = -signed_distance / T(min_half_size);
    pressure_values.push_back(hydroelastic_modulus * extent);
  }

  return VolumeMeshFieldLinear<T, T>(std::move(pressure_values), mesh_B);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &MakeBoxPressureField<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
