#pragma once

#include <functional>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(DamrongGuoy): Update documentation if we change to a more
//  sophisticated field.

/**
 Generates a pressure field in a box B based on the distance to boundary.
 We require the input parameter `elastic_modulus` E that acts on
 extent e ∈ [0,1] to give pressure p ∈ [0,E]. Internally we generate the
 extent field e:B→[0, 1] based on the distance to boundary with minimum
 value 0.0 on the boundary and maximum value 1.0 in the interior. Then,
 we return the pressure field p on the box, p:B→[0, E], p(Q) = E*e(Q).
 Therefore, the pressure field is zero on the boundary with maximum E
 interior.
 @param box   The box B.
 @param mesh  A tetrahedral mesh of the box B.
 @param elastic_modulus  Scale extent to pressure.
 @return      The pressure field defined on the tetrahedral mesh.
 @pre        `elastic_modulus` is strictly positive.
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeBoxPressureField(
    const Box& box, const VolumeMesh<T>& mesh,
    const T elastic_modulus) {
  DRAKE_DEMAND(elastic_modulus > T(0));
  const Vector3<double> half_size = box.size() / 2.0;
  const double min_half_size = half_size.minCoeff();

  std::vector<T> pressure_values;
  pressure_values.reserve(mesh.num_vertices());
  for (const VolumeVertex<T>& vertex : mesh.vertices()) {
    // V is for a vertex of the mesh of the box B.
    const Vector3<T>& r_BV = vertex.r_MV();
    // N is for the nearest point of V on the boundary of B, and grad_B is
    // the gradient vector of the signed distance function of B at V.
    Vector3<T> r_BN, grad_B;
    bool is_V_on_edge_or_vertex;  // Not used.
    std::tie(r_BN, grad_B, is_V_on_edge_or_vertex) =
        point_distance::DistanceToPoint<T>::ComputeDistanceToBox(half_size,
                                                                 r_BV);
    T signed_distance = grad_B.dot(r_BV - r_BN);
    // Map signed_distance ∈ [-min_half_size, 0] to extent e ∈ [0, 1],
    // -min_half_size ⇝ 1, 0 ⇝ 0.
    T extent = -signed_distance / T(min_half_size);
    pressure_values.push_back(elastic_modulus * extent);
  }

  return VolumeMeshFieldLinear<T, T>("pressure", std::move(pressure_values),
                                     &mesh);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
