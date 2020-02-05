#pragma once

#include <functional>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/**
 Generates a linear approximation of a pressure field inside the given box as
 represented by the given volume mesh. The pressure at a point is defined
 as E * e(x) where e ∈ [0,1] is the extent -- a measure of penetration into
 the volume, and E is the given `elastic_modulus`. The pressure is zero on the
 boundary with maximum E in the interior.
 @param box              The box with its canonical frame B.
 @param mesh_B           A pointer to a tetrahedral mesh of the box. It is
                         aliased in the returned pressure field and must remain
                         alive as long as the field. The position vectors of
                         mesh vertices are expressed in the box's frame B.
 @param elastic_modulus  Scale extent to pressure.
 @return                 The pressure field defined on the tetrahedral mesh.
 @pre                    `elastic_modulus` is strictly positive.
                         `mesh_B` represents the box well (the space enclosed
                         by the mesh should be exactly the same space as the
                         box specification). `mesh_B` has enough resolution
                         to approximate the pressure field.
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeBoxPressureField(
    const Box& box, const VolumeMesh<T>* mesh_B,
    const T elastic_modulus) {
  DRAKE_DEMAND(elastic_modulus > T(0));
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
  for (const VolumeVertex<T>& vertex : mesh_B->vertices()) {
    // V is a vertex of the mesh of the box with frame B.
    const Vector3<T>& r_BV = vertex.r_MV();
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
    pressure_values.push_back(elastic_modulus * extent);
  }

  return VolumeMeshFieldLinear<T, T>("pressure", std::move(pressure_values),
                                     mesh_B);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
