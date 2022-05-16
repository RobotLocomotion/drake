#pragma once

#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/*
 Generates a piecewise-linear pressure field inside the given sphere as
 represented by the given volume mesh. The pressure at a point is defined
 as E * e(x) where e ∈ [0,1] is the extent -- a measure of penetration into
 the volume, and E is the given `hydroelastic_modulus`. The pressure is zero on
 the boundary with maximum E in the interior.
 @param sphere           The sphere with its canonical frame S.
 @param mesh_S           A pointer to a tetrahedral mesh of the sphere. It is
                         aliased in the returned pressure field and must remain
                         alive as long as the field. The position vectors of
                         mesh vertices are expressed in the sphere's frame S.
 @param hydroelastic_modulus  Scale extent to pressure.
 @return                 The pressure field defined on the tetrahedral mesh.
 @pre                    `hydroelastic_modulus` is strictly positive.
                         `mesh_S` represents the sphere and has enough
                         resolution to represent the pressure field.
 @tparam T               The scalar type for representing the mesh
                         vertex positions and the pressure value.
*/
template <typename T>
VolumeMeshFieldLinear<T, T> MakeSpherePressureField(
    const Sphere& sphere, const VolumeMesh<T>* mesh_S,
    const T hydroelastic_modulus) {
  // TODO(DamrongGuoy): Switch to a better implementation in the future. The
  //  current implementation has a number of limitations:
  //  1. For simplicity, we use a scaling of distance to boundary, which is
  //     not differentiable at the center of the sphere.
  //  2. Implicitly we impose the rigid core as the center of the sphere.
  //     In the future, we will consider a rigid core as an offset of the
  //     sphere.
  //  3. We do not have a mechanism to define a pressure field using barrier
  //     functions.  One possibility is to generate the mesh in offset
  //     layers, and define linear pressure fields in each offset with
  //     different elastic modulus.
  DRAKE_DEMAND(hydroelastic_modulus > T(0));
  const T radius = sphere.radius();
  std::vector<T> pressure_values;
  pressure_values.reserve(mesh_S->num_vertices());
  // A threshold to treat near-zero extent as zero extent. Some boundary
  // vertices do not lie exactly on the surface of the sphere due to rounding
  // errors. We will treat their near-zero extent as exactly zero extent.
  const T kExtentEpsilon = 1e-14;
  for (const Vector3<T>& r_SV : mesh_S->vertices()) {
    T extent = T(1.0) - r_SV.norm() / radius;
    if (extent < kExtentEpsilon) {
      extent = T(0.0);
    }
    pressure_values.push_back(hydroelastic_modulus * extent);
  }
  return VolumeMeshFieldLinear<T, T>(std::move(pressure_values), mesh_S);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
