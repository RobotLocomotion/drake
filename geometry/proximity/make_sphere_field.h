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
 represented by the given volume mesh. The pressure at a point with distance r
 from the origin is defined as E * e(r) where e is the extent -- a measure of
 penetration into the volume, and E is the given `hydroelastic_modulus`. The
 extent is defined as e(r) = 1 - r/ (R-δ), with R the radius of the sphere and δ
 the `margin`. Therefore the zero pressure level set of this field defines
 sphere of radius R-δ. Pressure is maximum at the center, with value E.

 @param sphere           The sphere with its canonical frame S.
 @param mesh_S           A pointer to a tetrahedral mesh of the sphere. It is
                         aliased in the returned pressure field and must remain
                         alive as long as the field. The position vectors of
                         mesh vertices are expressed in the sphere's frame S.
 @param hydroelastic_modulus  Scale extent to pressure.
 @param margin           The magnitude of the margin δ, see @ref hydro_margin.
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
    const T hydroelastic_modulus, double margin = 0.0) {
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
  DRAKE_DEMAND(sphere.radius() > margin);
  using std::abs;
  const T radius = sphere.radius();
  std::vector<T> pressure_values;
  pressure_values.reserve(mesh_S->num_vertices());
  // A threshold to treat near-zero extent as zero extent. Some boundary
  // vertices do not lie exactly on the surface of the sphere due to rounding
  // errors. We will treat their near-zero extent as exactly zero extent.
  const T kExtentEpsilon = 1e-14;
  const T min_extent = -margin / (radius - margin);
  for (const Vector3<T>& r_SV : mesh_S->vertices()) {
    const T distance = radius - r_SV.norm();
    T extent = (distance - margin) / (radius - margin);
    if (abs(distance / radius) < kExtentEpsilon) {
      extent = min_extent;
    }
    pressure_values.push_back(hydroelastic_modulus * extent);
  }
  return VolumeMeshFieldLinear<T, T>(std::move(pressure_values), mesh_S,
                                     MeshGradientMode::kOkOrMarkDegenerate);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
