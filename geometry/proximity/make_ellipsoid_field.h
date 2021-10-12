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
 Generates a piecewise-linear pressure field inside the given ellipsoid as
 represented by the given volume mesh. The pressure at a point is defined
 as E * e(x) where e ∈ [0,1] is the extent -- a measure of penetration into
 the volume, and E is the given `hydroelastic_modulus`. The pressure is zero on
 the boundary with maximum E in the interior.
 @param ellipsoid        The ellipsoid with its canonical frame E.
 @param mesh_E           A pointer to a tetrahedral mesh of the ellipsoid. It
                         is aliased in the returned pressure field and must
                         remain alive as long as the field. The position
                         vectors of mesh vertices are expressed in the
                         ellipsoid's frame E.
 @param hydroelastic_modulus  Scale extent to pressure.
 @return                 The pressure field defined on the tetrahedral mesh.
 @pre                    `hydroelastic_modulus` is strictly positive.
                         `mesh_E` represents the ellipsoid and has enough
                         resolution to represent the pressure field.
 @tparam T               The scalar type for representing the mesh
                         vertex positions and the pressure value.
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeEllipsoidPressureField(
    const Ellipsoid& ellipsoid, const VolumeMesh<T>* mesh_E,
    const T hydroelastic_modulus) {
  // TODO(DamrongGuoy): Switch to a better implementation in the future.
  //  The current implementation uses a simple map from the ellipsoid to the
  //  unit sphere and assigns extent on the ellipsoid from the extent on the
  //  unit sphere, which is a scaled distance to boundary of the unit sphere.
  //  The current implementation has a number of limitations:
  //  1. It is not differentiable at the center of the ellipsoid.
  //  2. It is not uniformly stiff. It is stiffer along a shorter principal
  //     axis than a longer principal axis of the same ellipsoid, i.e., the
  //     gradient of pressure field is larger on the shorter principal axis.
  //  3. Implicitly we impose the rigid core as the center of the ellipsoid.
  //     In the future, we will consider a rigid core as an offset of the
  //     ellipsoid. The offset may or may not have uniform thickness from the
  //     boundary.
  //  4. We do not have a mechanism to define a pressure field using barrier
  //     functions.  One possibility is to generate the mesh in offset
  //     layers, and define linear pressure fields in each offset with
  //     different elastic modulus.
  DRAKE_DEMAND(hydroelastic_modulus > T(0));
  const T a = ellipsoid.a();
  const T b = ellipsoid.b();
  const T c = ellipsoid.c();
  // For scaling a position vector in the ellipsoid to the unit sphere.
  const Vector3<T> scale{T(1.0) / a, T(1.0) / b, T(1.0) / c};
  std::vector<T> pressure_values;
  pressure_values.reserve(mesh_E->num_vertices());
  // A threshold to treat near-zero extent as zero extent. Some boundary
  // vertices do not lie exactly on the surface of the ellipsoid due to
  // rounding errors. We will treat their near-zero extent as exactly zero
  // extent.
  const T kExtentEpsilon = 1e-14;
  for (const Vector3<T>& r_EV : mesh_E->vertices()) {
    // Scale V in the ellipsoid to U in the unit sphere.
    const Vector3<T> r_EU = scale.cwiseProduct(r_EV);
    T extent = T(1.0) - r_EU.norm();
    if (extent < kExtentEpsilon) {
      extent = T(0.0);
    }
    pressure_values.push_back(hydroelastic_modulus * extent);
  }
  return VolumeMeshFieldLinear<T, T>(std::move(pressure_values), mesh_E);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
