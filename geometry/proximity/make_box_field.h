#pragma once

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/*
 Generates a linear approximation of a pressure field inside the given box as
 represented by the given volume mesh. The pressure at a point is defined
 as E * e(x) where e ∈ [0,1] is the extent -- a measure of penetration into
 the volume, and E is the given `hydroelastic_modulus`. The pressure is zero on
 the boundary with maximum E in the interior.
 @param box              The box with its canonical frame B.
 @param mesh_B           A pointer to a tetrahedral mesh of the box. It is
                         aliased in the returned pressure field and must remain
                         alive as long as the field. The position vectors of
                         mesh vertices are expressed in the box's frame B.
 @param hydroelastic_modulus  Scale extent to pressure.
 @return                 The pressure field defined on the tetrahedral mesh.
 @pre                    `hydroelastic_modulus` is strictly positive.
                         `mesh_B` represents the box well (the space enclosed
                         by the mesh should be exactly the same space as the
                         box specification). `mesh_B` has enough resolution
                         to approximate the pressure field.
 @tparam_nonsymbolic_scalar
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeBoxPressureField(const Box& box,
                                                 const VolumeMesh<T>* mesh_B,
                                                 const T hydroelastic_modulus);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
