#pragma once

#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Sets up the equilibrium plane between the two linear functions from the
 two tetrahedra. The normal vector of the plane will point *out* of field1
 and *into* field0.

 @retval true if the equilibrium plane exists. The plane may not exist, for
         example, if the two fields have the same gradient vectors, which
         means they have equal values everywhere in space or nowhere at all.

 @pre For each field, the field value increases from the boundary into the
      interior.
 */
template <typename T>
bool CalcEquilibriumPlane(int element0,
                          const VolumeMeshFieldLinear<double, double>& field0_M,
                          int element1,
                          const VolumeMeshFieldLinear<double, double>& field1_N,
                          const math::RigidTransform<T>& X_MN,
                          Plane<T>* plane_M);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
