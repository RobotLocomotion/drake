#pragma once

#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Sets up the equilibrium plane between two linear functions where they
 are equal.

 The first linear function f₀:ℝ³→ ℝ is specified by the tetrahedron index
 `element0` into the piecewise-linear field `field0_M`, expressed in frame M.
 The second function f₁:ℝ³→ ℝ is specified by the tetrahedron index
 `element1` into the piecewise-linear field `field1_N`, expressed in frame N.

 @param[in] X_MN Relative pose of frame N in frame M.

 @param[out] plane_M  Equilibrium plane expressed in frame M if it exists.
 It is not set if the gradient of the two field are deemed identical.
 Its unit normal vector n̂ will point *out of* f₁ and *into* f₀, i.e.,
 n̂ points in the direction of increasing f₀ and decreasing f₁:

                 n̂ = (∇f₀−∇f₁)/‖∇f₀−∇f₁‖

 @retval true if the equilibrium plane exists. The plane may not exist if the
  two functions have the same gradient vector, which means they are equal
  everywhere, or they are nowhere equal. We use an internal threshold to
  decide.

 @note The equilibrium plane may or may not intersect the tetrahedra.

 @throw std::runtime_error if `field0_M` or `field1_N` has no gradient
 calculated.

 @tparam T A valid Eigen scalar.
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
