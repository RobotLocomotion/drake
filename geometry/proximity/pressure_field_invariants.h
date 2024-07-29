#pragma once

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

/* For pressure fields computed using an elastic foundation model, pressure
relates to the distance to the surface by:
 p = (−ϕ−δ)/(H−δ)⋅E
where ϕ is the signed distance to the surface, δ the margin, E the hydroelastic
modulus, and H is a characteristic length that depends on the geometry.

Denoting with `p` the pressure with no margin (δ = 0) and with `p̃` the pressure
computed with a non-zero margin value, there is a simple, linear relationship
between `p` and `p̃`:
    p̃ = s⋅p + p̃₀
where p̃₀ = −δ/(H−δ)⋅E is the value that the pressure field with margin takes at
the boundary of the geometry, and s = H/(H−δ) > 1 is the "slope" of this linear
relation ship.

This function verifies that the above linear relationship is true, with constant
values of `s` and `p̃₀`. */
void VerifyInvariantsOfThePressureFieldWithMargin(
    const VolumeMeshFieldLinear<double, double>& field_no_magin,
    const VolumeMeshFieldLinear<double, double>& field_with_margin);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
