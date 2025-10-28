#pragma once

#include <limits>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

/* This function compares two "elastic foundation" pressure fields defined on a
same mesh.

For pressure fields computed using an elastic foundation model, pressure
relates to the distance to the surface by:
 p = (−ϕ−δ)/(H−δ)⋅E
where ϕ is the signed distance to the surface, δ the margin, E the hydroelastic
modulus, and H is a characteristic length that depends on the geometry.

Denoting with `p` the pressure with no margin (δ = 0) and with `p̃` the pressure
computed with a positive margin value, there is a simple, linear relationship
between `p` and `p̃`:
    p̃ = s⋅p + p̃₀
where p̃₀ = −δ/(H−δ)⋅E is the value that the pressure field with margin takes at
the boundary of the geometry, and s = H/(H−δ) > 1 is the "slope" of this linear
relation ship.

For such pressure fields, this function verifies the invariants:
 1. The two fields are defined on the same mesh.
 2. Pressure on the surface is uniform and equal to `p̃₀`.
 3. The linear relationship slope is uniform and equal to s.

@param[in] field_no_margin The pressure field for δ = 0.
@param[in] field_with_margin The pressure field for δ > 0.
@param[in] margin The margin value δ.
@param[in] elastic_foundation_depth The Elastic Foundation depth H.
@param[in] hydroelastic_modulus The hydroelastic modulus E.
@param[in] relative_tolerance Relative dimensionless tolerance used to perform
floating point comparisons.

@pre The two fields are defined on the same mesh. */
void VerifyInvariantsOfThePressureFieldWithMargin(
    const VolumeMeshFieldLinear<double, double>& field_no_margin,
    const VolumeMeshFieldLinear<double, double>& field_with_margin,
    double margin, double elastic_foundation_depth, double hydroelastic_modulus,
    double relative_tolerance = std::numeric_limits<double>::epsilon());

}  // namespace internal
}  // namespace geometry
}  // namespace drake
