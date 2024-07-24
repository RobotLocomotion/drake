#pragma once

#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* "Inflates" a general non-convex volume mesh by creating a layer of thickness
`margin` all along the boundary.

More formally, given a margin value δ, for each i-th surface vertex this
function finds a displacement uᵢ such that the condition uᵢ⋅n̂ₐ ≥ δ, with n̂ₐ
the normal to an adjacent face, is satisfied for all adjacent faces. Internal
vertices are not modified. This function finds the minimum L2 norm displacement,
so that the solution is unique.

Notice that this procedure does not strictly enforce a layer of thickness δ, but
it is relaxed so that the margin layer is at least of thickness δ.

This problem is formulated in terms of a QP with guaranteed solution.

@throws std::exception if margin is negative. */
VolumeMesh<double> MakeInflatedMesh(const VolumeMesh<double>& mesh,
                                    double margin);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
