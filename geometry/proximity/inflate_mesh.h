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

This problem is formulated in terms of a QP with a guaranteed solution.

@warning The input volume mesh is assumed to be "watertight". With this we mean
that `mesh` is assumed to have a "connected" topology. "Soups" of disconnected
tetrahedra do not meet this requirement, but really represent a volume full of
"cracks" at where adjacent tetrahedra meet.

@warning This method can lead to the inversion of small tetrahedra, with
negative volumes, and the overlap of adjacent tetrahedra. It is the
responsibility of the calling code to verify whether this is appropriate for the
application at hand or not.

@throws std::exception if margin is negative. */
VolumeMesh<double> MakeInflatedMesh(const VolumeMesh<double>& mesh,
                                    double margin);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
