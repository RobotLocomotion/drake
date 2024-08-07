#pragma once

#include <map>

#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* "Inflates" a general non-convex volume mesh by creating a layer of thickness
`margin` all along the boundary.

More formally, given a non-negative margin value δ, for each i-th surface vertex
this function finds a displacement uᵢ such that the condition uᵢ⋅n̂ₐ ≥ δ, with n̂ₐ
the outward normal to an adjacent surface face, is satisfied for all such
adjacent faces a ∈ Aₛ. Internal vertices are not modified. This function finds
the minimum L2 norm displacement, so that the solution is unique.

Notice that this procedure does not strictly enforce a layer of thickness δ, but
it is relaxed so that the margin layer is at least of thickness δ.

The input mesh may contain vertices whose constraint set is infeasible for any
positive δ. In other words, the set { uᵢ⋅n̂ₐ ≥ δ | ∀a ∈ Aₛ } is empty. This can
happen, for instance, if n̂ₐⱼ⋅n̂ₐₖ = −1 for two faces aⱼ and aₖ, adjacent to vᵢ.
In this instance, we partition the set of adjacent faces such that each
partition presents feasible constraints. The original vertex is duplicated so
that a unique copy is associated with each partition. For N > 1 partitions, we
add N - 1 duplicate "split" vertices (see `split_vertex_to_original`). A
displacement (uᵢ₀, ..., uᵢₖ) is found for each "split" vertex and face
partition, such that uᵢⱼ⋅n̂ₐ ≥ δ is satisfied for the partition's faces. The
resulting constraint sets are guaranteed to be non-empty.

This problem is formulated in terms of a QP with a guaranteed solution.

@warning If the program is forced to "split" vertices in order to find a
solution, the resulting mesh will have a different topology in the neighborhood
of the split vertex. The inflated mesh will have discontinuities near the vertex
potentially leading to overlapping tetrahedra or apparent cracks in the
inflated mesh. The magnitude of these defects is approximately that of δ.

@warning The input volume mesh is assumed to be "watertight". With this we mean
that adjacent tetrahedra that span a contiguous internal region, should be built
from their common and not duplicate coincident vertices. Introducing duplicate
contiguous vertices turns the contiguous mesh into a tetrahedral "soup" leading
to an undesirable inflation.

@warning This method can lead to the inversion of small tetrahedra, with
negative volumes, and the overlap of adjacent tetrahedra. It is the
responsibility of the calling code to verify whether this is appropriate for the
application at hand or not.

@param[in] mesh The input mesh to inflate.
@param[in] margin The non-negative margin δ.
@param[out] split_vertex_to_original A mapping of the index of "split" vertices
in the returned mesh, to the index of the vertex that was split in `mesh`.
Additional vertices in the returned mesh created to accomodate "split" vertices
have index >= mesh.num_vertices(). Hence, for any pair (w, v) in the map, w >=
mesh.num_vertices() and v < mesh.num_vertices(). A vertex with index v in the
original mesh that was split is reused as one of the "split" vertices in the
returned mesh. Thus (v, v), while not included in the map itself, is implicitly
a pair in this mapping.

@throws std::exception if margin is negative.
@throws std::exception if new_vertices == nullptr */
VolumeMesh<double> MakeInflatedMesh(
    const VolumeMesh<double>& mesh, double margin,
    std::map<int, int>* split_vertex_to_original);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
