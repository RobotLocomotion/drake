#pragma once

#include <Eigen/Sparse>

#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {

/** Given some number of sampled points in the configuration space of
`checker`'s plant(), computes the "visibility graph" -- two `points` have an
edge between them if the line segment connecting them is collision free. See
the CollisionChecker documentation for details on how this *approximate* edge
collision checking is configured.

If `parallelize=`==true, then the
CollisionCheckerParams::distance_and_interpolation_provider for `checker` must
be implemented in C++, either by providing the C++ implementation directly
directly or by using the default provider.

@returns the adjacency matrix, A(i,j) == true iff points.col(i) is visible from
points.col(j). A is always symmetric.

@pre points.rows() == total number of positions in the collision checker plant.
*/
Eigen::SparseMatrix<bool> VisibilityGraph(
    const CollisionChecker& checker,
    const Eigen::Ref<const Eigen::MatrixXd>& points, bool parallelize = true);

}  // namespace planning
}  // namespace drake
