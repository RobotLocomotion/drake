#pragma once

#include <Eigen/Sparse>

#include "drake/common/parallelism.h"
#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {

/** Given some number of sampled points in the configuration space of
`checker`'s plant(), computes the "visibility graph" -- two `points` have an
edge between them if the line segment connecting them is collision free. See
CollisionChecker documentation for more information on how edge collision
checks are performed.

Note that this method assumes the collision checker has symmetric behavior
(i.e. checking edge (q1, q2) is the same as checking edge (q2, q1)). This is
true for many collision checkers (e.g. those using
LinearDistanceAndInterpolationProvider, which is the default), but some more
complex spaces with non-linear interpolation (e.g. a Dubin's car) are not
symmetric.

If `parallelize` specifies more than one thread, then the
CollisionCheckerParams::distance_and_interpolation_provider for `checker` must
be implemented in C++, either by providing the C++ implementation directly
directly or by using the default provider.

@returns the adjacency matrix, A(i,j) == true iff points.col(i) is visible from
points.col(j). A is always symmetric.

@pre points.rows() == total number of positions in the collision checker plant.
*/
Eigen::SparseMatrix<bool> VisibilityGraph(
    const CollisionChecker& checker,
    const Eigen::Ref<const Eigen::MatrixXd>& points,
    Parallelism parallelize = Parallelism::Max());

}  // namespace planning
}  // namespace drake
