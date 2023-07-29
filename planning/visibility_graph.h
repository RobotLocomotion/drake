#pragma once

#include <Eigen/Sparse>

#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {

/** Given some number of sampled points in the configuration space of `plant`,
computes the "visibility graph" -- two `points` have an edge between them if
the line segment connecting them is collision free.

If parallelize==true, then the configuration_distance_function in the
`collision_checker` must be implemented in C++, either by providing the C++
callback directly or by using CollisionCheckerParams::distance_function_weights
to define the distance function.

@returns the adjacency matrix, A(i,j) == true iff points.col(i) is visible from
points.col(j). A is always symmetric.

@pre points.rows() == total number of positions the collision checker plant.
*/
Eigen::SparseMatrix<bool> VisibilityGraph(
    const CollisionChecker& checker,
    const Eigen::Ref<const Eigen::MatrixXd>& points, bool parallelize = true);

}  // namespace planning
}  // namespace drake
