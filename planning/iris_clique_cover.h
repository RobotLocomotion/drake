#pragma once

#include <vector>

#include "drake/geometry/optimization/iris.h"
#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {

/** Computes an approximate collision-free convex cover of the `points` in
 configuration space.

 Cliques in a visibility graph correspond to a sampled approximation of convex
 sets in configuration space.

 The basic algorithm proceeds in a few steps:
 - Computes the planning::VisibilityGraph() of `points`.
 - Finds a maximum clique of the visibility graph, and removes the associated
   vertices from the graph. Repeats until the maximum clique is smaller than
   the `minimum_clique_size`.
 - For each clique, compute a
   Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid(), and use this as the
   IrisOptions::starting_ellipse for one iteration of IrisInConfigurationSpace.

 For small problems, `points` can be an extensive sampling of the configuration
 space. (Points in collision will be detected and ignored efficiently.) For
 larger problems, where a dense sampling of the configuration space is
 intractable due to the curse of dimensionality, `points` may represent samples
 along trajectories obtained from human demonstrations, or some more
 approximate motion-planning algorithm. This algorithm will compute an
 approximate convex decomposition of the portions of configuration space
 represented by the samples.

 Note: This algorithm leverage convex optimization solvers (e.g. Gurobi and/or
 Mosek) and nonlinear optimization (e.g. SNOPT). We recommend enabling those
 solvers if possible (https://drake.mit.edu/bazel.html#proprietary_solvers)

 Note: This algorithm can benefit greatly from OpenMP parallelization. Consider
  - using a supported platform (OpenMP is not yet supported by Drake on mac),
  - making sure this is enabled in your build
    (https://drake.mit.edu/bazel.html#openmp), and
  - making sure that `parallelize = true`.
*/
std::vector<geometry::optimization::HPolyhedron> IrisRegionsFromCliqueCover(
    const CollisionChecker& checker,
    const Eigen::Ref<const Eigen::MatrixXd>& points,
    const geometry::optimization::IrisOptions& options =
        geometry::optimization::IrisOptions(),
    int minimum_clique_size = 3, bool parallelize = true);

}  // namespace planning
}  // namespace drake
