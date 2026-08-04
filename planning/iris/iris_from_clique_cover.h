#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <variant>
#include <vector>

#include "drake/common/parallelism.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/planning/graph_algorithms/max_clique_solver_base.h"
#include "drake/planning/graph_algorithms/max_clique_solver_via_greedy.h"
#include "drake/planning/iris/iris_np2.h"
#include "drake/planning/iris/iris_zo.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace planning {

struct IrisFromCliqueCoverOptions {
  // TODO(cohnt): Support user-specified parameterizations contained in
  // IrisZoOptions and IrisNp2Options. Note from Alexandre.Amice: the reason we
  // don't support subspaces is due to the fact that the current code does not
  // know how to draw samples from the subspace. You will need to add this
  // feature if you want to be able to implement CliqueCovers on the subspaces.
  /**
   * The options used on internal calls to Iris. The type of this option
   * determines which variant of Iris is called. Currently, it is recommended to
   * only run Iris for one iteration when building from a clique so as to avoid
   * discarding the information gained from the clique.
   *
   * Note that `IrisOptions` can optionally include a meshcat instance to
   * provide debugging visualization. If this is provided `IrisFromCliqueCover`
   * will provide debug visualization in meshcat showing where in configuration
   * space it is drawing from. However, if the parallelism option is set to
   * allow more than 1 thread, then the debug visualizations of internal Iris
   * calls will be disabled. This is due to a limitation of drawing to meshcat
   * from outside the main thread.
   * @note some of these variants specify a parallelism parameter. In
   * IrisInConfigurationSpaceFromCliqueCover, the iris_options.parallelism is
   * ignored and the value of parallelism specified by `this.parallelism` will
   * be used instead.
   *
   * @ingroup planning_iris
   */
  std::variant<geometry::optimization::IrisOptions, IrisNp2Options,
               IrisZoOptions>
      iris_options{geometry::optimization::IrisOptions{.iteration_limit = 1}};

  /**
   * The fraction of the domain that must be covered before we terminate the
   * algorithm.
   */
  double coverage_termination_threshold{0.7};

  /**
   * The maximum number of iterations of the algorithm.
   */
  int iteration_limit{100};

  /**
   * The number of points to sample when testing coverage.
   */
  int num_points_per_coverage_check{static_cast<int>(1e3)};

  /**
   * The amount of parallelism to use. This algorithm makes heavy use of
   * parallelism at many points and thus it is highly recommended to set this to
   * the maximum tolerable parallelism.
   */
  Parallelism parallelism{Parallelism::Max()};

  /**
   * The minimum size of the cliques used to construct a region. If this is set
   * lower than the ambient dimension of the space we are trying to cover, then
   * this option will be overridden to be at least 1 + the ambient dimension.
   */
  int minimum_clique_size{3};

  /**
   * Number of points to sample when building visibilty cliques. If this option
   * is less than twice the minimum clique size, it will be overridden to be at
   * least twice the minimum clique size. If the algorithm ever fails to find a
   * single clique in a visibility round, then the number of points in a
   * visibility round will be doubled.
   */
  int num_points_per_visibility_round{200};

  /**
   * The rank tolerance used for computing the
   * MinimumVolumeCircumscribedEllipsoid of a clique. See
   * @MinimumVolumeCircumscribedEllipsoid.
   */
  double rank_tol_for_minimum_volume_circumscribed_ellipsoid{1e-6};

  /**
   * The tolerance used for checking whether a point is contained inside an
   * HPolyhedron. See @ConvexSet::PointInSet.
   */
  double point_in_set_tol{1e-6};

  // TODO(AlexandreAmice): Implement a constructor/option that automatically
  // sets up the ILP solver and selects MaxCliqueViaMip
};

/**
 * Cover the configuration space in Iris regions using the Visibility Clique
 * Cover Algorithm as described in
 *
 * P. Werner, A. Amice, T. Marcucci, D. Rus, R. Tedrake "Approximating Robot
 * Configuration Spaces with few Convex Sets using Clique Covers of Visibility
 * Graphs" In 2024 IEEE Internation Conference on Robotics and Automation.
 * https://arxiv.org/abs/2310.02875
 *
 * @param checker The collision checker containing the plant and its associated
 * scene_graph.
 * @param generator There are points in the algorithm requiring randomness. The
 * generator controls this source of randomness.
 * @param sets [in/out] initial sets covering the space (potentially empty).
 * The cover is written into this vector.
 * @param max_clique_solver The min clique cover problem is approximatley solved
 * by repeatedly solving max clique on the uncovered graph and adding this
 * largest clique to the cover. The max clique problem is solved by this solver.
 * If parallelism is set to allow more than 1 thread, then the solver **must**
 * be implemented in C++.
 *
 * If nullptr is passed as the `max_clique_solver`, then max clique will be
 * solved using an instance of MaxCliqueSolverViaGreedy, which is a fast
 * heuristic. If higher quality cliques are desired, consider changing the
 * solver to an instance of MaxCliqueSolverViaMip. Currently, the padding in the
 * collision checker is not forwarded to the algorithm, and therefore the final
 * regions do not necessarily respect this padding. Effectively, this means that
 * the regions are generated as if the padding is set to 0. This behavior may be
 * adjusted in the future at the resolution of #18830.
 *
 * @note that MaxCliqueSolverViaMip requires the availability of a
 * Mixed-Integer Linear Programming solver (e.g. Gurobi and/or Mosek). We
 * recommend enabling those solvers if possible because they produce higher
 * quality cliques (https://drake.mit.edu/bazel.html#proprietary_solvers). The
 * method will throw if @p max_clique_solver cannot solve the max clique
 * problem.
 * @note If IrisNp2Options is used, then the collision checker must be a
 * SceneGraphCollisionChecker.
 * @throw std::exception Parameterizations are not currently supported for
 * `IrisZo` and `IrisNp2` when running `IrisFromCliqueCover`. This method will
 * throw if options.iris_options is of type `IrisZoOptions` or `IrisNp2Options`
 * and specifies a parametrization function. See the documentation of
 * `IrisZoOptions` and `IrisNp2Options` for more information about subspace
 * parametrization.
 * @throw std::exception If the
 * options.iris_options.prog_with_additional_constraints is not nullptr i.e. if
 * a prog with additional constraints is provided.
 *
 * @ingroup planning_iris
 */
void IrisInConfigurationSpaceFromCliqueCover(
    const CollisionChecker& checker, const IrisFromCliqueCoverOptions& options,
    RandomGenerator* generator,
    std::vector<geometry::optimization::HPolyhedron>* sets,
    const planning::graph_algorithms::MaxCliqueSolverBase* max_clique_solver =
        nullptr);

}  // namespace planning
}  // namespace drake
