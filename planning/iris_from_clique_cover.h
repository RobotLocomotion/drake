#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

#include "drake/common/parallelism.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/planning/graph_algorithms/max_clique_solver_base.h"
#include "drake/planning/graph_algorithms/max_clique_solver_via_mip.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace planning {
/**
 * The default configurations for running IRIS when building a convex set from a
 * clique. Currently, it is recommended to only run IRIS for one iteration when
 * building from a clique so as to avoid discarding the information gained from
 * the clique.
 */
struct IrisFromCliqueCoverOptions {
  IrisFromCliqueCoverOptions() = default;

  /**
   * The options used on internal calls to IRIS.
   */
  geometry::optimization::IrisOptions iris_options{.iteration_limit = 1};

  /**
   * The fraction of the domain that must be covered before we terminate the
   * algorithm.
   */
  double coverage_termination_threshold{0.7};

  /**
   * The maximum number of iterations before the algorithm terminates.
   */
  int iteration_limit{100};

  /**
   * The number of points to sample when testing coverage.
   */
  int num_points_per_coverage_check{static_cast<int>(1e3)};

  /**
   * The amount of parallelism to use when performing the coverage check.
   */
  Parallelism num_coverage_checkers{Parallelism::Max()};

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
   * The max clique solver used.
   */
  std::unique_ptr<planning::graph_algorithms::MaxCliqueSolverBase>
      max_clique_solver{
          new planning::graph_algorithms::MaxCliqueSolverViaMip()};

  /**
   * The number of threads used to build sets. It is recommended to set this no
   * larger than 1 - the hardware concurrency of the current machine. If this
   * number is larger than the implicit_context_parallelism of the collision
   * checker used in IrisInConfigurationSpaceFromCliqueCover then it will be
   * overridden to be no larger than this implicit context parallelism.
   */
  Parallelism num_builders{std::max(1, Parallelism::Max().num_threads() - 1)};

  /**
   * The rank tolerance used for computing the
   * MinimumVolumeCircumscribedEllipsoid of a clique. See
   * @MinimumVolumeCircumscribedEllipsoid.
   */
  double rank_tol_for_lowner_john_ellipse{1e-6};

  /**
   * The tolerance used for checking whether a point is contained inside an
   * HPolyhedron. See @ConvexSet::PointInSet.
   */
  double point_in_set_tol{1e-6};

  /**
   * The random generator used as the source of randomness across the whole
   * run of the method.
   */
  RandomGenerator generator{};

  /**
   * The amount of parallelism to use when constructing the visibility graph. If
   * this number is larger than the implicit_context_parallelism of the
   * collision checker used in IrisInConfigurationSpaceFromCliqueCover then it
   * will be overridden to be no larger than this implicit context parallelism.
   */
  Parallelism visibility_graph_parallelism{Parallelism::Max()};
};

/**
 * Cover the configuration space in IRIS regions using the Visibility Clique
 * Cover Algorithm.
 * @param checker The collision checker containing the plant and it's associated
 * scene_graph
 * @param generator The source of randomness across the entire run of the
 * algorithm.
 * @param sets [in/out] initial sets covering the space (potentially empty).
 * The cover is written into this vector.
 */
void IrisInConfigurationSpaceFromCliqueCover(
    const SceneGraphCollisionChecker& checker,
    const IrisFromCliqueCoverOptions& options, RandomGenerator* generator,
    std::vector<geometry::optimization::HPolyhedron>* sets);

}  // namespace planning
}  // namespace drake
