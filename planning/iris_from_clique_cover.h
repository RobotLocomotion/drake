#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/planning/graph_algorithms/max_clique_solver_base.h"
#include "drake/planning/graph_algorithms/max_clique_solver_via_mip.h"
#include "drake/planning/iris_region_from_clique_builder.h"
#include "drake/planning/point_sampler_base.h"

namespace drake {
namespace planning {
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;
using geometry::optimization::IrisOptions;
using planning::graph_algorithms::MaxCliqueSolverBase;
using planning::graph_algorithms::MaxCliqueSolverViaMip;

struct IrisFromCliqueCoverOptions {
  IrisFromCliqueCoverOptions() = default;

  /**
   * The options used on internal calls to IRIS.
   */
  IrisOptions iris_options{DefaultIrisOptionsForIrisRegionFromCliqueBuilder()};

  /**
   * The amount of coverage at which we terminate
   */
  double coverage_termination_threshold{0.7};

  /**
   * Number of points to sample when testing coverage
   */
  int num_points_per_coverage_check{static_cast<int>(1e3)};

  /**
   * Minimum clique size
   */
  int minimum_clique_size{3};

  /**
   * Number of points to sample when building visibilty cliques.
   */
  int num_points_per_visibility_round{200};

  /**
   * The distribution from which to draw points at random. If this option is
   * nullopt, then points will be drawn uniformly from domain.
   */
  std::optional<std::shared_ptr<PointSamplerBase>> point_sampler{std::nullopt};

  /**
   * The max clique solver used.
   */
  std::unique_ptr<MaxCliqueSolverBase> max_clique_solver{
      new MaxCliqueSolverViaMip()};

  /**
   * The number of threads used to build sets. If this number is less than 0,
   * then we will use the hardware concurrency minus 1 builders.
   */
  int num_builders{-1};

  /**
   * The rank tolerance used for the MinimumVolumeCircumscribedEllipsoid.
   */
  double rank_tol_for_lowner_john_ellipse{1e-6};

  /**
   * The random generator used as the source of randomness across the whole run
   * of the method.
   */
  RandomGenerator generator{};
};

void IrisFromCliqueCover(const ConvexSets& obstacles, const HPolyhedron& domain,
                         const IrisFromCliqueCoverOptions& options,
                         std::vector<copyable_unique_ptr<HPolyhedron>>* sets);

}  // namespace planning
}  // namespace drake
