#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/iris/iris_interface.h"
#include "drake/planning/iris/iris_interface_options.h"

namespace drake {
namespace planning {
namespace internal {

struct IrisViaCollisionsAndEllipsoidInterfaceOptions
    : public IrisInterfaceOptions {
  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this threshold. This termination condition can
  be disabled by setting to a negative value.

  \b Algorithms: All */
  double absolute_termination_threshold{2e-2};  // from rdeits/iris-distro.

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this percent of the previous best volume.
  This termination condition can be disabled by setting to a negative value.

  \b Algorithms: All */
  double relative_termination_threshold{1e-3};  // from rdeits/iris-distro.

  // TODO(russt): Improve the implementation so that we can clearly document the
  // units for this margin.
  /** When adding hyperplanes, we retreat by this margin from each
  C-space obstacle in order to avoid the possibility of requiring an infinite
  number of faces to approximate a curved boundary.

  \b Algorithms: NP, NP2, and ZO */
  double configuration_space_margin{1e-2};

  /** For each possible collision, IRIS will search for a counter-example by
  formulating a (likely nonconvex) optimization problem. The initial guess for
  this optimization is taken by sampling uniformly inside the current IRIS
  region. This option controls the termination condition for that
  counter-example search, defining the number of consecutive failures to find a
  counter-example requested before moving on to the next constraint.

  \b Algorithms: NP */
  int num_collision_infeasible_samples{5};

  /** For IRIS in configuration space, it can be beneficial to not only specify
  task-space obstacles (passed in through the plant) but also obstacles that are
  defined by convex sets in the configuration space. This option can be used to
  pass in such configuration space obstacles.

  \b Algorithms: All */
  geometry::optimization::ConvexSets configuration_obstacles{};

  /** The current seed point around which a region is grown. */
  Eigen::VectorXd seed{};

  /** The initial hyperellipsoid used for calculating hyperplanes
  in the first iteration. If no hyperellipsoid is provided, a small hypershpere
  centered at the current seed will be used. */
  std::optional<geometry::optimization::Hyperellipsoid> starting_ellipse{};

  /** The p in the unadpative test from Faster Algorithms for Growing
   * Collision-Free Convex Polytopes*/
  double p{0.9};

  /** The delta in the unadpative test from Faster Algorithms for Growing
   * Collision-Free Convex Polytopes*/
  double delta{0.9};

  /** The tau in the unadpative test from Faster Algorithms for Growing
   * Collision-Free Convex Polytopes*/
  double tau{0.9};

  /** Number of mixing steps used when sampling from the set during the
   * unadpative test. */
  int mixing_steps{10};
};

}  // namespace internal
}  // namespace planning
}  // namespace drake