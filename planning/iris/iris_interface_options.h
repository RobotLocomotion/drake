#pragma once

#include "drake/common/random.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"

namespace drake {
namespace planning {

struct IrisInterfaceOptions {
  /** Maximum number of iterations. */
  int iteration_limit{100};

  /** A function which returns true when a set is done being constructed.*/
  std::function<bool(const geometry::optimization::HPolyhedron&)>
      termination_func;

  /** Solver options used by any internal solvers used in the loop. */
  std::optional<solvers::SolverOptions> solver_options;

  /** The domain in which the set is constructed. If unset, the plant's joint
   * limits are used. */
  std::optional<geometry::optimization::HPolyhedron> domain;

  /** A default randomness generator for source of randomness which may occur in
   * the algorithm. */
  RandomGenerator random_generator{0};

  /** Passing a meshcat instance may enable debugging visualizations. */
  std::shared_ptr<geometry::Meshcat> meshcat{};
};

}  // namespace planning
}  // namespace drake