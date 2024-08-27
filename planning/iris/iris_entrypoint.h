#pragma once

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/planning/collision_checker.h"
#include "drake/geometry/meshcat.h"

namespace drake {
namespace planning {

enum class IrisAlgorithm {
  Unset,      ///< Default value -- must be changed by the user.
  Convex,     ///< The original IRIS algorithm for convex obstacles in task space.
  NP,         ///< The IRIS-NP algorithm, which uses nonlinear programming to grow
              ///< regions in configuration space.
  NP2,        ///< The IRIS-NP2 algorithm, an improved version of IRIS-NP.
  ZO,         ///< The IRIS-ZO algorithm, which uses a gradient-free (zero-order)
              ///< optimization to grow regions in configuration space.
  Certified,  ///< The sums-of-squares certified version of the IRIS algorithm,
              ///< for growing regions in the rational parametrization of
              ///< configuration space.
};

enum class IrisRegionSpace {
  Unset,                      ///< Default value -- must be changed by the user.
  TaskSpace2d,                ///< Regions are grown in task space along the horizontal (xz)
                              ///< plane.
  TaskSpace3d,                ///< Regions are grown in task space.
  AbstractSpaceNd,            ///< Regions are grown in an abstract n-dimensional
                              ///< space. The CollisionChecker is ignored, and only
                              ///< the convex obstacles specified in the options are
                              ///< used. TODO(cohnt): Clean this explanation up.
  ConfigurationSpace,         ///< Regions are grown in configuration space.
  RationalConfigurationSpace, ///< Regions are grown in the rational
                              ///< parametrization of configuration space.
};

// TODO(cohnt): Annotate each option with which algorithms and spaces they are
// used for.
// TODO(cohnt): Support serialization.
/** Stores the options and parameters a user can set for the various IRIS
algorithms.

See @ref IrisAlgorithm for a list of algorithms, and @ref IrisRegionSpace for a
list of spaces where regions can be grown. The below table describes which
algorithms can be used with which spaces.

|           |   %TaskSpace2d  | %TaskSpace3d | %AbstractSpaceNd | %ConfigruationSpace | %RationalConfigurationSpace |
| --------: | :-------------: | :----------: | :--------------: | :-----------------: | :-------------------------: |
| Convex    |  Planned        |  Planned     |  Planned         |  Unsupported        |  Unsupported                |
| NP        |  Unsupported    |  Unsupported |  Unsupported     |  Supported          |  Planned                    |
| NP2       |  Unsupported    |  Unsupported |  Unsupported     |  Planned            |  Planned                    |
| ZO        |  Unsupported    |  Unsupported |  Unsupported     |  Planned            |  Planned                    |
| Certified |  Unsupported    |  Unsupported |  Unsupported     |  Unsupported        |  Planned                    |
__*Table 1*__: Algorithm/space compatibility matrix.

*/
struct IrisOptions {
  /** TODO(cohnt): Document */
  IrisAlgorithm algorithm{IrisAlgorithm::Unset};

  /** TODO(cohnt): Document */
  IrisRegionSpace region_space{IrisRegionSpace::Unset};

  // TODO(cohnt): Add IRIS-NP options.
  // TODO(cohnt): Verify which IRIS-NP options are also used by C-IRIS.
  // TODO(cohnt): Clean up documentation so it's not just talking about IRIS-NP.
  // TODO(cohnt): Add (original) IRIS options.
  // TODO(rhjiang): Add IRIS-NP2 options.
  // TODO(wernerpe): Add IRIS-ZO options.
  // TODO(alexandreamice): Add C-IRIS options.

  /** The polytope produced by the first iteration of each algorithm is guaranteed to contain the seed point, as long as that point is
  collision-free. However, subsequent iterations do not make this guarantee, so the IRIS paper recommends that if containment is a
  requirement, then the algorithm should simply terminate early if alternations
  would ever cause the set to not contain the point. 

  \b Algorithms: All */
  bool require_sample_point_is_contained{false};

  /** Maximum number of alternations.

  \b Algorithms: All */
  int iteration_limit{100};

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

  /** The initial hyperellipsoid used for calculating hyperplanes
  in the first iteration. If no hyperellipsoid is provided, a small hypershpere
  centered at the given sample will be used.

  \b Algorithms: All */
  std::optional<geometry::optimization::Hyperellipsoid> starting_ellipse{};

  // TODO(cohnt): Update this documentation.
  /** Optionally allows the caller to restrict the space within which IRIS
  regions are allowed to grow. By default, IRIS regions are bounded by the
  `domain` argument in the case of `Iris` or the joint limits of the input
  `plant` in the case of `IrisInConfigurationSpace`. If this option is
  specified, IRIS regions will be confined to the intersection between the
  domain and `bounding_region`.

  \b Algorithms: All */
  std::optional<geometry::optimization::HPolyhedron> bounding_region{};

  /** By default, IRIS in configuration space certifies regions for collision
  avoidance constraints and joint limits. This option can be used to pass
  additional constraints that should be satisfied by the IRIS region. We accept
  these in the form of a MathematicalProgram:

    find q subject to g(q) ≤ 0.

  The decision_variables() for the program are taken to define `q`. IRIS will
  silently ignore any costs in `prog_with_additional_constraints`, and will
  throw std::runtime_error if it contains any unsupported constraints.

  For example, one could create an InverseKinematics problem with rich
  kinematic constraints, and then pass `InverseKinematics::prog()` into this
  option.

  \b Algorithms: NP */
  const solvers::MathematicalProgram* prog_with_additional_constraints{};

  /** For each constraint in `prog_with_additional_constraints`, IRIS will
  search for a counter-example by formulating a (likely nonconvex) optimization
  problem. The initial guess for this optimization is taken by sampling
  uniformly inside the current IRIS region. This option controls the
  termination condition for that counter-example search, defining the number of
  consecutive failures to find a counter-example requested before moving on to
  the next constraint.

  \b Algorithms: NP */
  int num_additional_constraint_infeasible_samples{5};

  /** Set the initial seed for any random number generation.
  
  \b Algorithms: NP, NP2, ZO */
  int random_seed{1234};

  /** Passing a meshcat instance may enable debugging visualizations; this
  currently only happens for IrisRegionSpace::ConfigurationSpace and when the
  configuration space is <= 3 dimensional.

  \b Algorithms: NP, NP2, ZO */
  std::shared_ptr<geometry::Meshcat> meshcat{};

  /** A user-defined termination function to
  determine whether the iterations should stop. This function is called after
  computing each hyperplane at every IRIS iteration. If the function returns
  true, then the computations will stop and the last step region will be
  returned. Therefore, it is highly recommended that the termination function
  possesses a monotonic property such that for any two HPolyhedrons A and B such
  that B ⊆ A, we have if termination(A) -> termination(B). For example, a valid
  termination function is to check whether if the region does not contain any of
  a set of desired points.
  ```
  auto termination_func = [](const HPolyhedron& set) {
    for (const VectorXd& point : desired_points) {
      if (!set.PointInSet(point)) {
        return true;
      }
    }
    return false;
  };
  ```
  The algorithm will stop when as soon as the region leaves one
  of the desired points, in a similar way to how @p
  require_sample_point_is_contained is enforced.

  \b Algorithms: All */
  std::function<bool(const geometry::optimization::HPolyhedron&)> termination_func{};

  /* The `mixing_steps` parameters is passed to HPolyhedron::UniformSample to
  control the total number of hit-and-run steps taken for each new random
  sample.

  \b Algorithms: NP, NP2, ZO */
  int mixing_steps{10};

  /* SolverOptions passed into the optimization programs. For IRIS-NP and IRIS-NP2, this is sent to the nonlinear optimizer solving the counterexample search programs. For C-IRIS, these options are used for solving the SOS program.

  \b Algorithms: NP, NP2, Certified*/
  std::optional<solvers::SolverOptions> solver_options;
};

/** General entry point for the IRIS algorithm and its variants, used for
generating convex collision-free subsets in task and configuration space.

@pre options.algorithm != IrisAlgorithm::Unset
@pre options.region_space != IrisRegionSpace::Unset
*/
geometry::optimization::HPolyhedron GrowIrisRegion(
    const CollisionChecker& checker, const IrisOptions& options,
    const Eigen::VectorXd seed);

// TODO(cohnt): Add an algorithm/region_space compatibility matrix.
// TODO(cohnt): Add IRIS-NP support.
// TODO(cohnt): Add (original) IRIS support.
// TODO(rhjiang): Add IRIS-NP2 support.
// TODO(wernerpe): Add IRIS-ZO support.
// TODO(alexandreamice): Add C-IRIS support.

}  // namespace planning
}  // namespace drake
