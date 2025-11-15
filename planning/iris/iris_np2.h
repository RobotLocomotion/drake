#pragma once

#include <string>

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/iris/iris_common.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {

namespace internal {
/** Sampling strategies for the IRIS algorithm. */
enum class IrisNp2SamplingStrategy {
  kGreedySampler,
  kRaySampler,
};

/** Writes the IrisNp2SamplingStrategy as a string. */
std::ostream& operator<<(std::ostream& out, const IrisNp2SamplingStrategy& t);

/** Instantiates a IrisNp2SamplingStrategy from its string representation.
 @param spec  Must be 'greedy' or 'ray'.
 @throws if `spec` is an unrecognized string. */
IrisNp2SamplingStrategy iris_np2_sampling_strategy_from_string(
    const std::string& spec);
}  // namespace internal

/** RaySamplerOptions contains settings specific to the kRaySampler strategy for
 * drawing the initial samples.
 *
 * @ingroup planning_iris */
struct RaySamplerOptions {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background.
  Note: This only serializes options that are YAML built-in types. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(only_walk_toward_collisions));
    a->Visit(DRAKE_NVP(ray_search_num_steps));
    a->Visit(DRAKE_NVP(num_particles_to_walk_towards));
  }

  /** If true, the ray stepping strategy is only applied to samples which are
   * initially in collision. If false, it is applied to all samples. */
  bool only_walk_toward_collisions{false};

  /** The step size for the ray search is defined per-particle, as the distance
   * between the current ellipsoid center and the particle, divided by this
   * option. A larger number requires more time for computing samples, but will
   * lead to the samples in-collision being closer to the ellipsoid center, and
   * higher quality hyperplanes. We choose a default value to roughly match the
   * results in [Werner et al., 2024]. Must be at least 1. */
  int ray_search_num_steps{10};

  /** The number of particles to step towards. Ignored if
   * only_walk_toward_collisions is true, because we walk toward all collisions
   * in that case. Must be at least 1. */
  int num_particles_to_walk_towards{200};
};

/**
 * IrisNp2Options collects all parameters for the IRIS-NP2 algorithm.
 *
 * @experimental
 * @see IrisNp2 for more details.
 *
 * @ingroup planning_iris */
class IrisNp2Options {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IrisNp2Options);

  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background.
  Note: This only serializes options that are YAML built-in types. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(sampled_iris_options));
    a->Visit(DRAKE_NVP(sampling_strategy));
    a->Visit(DRAKE_NVP(ray_sampler_options));
  }

  IrisNp2Options() = default;

  /** The user can specify a solver to use for the counterexample search
   * program. If nullptr (the default value) is given, then
   * solvers::MakeFirstAvailableSolver will be used to pick the solver. */
  const solvers::SolverInterface* solver{nullptr};

  /** Options passed to the counterexample search program solver. */
  solvers::SolverOptions solver_options{};

  /** Options common to IRIS-type algorithms. */
  CommonSampledIrisOptions sampled_iris_options{};

  /** Parameterization of the subspace along which to grow the region. Default
   * is the identity parameterization, corresponding to growing regions in the
   * ordinary configuration space. */
  IrisParameterizationFunction parameterization{};

  /** Which sampling strategy to use when growing the region. Use "ray" for
   * kRaySmpler, and "greedy" for kGreedySampler. kRaySampler finds collisions
   * closer to the ellipsoid center in order to achieve more efficient
   * hyperplane placement, yielding fewer hyperplanes in the resulting region,
   * but may take more runtime than kGreedySampler.
   * @note See §5.3 of [Werner et al., 2024] for further details. */
  std::string sampling_strategy{"greedy"};

  /** Additional options for kRaySampler. Ignored if kGreedySampler is used. */
  RaySamplerOptions ray_sampler_options;

  /** Add a hyperplane at a particle in collision if the nonlinear solve
   * (initialized at that point) fails. Generally leads to regions with more
   * faces, but helpful for getting the algorithm unstuck if most nonlinear
   * solves are failing. */
  bool add_hyperplane_if_solve_fails{false};
};

/** The IRIS-NP2 (Iterative Regional Inflation by Semidefinite and Nonlinear
Programming 2) algorithm, as described in

[Werner et al., 2024] P. Werner, T. Cohn\*, R. H. Jiang\*, T. Seyde, M.
Simchowitz, R. Tedrake, and D. Rus, "Faster Algorithms for Growing
Collision-Free Convex Polytopes in Robot Configuration Space,"
&nbsp;* Denotes equal contribution.

https://groups.csail.mit.edu/robotics-center/public_papers/Werner24.pdf

This algorithm constructs probabilistically collision-free polytopes in robot
configuration space using a scene graph collision checker. The sets are
constructed by identifying collisions with sampling and nonlinear programming.
The produced polytope P is probabilistically collision-free in the sense that
one gets to control the probability δ that the fraction of the
volume-in-collision is larger than ε

Pr[λ(P\Cfree)/λ(P) > ε] ≤ δ.

@param starting_ellipsoid provides the initial ellipsoid around which to grow
the region. This is typically a small ball around a collision-free
configuration (e.g. Hyperellipsoid::MakeHyperSphere(radius, seed_point)). The
center of this ellipsoid is required to be collision-free.
@param domain describes the total region of interest; computed IRIS regions will
be inside this domain. It must be bounded, and is typically a simple bounding
box representing joint limits (e.g. from HPolyhedron::MakeBox).
@param options contains algorithm parameters such as the desired collision-free
fraction, confidence level, and various algorithmic settings.

The @p starting_ellipsoid and @p domain must describe elements in the same
ambient dimension as the configuration space of the robot, unless a
parameterization is specified (in which case, they must match
`options.parameterization_dimension`).
@return A HPolyhedron representing the computed collision-free region in
configuration space.
@ingroup robot_planning
@experimental

@throws if the center of `starting_ellipsoid` is in collision, or violates any
of the user-specified constraints in `options.prog_with_additional_constraints`.

@note This can be a long running function that needs to solve many QPs. If you
have a solver which requires a license, consider acquiring the license before
solving this function. See AcquireLicense for more details.

IrisNp2 is still in development, so certain features of
SceneGraphCollisionChecker and parts of [Werner et al., 2024] are not yet
supported.

@throws if any collision pairs in `checker` have negative padding.
@throws if any collision geometries have been been added in `checker`.

@ingroup planning_iris */

geometry::optimization::HPolyhedron IrisNp2(
    const SceneGraphCollisionChecker& checker,
    const geometry::optimization::Hyperellipsoid& starting_ellipsoid,
    const geometry::optimization::HPolyhedron& domain,
    const IrisNp2Options& options = IrisNp2Options());
}  // namespace planning
}  // namespace drake
