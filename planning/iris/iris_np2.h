#pragma once

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/iris/iris_common.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {
/**
 * IrisNp2Options collects all parameters for the IRIS-NP2 algorithm.
 *
 * @experimental
 * @see IrisNp2 for more details.
 **/
class IrisNp2Options {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IrisNp2Options);

  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background.
  Note: This only serializes options that are YAML built-in types. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(sampled_iris_options));
  }

  IrisNp2Options() = default;

  /** The user can specify a solver to use for the counterexample search
   * program. If nullptr (the default value) is given, then
   * solvers::MakeFirstAvailableSolver will be used to pick the solver. */
  const solvers::SolverInterface* solver{nullptr};

  /** Options common to IRIS-type algorithms. */
  CommonSampledIrisOptions sampled_iris_options{};

  /** Parameterization of the subspace along which to grow the region. Default
   * is the identity parameterization, corresponding to growing regions in the
   * ordinary configuration space. */
  IrisParameterizationFunction parameterization{};
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

@throws if you set `options.parameterization`.
@throws if any collision pairs in `checker` have negative padding.
@throws if any collision geometries have been been added in `checker`.
*/

geometry::optimization::HPolyhedron IrisNp2(
    const SceneGraphCollisionChecker& checker,
    const geometry::optimization::Hyperellipsoid& starting_ellipsoid,
    const geometry::optimization::HPolyhedron& domain,
    const IrisNp2Options& options = IrisNp2Options());
}  // namespace planning
}  // namespace drake
