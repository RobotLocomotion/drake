#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <utility>

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/iris/iris_common.h"

namespace drake {
namespace planning {
/**
 * IrisZoOptions collects all parameters for the IRIS-ZO algorithm.
 *
 * @experimental
 * @see IrisZo for more details.
 *
 * @ingroup planning_iris */
class IrisZoOptions {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IrisZoOptions);

  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background.
  Note: This only serializes options that are YAML built-in types. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(sampled_iris_options));
    a->Visit(DRAKE_NVP(bisection_steps));
  }

  IrisZoOptions() = default;

  /** Options pertaining to the sampling and termination conditions. */
  CommonSampledIrisOptions sampled_iris_options{};

  /** Maximum number of bisection steps. */
  int bisection_steps{10};

  /** Parameterization of the subspace along which to grow the region. Default
   * is the identity parameterization, corresponding to growing regions in the
   * ordinary configuration space. */
  IrisParameterizationFunction parameterization{};
};

/** The IRIS-ZO (Iterative Regional Inflation by Semidefinite programming - Zero
Order) algorithm, as described in

P. Werner, T. Cohn\*, R. H. Jiang\*, T. Seyde, M. Simchowitz, R. Tedrake, and D.
Rus, "Faster Algorithms for Growing Collision-Free Convex Polytopes in Robot
Configuration Space,"
&nbsp;* Denotes equal contribution.

https://groups.csail.mit.edu/robotics-center/public_papers/Werner24.pdf

This algorithm constructs probabilistically collision-free polytopes in robot
configuration space while only relying on a collision checker. The sets are
constructed using a simple parallel zero-order optimization strategy. The
produced polytope P is probabilistically collision-free in the sense that one
gets to control the probability δ that the fraction of the volume-in-collision
is larger than ε

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

@ingroup planning_iris */

geometry::optimization::HPolyhedron IrisZo(
    const CollisionChecker& checker,
    const geometry::optimization::Hyperellipsoid& starting_ellipsoid,
    const geometry::optimization::HPolyhedron& domain,
    const IrisZoOptions& options = IrisZoOptions());
}  // namespace planning
}  // namespace drake
