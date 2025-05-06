#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/parallelism.h"
#include "drake/common/symbolic/expression.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/iris/iris_common.h"

namespace drake {
namespace planning {
/**
 * IrisZoOptions collects all parameters for the IRIS-ZO algorithm.
 *
 * @experimental
 * @see IrisZo for more details.
 **/
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

  /** Options pertaining to the sampling and temrination conditions. */
  CommonSampledIrisOptions sampled_iris_options{};

  /** Maximum number of bisection steps. */
  int bisection_steps{10};

  typedef std::function<Eigen::VectorXd(const Eigen::VectorXd&)>
      ParameterizationFunction;

  /** Ordinarily, IRIS-ZO grows collision free regions in the robot's
   * configuration space C. This allows the user to specify a function f:Q→C ,
   * and grow the region in Q instead. The function should be a map R^m to
   * R^n, where n is the dimension of the plant configuration space, determined
   * via `checker.plant().num_positions()` and m is `parameterization_dimension`
   * if specified. The user must provide `parameterization`, which is the
   * function f, `parameterization_is_threadsafe`, which is whether or not
   * `parametrization` can be called concurrently, and
   * `parameterization_dimension`, the dimension of the input space Q. */
  void set_parameterization(const ParameterizationFunction& parameterization,
                            bool parameterization_is_threadsafe,
                            int parameterization_dimension) {
    parameterization_ = parameterization;
    parameterization_is_threadsafe_ = parameterization_is_threadsafe;
    parameterization_dimension_ = parameterization_dimension;
  }

  /** Alternative to `set_parameterization` that allows the user to define the
   * parameterization using a `VectorX<Expression>`. The user must also provide
   * a vector containing the variables used in `expression_parameterization`, in
   * the order that they should be evaluated. Each `Variable` in `variables`
   * must be used, each `Variable` used in `expression_parameterization` must
   * appear in `variables`, and there must be no duplicates in `variables`.
   * @note Expression parameterizations are always threadsafe.
   * @throws if the number of variables used across
   * `expression_parameterization` does not match `ssize(variables)`.
   * @throws if any variables in `expression_parameterization` are not listed in
   * `variables`.
   * @throws if any variables in `variables` are not used anywhere in
   * `expression_parameterization`. */
  void SetParameterizationFromExpression(
      const Eigen::VectorX<symbolic::Expression>& expression_parameterization,
      const Eigen::VectorX<symbolic::Variable>& variables);

  /** Get the parameterization function.
   * @note If the user has not specified this with `set_parameterization()`,
   * then the default value of `parameterization_` is the identity function,
   * indicating that the regions should be grown in the full configuration space
   * (in the standard coordinate system). */
  const ParameterizationFunction& get_parameterization() const {
    return parameterization_;
  }

  /** Returns whether or not the user has specified the parameterization to be
   * threadsafe.
   * @note The default `parameterization_` is the identity function, which is
   * threadsafe. */
  bool get_parameterization_is_threadsafe() const {
    return parameterization_is_threadsafe_;
  }

  /** Returns what the user has specified as the input dimension for the
   * parameterization function, or std::nullopt if it has not been set. A
   * std::nullopt value indicates that
   * IrisZo should use the ambient configuration space dimension as the input
   * dimension to the parameterization. */
  std::optional<int> get_parameterization_dimension() const {
    return parameterization_dimension_;
  }

  /** Constructs an instance of IrisZoOptions that handles a rational kinematic
   * parameterization. Regions are grown in the `s` variables, so as to minimize
   * collisions in the `q` variables. See RationalForwardKinematics for details.
   * @note The user is responsible for ensuring `kin` (and the underlying
   * MultibodyPlant it is built on) is kept alive. If that object is deleted,
   * then the parametrization can no longer be used. */
  static IrisZoOptions CreateWithRationalKinematicParameterization(
      const multibody::RationalForwardKinematics* kin,
      const Eigen::Ref<const Eigen::VectorXd>& q_star_val);

 private:
  bool parameterization_is_threadsafe_{true};

  std::optional<int> parameterization_dimension_{std::nullopt};

  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> parameterization_{
      [](const Eigen::VectorXd& q) -> Eigen::VectorXd {
        return q;
      }};
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

Pr[λ(P\Cfree)/λ(P) > ε] ⋞ δ.

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
*/

geometry::optimization::HPolyhedron IrisZo(
    const CollisionChecker& checker,
    const geometry::optimization::Hyperellipsoid& starting_ellipsoid,
    const geometry::optimization::HPolyhedron& domain,
    const IrisZoOptions& options = IrisZoOptions());
}  // namespace planning
}  // namespace drake
