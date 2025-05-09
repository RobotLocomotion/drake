#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include <Eigen/Dense>

#include "drake/common/name_value.h"
#include "drake/common/parallelism.h"
#include "drake/geometry/meshcat.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace planning {

class CommonSampledIrisOptions {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CommonSampledIrisOptions);

  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background.
  Note: This only serializes options that are YAML built-in types. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(num_particles));
    a->Visit(DRAKE_NVP(tau));
    a->Visit(DRAKE_NVP(delta));
    a->Visit(DRAKE_NVP(epsilon));
    a->Visit(DRAKE_NVP(containment_points));
    a->Visit(DRAKE_NVP(max_iterations));
    a->Visit(DRAKE_NVP(max_iterations_separating_planes));
    a->Visit(DRAKE_NVP(max_separating_planes_per_iteration));
    a->Visit(DRAKE_NVP(verbose));
    a->Visit(DRAKE_NVP(require_sample_point_is_contained));
    a->Visit(DRAKE_NVP(configuration_space_margin));
    a->Visit(DRAKE_NVP(termination_threshold));
    a->Visit(DRAKE_NVP(relative_termination_threshold));
    a->Visit(DRAKE_NVP(random_seed));
    a->Visit(DRAKE_NVP(mixing_steps));
  }

  CommonSampledIrisOptions() = default;

  /** Number of particles used to estimate the closest collision. */
  int num_particles = 1e3;

  /** Decision threshold for the unadaptive test. Choosing a small value
   * increases both the cost and the power of the statistical test. Increasing
   * the value of `tau` makes running an individual test cheaper but decreases
   * its power to accept a polytope. We find choosing a value of 0.5 a good
   * trade-off. */
  double tau = 0.5;

  /** Upper bound on the probability the returned region has a
   * fraction-in-collision greater than `epsilon`. */
  double delta = 5e-2;

  /** Admissible fraction of the region volume allowed to be in collision. */
  double epsilon = 1e-2;

  /** Points that are guaranteed to be contained in the final region
   * provided their convex hull is collision free. Note that if the containment
   * points are closer than configuration_margin to an obstacle we will relax
   * the margin in favor of including the containment points. The matrix
   * `containment_points` is expected to be of the shape dimension times number
   * of points. IrisZo throws if the center of the starting ellipsoid is
   * not contained in the convex hull of these containment points. */
  std::optional<Eigen::MatrixXd> containment_points{std::nullopt};

  /** Maximum number of alternations between the ellipsoid and the separating
   * planes step (a.k.a. outer iterations). */
  int max_iterations{3};

  /** Maximum number of rounds of adding faces to the polytope per outer
   * iteration. */
  int max_iterations_separating_planes{20};

  /** Maximum number of faces to add per inner iteration. Setting the value to
   * -1 means there is no limit to the number of faces that can be added. */
  int max_separating_planes_per_iteration{10};

  /** Number of threads to use when updating the particles. If the user requests
   * more threads than the CollisionChecker supports, that number of threads
   * will be used instead. However, see also `parameterization_is_threadsafe`.
   */
  Parallelism parallelism{Parallelism::Max()};

  /** Enables print statements indicating the progress of IrisZo. */
  bool verbose{false};

  /** The initial polytope is guaranteed to contain the point if that point is
   * collision-free. */
  bool require_sample_point_is_contained{true};

  /** We retreat by this margin from each C-space obstacle in order to avoid the
   * possibility of requiring an infinite number of faces to approximate a
   * curved boundary. */
  double configuration_space_margin{1e-2};

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this threshold. This termination condition can
  be disabled by setting to a negative value. */
  double termination_threshold{2e-2};

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this percent of the previous best volume.
  This termination condition can be disabled by setting to a negative value. */
  double relative_termination_threshold{1e-3};

  /** This option sets the random seed for random sampling throughout the
   * algorithm. */
  int random_seed{1234};

  /** Number of mixing steps used for hit-and-run sampling. */
  int mixing_steps{50};

  /** Passing a meshcat instance may enable debugging visualizations when the
   * configuration space is <= 3 dimensional.*/
  std::shared_ptr<geometry::Meshcat> meshcat{};

  /** By default, IRIS-ZO only considers collision avoidance constraints. This
  option can be used to pass additional constraints that should be satisfied by
  the output region. We accept these in the form of a MathematicalProgram:

    find q subject to g(q) ≤ 0.

  The decision_variables() for the program are taken to define `q`. IRIS-ZO will
  silently ignore any costs in `prog_with_additional_constraints`. If any
  constraints are not threadsafe, then `parallelism` will be overridden, and
  only one thread will be used.
  @note If the user has specified a parameterization, then these constraints are
  imposed on the points in the parameterized space Q, not the configuration
  space C.
  @note Internally, these constraints are checked after collisions checking is
  performed. */
  const solvers::MathematicalProgram* prog_with_additional_constraints{};
};

class IrisParameterizationFunction {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IrisParameterizationFunction);

  typedef std::function<Eigen::VectorXd(const Eigen::VectorXd&)>
      ParameterizationFunction;

  /** Ordinarily, IRIS algorithms grow collision free regions in the robot's
   * configuration space C. This allows the user to specify a function f:Q→C ,
   * and grow the region in Q instead. The function should be a map R^m to
   * R^n, where n is the dimension of the plant configuration space, determined
   * via `checker.plant().num_positions()` and m is `parameterization_dimension`
   * if specified. The user must provide `parameterization`, which is the
   * function f, `parameterization_is_threadsafe`, which is whether or not
   * `parameterization` can be called concurrently, and
   * `parameterization_dimension`, the dimension of the input space Q. */
  IrisParameterizationFunction(const ParameterizationFunction& parameterization,
                               bool parameterization_is_threadsafe,
                               int parameterization_dimension)
      : parameterization_is_threadsafe_(parameterization_is_threadsafe),
        parameterization_dimension_(parameterization_dimension),
        parameterization_(parameterization) {}

  /** Default constructor -- returns the identity mapping, which is threadsafe
   * and compatible with any dimension configuration space. */
  IrisParameterizationFunction() = default;

  /** Alternative constructor that allows the user to define the
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
  IrisParameterizationFunction(
      const Eigen::VectorX<symbolic::Expression>& expression_parameterization,
      const Eigen::VectorX<symbolic::Variable>& variables);

  /** Constructs an instance of IrisParameterizationFunction that handles a
   * rational kinematic parameterization. Regions are grown in the `s`
   * variables, so as to minimize collisions in the `q` variables. See
   * RationalForwardKinematics for details.
   * @note The user is responsible for ensuring `kin` (and the underlying
   * MultibodyPlant it is built on) is kept alive. If that object is deleted,
   * then the parameterization can no longer be used. */
  IrisParameterizationFunction(
      const multibody::RationalForwardKinematics* kin,
      const Eigen::Ref<const Eigen::VectorXd>& q_star_val);

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

 private:
  bool parameterization_is_threadsafe_{true};

  std::optional<int> parameterization_dimension_{std::nullopt};

  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> parameterization_{
      [](const Eigen::VectorXd& q) -> Eigen::VectorXd {
        return q;
      }};
};

}  // namespace planning
}  // namespace drake
