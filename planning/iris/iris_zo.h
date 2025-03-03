#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/parallelism.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/planning/collision_checker.h"

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
    a->Visit(DRAKE_NVP(num_particles));
    a->Visit(DRAKE_NVP(tau));
    a->Visit(DRAKE_NVP(delta));
    a->Visit(DRAKE_NVP(epsilon));
    a->Visit(DRAKE_NVP(containment_points));
    a->Visit(DRAKE_NVP(max_iterations));
    a->Visit(DRAKE_NVP(max_iterations_separating_planes));
    a->Visit(DRAKE_NVP(max_separating_planes_per_iteration));
    a->Visit(DRAKE_NVP(bisection_steps));
    a->Visit(DRAKE_NVP(verbose));
    a->Visit(DRAKE_NVP(require_sample_point_is_contained));
    a->Visit(DRAKE_NVP(configuration_space_margin));
    a->Visit(DRAKE_NVP(termination_threshold));
    a->Visit(DRAKE_NVP(relative_termination_threshold));
    a->Visit(DRAKE_NVP(random_seed));
    a->Visit(DRAKE_NVP(mixing_steps));
  }

  IrisZoOptions() = default;

  /** Number of particles used to estimate the closest collision. */
  int num_particles = 1e3;

  /** Decision threshold for the unadaptive test. Choosing a small value
   * increases both the cost and the power statistical test. Increasing the
   * value of `tau` makes running an individual test cheaper but decreases its
   * power to accept a polytope. We find choosing a value of 0.5 a good
   * trade-off.
   */
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
  int max_separating_planes_per_iteration{-1};

  /** Maximum number of bisection steps. */
  int bisection_steps{10};

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

  /** Passing a meshcat instance may enable debugging visualizations; this
  currently and when the
  configuration space is <= 3 dimensional.*/
  std::shared_ptr<geometry::Meshcat> meshcat{};

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

  /** Construct an instance of IrisZoOptions that handles a rational kinematic
   * parameterization. Regions are grown in the `s` variables, so as to minimize
   * collisions in the `q` variables. See RationalForwardKinematics for details.
   * @note The user is responsible for ensuring `kin` (and the underlying
   * MultibodyPlant it is built on) is kept alive. If that object is deleted,
   * then the parametrization can no longer be used. */
  static IrisZoOptions CreateWithRationalKinematicParameterization(
      const multibody::RationalForwardKinematics& kin,
      const Eigen::Ref<const Eigen::VectorXd>& q_star_val) {
    const int dimension = kin.plant().num_positions();
    DRAKE_DEMAND(dimension > 0);
    IrisZoOptions instance;

    auto evaluate_s_to_q = [&kin, &q_star_val](const Eigen::VectorXd& s_val) {
      return kin.ComputeQValue(s_val, q_star_val);
    };

    instance.set_parameterization(evaluate_s_to_q,
                                  /* parameterization_is_threadsafe */ true,
                                  /* parameterization_dimension */ dimension);
    return instance;
  }

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
*/

geometry::optimization::HPolyhedron IrisZo(
    const CollisionChecker& checker,
    const geometry::optimization::Hyperellipsoid& starting_ellipsoid,
    const geometry::optimization::HPolyhedron& domain,
    const IrisZoOptions& options = IrisZoOptions());
}  // namespace planning
}  // namespace drake
