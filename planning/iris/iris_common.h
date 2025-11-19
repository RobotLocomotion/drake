#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/name_value.h"
#include "drake/common/parallelism.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/planning/collision_checker.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {

/** Various options which are common to the sampling-based algorithms IrisNp2
 * and IrisZo for generating collision free polytopes in configuration space.
 *
 * @ingroup planning_iris */
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
    a->Visit(DRAKE_NVP(remove_all_collisions_possible));
    a->Visit(DRAKE_NVP(random_seed));
    a->Visit(DRAKE_NVP(mixing_steps));
    a->Visit(DRAKE_NVP(sample_particles_in_parallel));
  }

  CommonSampledIrisOptions() = default;

  /** Minimum number of particles drawn per inner iteration. Some or all of
   * these particles, depending on the other algorithm settings, will be used to
   * find the closest collisions. */
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

  /** Suppose stepping back by configuration_space_margin would cut off the seed
   * point. If `relax_margin` is false, we throw an error, and if `relax_margin`
   * is true, we repeatedly divide configuration_space_margin by two (for that
   * hyperplane only) until the seed point is not cut off. Ignored if the user
   * has provided `containment_points`. */
  bool relax_margin{false};

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this threshold. This termination condition can
  be disabled by setting to a negative value. */
  double termination_threshold{2e-2};

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this percent of the previous best volume.
  This termination condition can be disabled by setting to a negative value. */
  double relative_termination_threshold{1e-3};

  /** A region may satisfy the user-requested fraction in-collision, but still
   * have some of the samples drawn be in-collision. If this flag is true, those
   * samples will also be used to produce hyperplanes. This produces slightly
   * smaller regions with more faces, but the region will beat the
   * user-requested fraction in-collision by a larger margin. */
  bool remove_all_collisions_possible{true};

  /** This option sets the random seed for random sampling throughout the
   * algorithm. */
  int random_seed{1234};

  /** Number of mixing steps used for hit-and-run sampling. */
  int mixing_steps{50};

  /** If true, the hit-and-run procedure is run in parallel to quickly draw all
   * the samples necessary. When the statistical test requires many samples
   * (e.g. due to constructing regions with a very low fraction in collision
   * with very high probability), the process of drawing the samples may become
   * a major time cost. Drawing the samples in parallel can lead to a major
   * speedup, at the cost of a sampling from a distribution that's slightly
   * further from a uniform distribution (due to the lower cumulative mixing
   * time). */
  bool sample_particles_in_parallel{false};

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

/** Ordinarily, IRIS algorithms grow collision free regions in the robot's
 * configuration space C. This allows the user to specify a function f:Q→C , and
 * grow the region in Q instead. The function should be a map R^m to R^n, where
 * n is the dimension of the plant configuration space and m is the input
 * dimension, if specified. If the user provides a version of the function for
 * Eigen::VectorX<double>, then the parameterization can be used with IrisZo.
 * IrisNp2 requires that the user also provies a version of the function for
 * Eigen::VectorX<AutoDiffXd>. If not specified, the input dimension is assumed
 * to be equal to the output dimension. The user must also specify whether or
 * not the parameterization function can be called in parallel.
 *
 * @ingroup planning_iris */
class IrisParameterizationFunction {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IrisParameterizationFunction);

  typedef std::function<Eigen::VectorXd(const Eigen::VectorXd&)>
      ParameterizationFunctionDouble;
  typedef std::function<Eigen::VectorX<AutoDiffXd>(
      const Eigen::VectorX<AutoDiffXd>&)>
      ParameterizationFunctionAutodiff;

  /** Constructor for when the user only provides a version of the
   * parameterization function for Eigen::VectorX<double>.
   * `parameterization_double` is the function itself,
   * `parameterization_is_threadsafe` specifies whether or not its threadsafe,
   * and `parameterization_dimension` is the input dimension. */
  IrisParameterizationFunction(
      const ParameterizationFunctionDouble& parameterization_double,
      bool parameterization_is_threadsafe, int parameterization_dimension)
      : parameterization_is_threadsafe_(parameterization_is_threadsafe),
        parameterization_dimension_(parameterization_dimension),
        parameterization_double_(parameterization_double),
        parameterization_autodiff_() {}

  /** Constructor for when the user only provides both versions of the
   * parameterization function.
   * `parameterization_double` is the version for Eigen::VectorX<double>,
   * `parameterization_autodiff_` is the version for Eigen::VectorX<AutoDiffXd>,
   * `parameterization_is_threadsafe` specifies whether or not its threadsafe,
   * and `parameterization_dimension` is the input dimension. */
  IrisParameterizationFunction(
      const ParameterizationFunctionDouble& parameterization_double,
      const ParameterizationFunctionAutodiff& parameterization_autodiff,
      bool parameterization_is_threadsafe, int parameterization_dimension)
      : parameterization_is_threadsafe_(parameterization_is_threadsafe),
        parameterization_dimension_(parameterization_dimension),
        parameterization_double_(parameterization_double),
        parameterization_autodiff_(parameterization_autodiff) {}

  /** Default constructor -- returns the identity mapping, which is threadsafe
   * and compatible with any dimension configuration space. */
  IrisParameterizationFunction() = default;

  /** Alternative constructor that allows the user to define the
   * parameterization using a `VectorX<Expression>`. The user must also provide
   * a vector containing the variables used in `expression_parameterization`, in
   * the order that they should be evaluated. Each `Variable` in `variables`
   * must be used, each `Variable` used in `expression_parameterization` must
   * appear in `variables`, and there must be no duplicates in `variables`.
   * @note This constructor only populates the VectorX<double> parameterization.
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
   * @note This constructor populates the VectorX<double> and
   * VectorX<AutoDiffXd> parameterizations.
   * @note The user is responsible for ensuring `kin` (and the underlying
   * MultibodyPlant it is built on) is kept alive. If that object is deleted,
   * then the parameterization can no longer be used. */
  IrisParameterizationFunction(
      const multibody::RationalForwardKinematics* kin,
      const Eigen::Ref<const Eigen::VectorXd>& q_star_val);

  /** Get the Eigen::VectorX<double> parameterization function.
   * @note If the user has not specified this with `set_parameterization()`,
   * then the default value of `parameterization_double_` is the identity
   * function, indicating that the regions should be grown in the full
   * configuration space (in the standard coordinate system). */
  const ParameterizationFunctionDouble& get_parameterization_double() const {
    return parameterization_double_;
  }

  /** Get the Eigen::VectorX<AutoDiffXd> parameterization function.
   * @note If the user has not specified this with `set_parameterization()`,
   * then the default value of `parameterization_double_` is the identity
   * function, indicating that the regions should be grown in the full
   * configuration space (in the standard coordinate system).
   * @throws If the user has specified the VectorX<double> parameterization but
   * not the VectorX<AutoDiffXd> parameterization. */
  const ParameterizationFunctionAutodiff& get_parameterization_autodiff()
      const {
    return parameterization_autodiff_;
  }

  /** Returns whether or not the user has specified the parameterization to be
   * threadsafe.
   * @note The default parameterization is the identity function, which is
   * threadsafe. */
  bool get_parameterization_is_threadsafe() const {
    return parameterization_is_threadsafe_;
  }

  /** Returns what the user has specified as the input dimension for the
   * parameterization function, or std::nullopt if it has not been set. A
   * std::nullopt value indicates that the chosen IRIS algorithm should use the
   * ambient configuration space dimension as the input dimension to the
   * parameterization. */
  std::optional<int> get_parameterization_dimension() const {
    return parameterization_dimension_;
  }

 private:
  bool parameterization_is_threadsafe_{true};
  std::optional<int> parameterization_dimension_{std::nullopt};
  ParameterizationFunctionDouble parameterization_double_{
      [](const Eigen::VectorXd& q) -> Eigen::VectorXd {
        return q;
      }};
  ParameterizationFunctionAutodiff parameterization_autodiff_{
      [](const Eigen::VectorX<AutoDiffXd>& q) -> Eigen::VectorX<AutoDiffXd> {
        return q;
      }};
};

namespace internal {

// See Definition 1 from the paper [Werner et al., 2024].
int unadaptive_test_samples(double epsilon, double delta, double tau);

float calc_delta_min(double delta, int max_iterations);

void AddTangentToPolytope(
    const geometry::optimization::Hyperellipsoid& E,
    const Eigen::Ref<const Eigen::VectorXd>& point,
    double configuration_space_margin, bool relax_margin,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>* A,
    Eigen::VectorXd* b, int* num_constraints);

void AddTangentToPolytope(
    const geometry::optimization::Hyperellipsoid& E,
    const Eigen::Ref<const Eigen::VectorXd>& point,
    const geometry::optimization::VPolytope& cvxh_vpoly,
    const solvers::SolverInterface& solver, double configuration_space_margin,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>* A,
    Eigen::VectorXd* b, int* num_constraints, double* max_relaxation);

// Given a pointer to a MathematicalProgram and a single particle, check
// whether the particle satisfies the constraints. If a nullptr is given for
// the program, return true, since the constraints are trivially-satisfied.
bool CheckProgConstraints(const solvers::MathematicalProgram* prog_ptr,
                          const Eigen::VectorXd& particle, const double tol);

// Given a pointer to a MathematicalProgram and a list of particles (where each
// particle is a choice of values for its decision variables), check in parallel
// which particles satisfy all constraints, and which don't. Each entry in the
// output vector corresponds to the corresponding particle. 1 means it satisfies
// the constraints, 0 means it doesn't. If a nullptr is given for the program,
// return a vector of all 1s, since all particles trivially satisfy the
// constraints. The user can specify a slice of particles to check [0,
// end_index). Default behavior is to check all particles -- when end_index is
// std::nullopt, it is set to ssize(particles).
std::vector<uint8_t> CheckProgConstraintsParallel(
    const solvers::MathematicalProgram* prog_ptr,
    const std::vector<Eigen::VectorXd>& particles,
    const Parallelism& parallelism, const double tol,
    std::optional<int> end_index = std::nullopt);

geometry::optimization::VPolytope ParseAndCheckContainmentPoints(
    const CollisionChecker& checker,
    const CommonSampledIrisOptions& sampled_iris_options,
    const IrisParameterizationFunction& parameterization,
    const geometry::optimization::Hyperellipsoid& starting_ellipsoid,
    const double constraints_tol = 1e-6);

Eigen::VectorXd ComputeFaceTangentToDistCvxh(
    const geometry::optimization::Hyperellipsoid& E,
    const Eigen::Ref<const Eigen::VectorXd>& point,
    const geometry::optimization::VPolytope& cvxh_vpoly,
    const solvers::SolverInterface& solver);

// Populates particles with random samples, drawn across potentially multiple
// threads. number_to_sample must be smaller than ssize(particles), and
// ssize(generators) threads will be used to draw samples.
// @param[out] particles is an output-only argument, which must be preallocated
// to size at least number_to_sample. The first number_to_sample elements will
// be overwritten, and remaining elements are undefined.
void PopulateParticlesByUniformSampling(
    const geometry::optimization::HPolyhedron& P, int number_to_sample,
    int mixing_steps, std::vector<RandomGenerator>* generators,
    std::vector<Eigen::VectorXd>* particles);

}  // namespace internal
}  // namespace planning
}  // namespace drake
