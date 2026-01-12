#include "drake/planning/iris/iris_np2.h"

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/common/text_logging.h"
#include "drake/geometry/optimization/affine_ball.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/iris_internal.h"
#include "drake/geometry/optimization/minkowski_sum.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForIndexLoop;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::FrameId;
using geometry::GeometryId;
using geometry::QueryObject;
using geometry::Role;
using geometry::SceneGraphInspector;
using geometry::Sphere;
using geometry::optimization::ConvexSet;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::internal::ClosestCollisionProgram;
using geometry::optimization::internal::CounterexampleConstraint;
using geometry::optimization::internal::CounterexampleProgram;
using geometry::optimization::internal::GeometryPairWithDistance;
using geometry::optimization::internal::IrisConvexSetMaker;
using geometry::optimization::internal::
    ParameterizedPointsBoundedDistanceConstraint;
using geometry::optimization::internal::ParameterizedSamePointConstraint;
using geometry::optimization::internal::PointsBoundedDistanceConstraint;
using geometry::optimization::internal::SamePointConstraint;
using internal::IrisNp2SamplingStrategy;
using math::RigidTransform;
using multibody::MultibodyPlant;
using solvers::Binding;
using solvers::Constraint;
using solvers::SolverInterface;
using systems::Context;

namespace {

double GetPaddingBetweenGeometries(const CollisionChecker& checker,
                                   const SceneGraphInspector<double>& inspector,
                                   const GeometryId geom_A,
                                   const GeometryId geom_B) {
  const FrameId frame_A = inspector.GetFrameId(geom_A);
  const FrameId frame_B = inspector.GetFrameId(geom_B);
  const multibody::RigidBody<double>* body_A =
      checker.plant().GetBodyFromFrameId(frame_A);
  const multibody::RigidBody<double>* body_B =
      checker.plant().GetBodyFromFrameId(frame_B);
  DRAKE_THROW_UNLESS(body_A != nullptr);
  DRAKE_THROW_UNLESS(body_B != nullptr);
  return checker.GetPaddingBetween(*body_A, *body_B);
}

/* Given a context whose position represents a configuration known to be in
 * collision, find a pair of collision geometries which are in collision and
 * return the corresponding index in sorted_pairs. */
int FindCollisionPairIndex(
    const CollisionChecker& checker,
    const SceneGraphInspector<double>& inspector,
    const Context<double>& context,
    const std::vector<GeometryPairWithDistance>& sorted_pairs) {
  int pair_in_collision = -1;
  int i_pair = 0;
  auto query_object = checker.plant()
                          .get_geometry_query_input_port()
                          .template Eval<QueryObject<double>>(context);
  for (const auto& pair : sorted_pairs) {
    const double distance =
        query_object
            .ComputeSignedDistancePairClosestPoints(pair.geomA, pair.geomB)
            .distance;
    const double padding =
        GetPaddingBetweenGeometries(checker, inspector, pair.geomA, pair.geomB);
    if (distance < padding) {
      pair_in_collision = i_pair;
      break;
    }
    ++i_pair;
  }

  return pair_in_collision;
}

/* Check if any unsupported features have been used, as well as any other
 * initial conditions that must be satisfied by the user inputs. */
void CheckInitialConditions(const SceneGraphCollisionChecker& checker,
                            const Hyperellipsoid& starting_ellipsoid,
                            const HPolyhedron& domain,
                            const IrisNp2Options& options) {
  if (!checker.GetAllAddedCollisionShapes().empty()) {
    // TODO(cohnt): Support handling additional collision shapes.
    throw std::runtime_error(
        "IrisNp2 does not yet support collision checkers with added collision "
        "shapes.");
  }
  if (checker.GetPaddingMatrix().minCoeff() < 0.0) {
    throw std::runtime_error(
        "IrisNp2 does not support collision checkers with negative padding.");
  }

  if (!options.parameterization.get_parameterization_autodiff()) {
    throw std::runtime_error(
        "IrisNp2 requires an autodiff-compatible parameterization. Make sure "
        "to provide the IrisParameterizationFunction with a "
        "parameterization_double and parameterization_autodiff, or consider "
        "using IrisZo instead.");
  }

  // The input domain must be bounded.
  DRAKE_THROW_UNLESS(domain.IsBounded());
  DRAKE_THROW_UNLESS(
      starting_ellipsoid.center().size() ==
      options.parameterization.get_parameterization_dimension().value_or(
          checker.plant().num_positions()));

  // Do a basic check of the parameterization dimension.
  const Eigen::VectorXd starting_ellipsoid_center_ambient =
      options.parameterization.get_parameterization_double()(
          starting_ellipsoid.center());
  const int computed_ambient_dimension =
      starting_ellipsoid_center_ambient.size();
  if (computed_ambient_dimension != checker.plant().num_positions()) {
    throw std::runtime_error(fmt::format(
        "The plant has {} positions, but the given parameterization "
        "returned a point with the wrong dimension (its size was "
        "{}) when called on {}.",
        checker.plant().num_positions(), computed_ambient_dimension,
        fmt_eigen(starting_ellipsoid.center().transpose())));
  }

  // Check ray search parameters.
  DRAKE_THROW_UNLESS(options.ray_sampler_options.ray_search_num_steps >= 1);
  DRAKE_THROW_UNLESS(
      options.ray_sampler_options.num_particles_to_walk_towards >= 1);
}

/* Check for certain conditions at the end of the separating hyperplanes step,
 * which, if satisfied, indicate that the algorithm should terminate. */
bool CheckTerminationConditions(int iteration_num, double delta_volume,
                                double last_iteration_volume,
                                const IrisNp2Options& options) {
  bool terminate = false;
  // Maximum iteration count.
  if (iteration_num >= options.sampled_iris_options.max_iterations) {
    if (options.sampled_iris_options.verbose) {
      log()->info(
          "IrisNp2: Terminating because the iteration limit "
          "{} has been reached.",
          options.sampled_iris_options.max_iterations);
    }
    terminate = true;
  }
  // Absolute volume change threshold.
  if (delta_volume <= options.sampled_iris_options.termination_threshold) {
    if (options.sampled_iris_options.verbose) {
      log()->info(
          "IrisNp2: Terminating because the hyperellipsoid "
          "volume change {} is below the threshold {}.",
          delta_volume, options.sampled_iris_options.termination_threshold);
    }
    terminate = true;
  }
  // Relative volume change threshold.
  if (delta_volume / last_iteration_volume <=
      options.sampled_iris_options.relative_termination_threshold) {
    if (options.sampled_iris_options.verbose) {
      log()->info(
          "IrisNp2: Terminating because the hyperellipsoid "
          "relative volume change {} is below the threshold {}.",
          delta_volume / last_iteration_volume,
          options.sampled_iris_options.relative_termination_threshold);
    }
    terminate = true;
  }
  return terminate;
}

/* This takes a particle (which may or may not be in collision) and performs
 * ray stepping procedure, starting at the center of the ellipsoid, and stepping
 * toward the particle until a collision (or constraint violation) is found, or
 * the boundary of the polytope is reached. Returns true if the sampler found a
 * collision or constraint violation, and then the resulting configuration will
 * be stored into the output argument `particle`. If no collision or constraint
 * violation is found, returns false, and no change is made to `particle`. */
bool RaySamplerProcess(const SceneGraphCollisionChecker& checker,
                       const VectorXd& ellipsoid_center,
                       const HPolyhedron& current_polytope,
                       const IrisNp2Options& options, int constraints_tol,
                       VectorXd* particle) {
  DRAKE_THROW_UNLESS(particle != nullptr);

  int chunk_size = options.sampled_iris_options.parallelism.num_threads();
  if (options.sampled_iris_options.prog_with_additional_constraints &&
      !options.sampled_iris_options.prog_with_additional_constraints
           ->IsThreadSafe()) {
    chunk_size = 1;
  }

  std::vector<VectorXd> candidate_particles;
  candidate_particles.reserve(chunk_size);

  double particle_distance = (*particle - ellipsoid_center).norm();
  double step_size =
      particle_distance / options.ray_sampler_options.ray_search_num_steps;
  VectorXd particle_step =
      (*particle - ellipsoid_center) * step_size / particle_distance;

  // We start at the center of the ellipsoid, and step outwards towards
  // particle, stopping when we hit the boundary of the polytope or a sample in
  // collision (or violating a constraint). For efficiency reasons, we check in
  // batches whose size matches the number of threads, to maximally exploit
  // parallelism, since configurations can be checked for collisions and
  // constraint violations in parallel.
  Eigen::VectorXd chunk_start_configuration = ellipsoid_center;
  while (
      current_polytope.PointInSet(chunk_start_configuration + particle_step)) {
    candidate_particles.clear();
    candidate_particles.push_back(chunk_start_configuration + particle_step);
    for (int i = 1; i < chunk_size; ++i) {
      if (!current_polytope.PointInSet(candidate_particles.back() +
                                       particle_step)) {
        break;
      } else {
        candidate_particles.push_back(candidate_particles.back() +
                                      particle_step);
      }
    }
    chunk_start_configuration = candidate_particles.back();

    // Transform via the parameterization.
    std::vector<Eigen::VectorXd> ambient_particles;
    for (const auto& candidate_particle : candidate_particles) {
      ambient_particles.push_back(
          options.parameterization.get_parameterization_double()(
              candidate_particle));
    }

    // Check for collisions.
    std::vector<uint8_t> particle_collision_free =
        checker.CheckConfigsCollisionFree(
            ambient_particles, options.sampled_iris_options.parallelism);

    // Check for additional constraint violations.
    std::vector<uint8_t> particle_satisfies_additional_constraints =
        internal::CheckProgConstraintsParallel(
            options.sampled_iris_options.prog_with_additional_constraints,
            candidate_particles, chunk_size, constraints_tol);

    for (int i = 0; i < ssize(ambient_particles); ++i) {
      if (!particle_collision_free[i] ||
          !particle_satisfies_additional_constraints[i]) {
        *particle = candidate_particles[i];
        return true;
      }
    }
  }
  // We never found a collision or constraint violation.
  return false;
}

}  // namespace

namespace internal {

std::ostream& operator<<(std::ostream& out, const IrisNp2SamplingStrategy& t) {
  switch (t) {
    case IrisNp2SamplingStrategy::kGreedySampler:
      out << "greedy";
      break;
    case IrisNp2SamplingStrategy::kRaySampler:
      out << "ray";
      break;
  }
  return out;
}

IrisNp2SamplingStrategy iris_np2_sampling_strategy_from_string(
    const std::string& spec) {
  if (spec == "greedy") {
    return IrisNp2SamplingStrategy::kGreedySampler;
  } else if (spec == "ray") {
    return IrisNp2SamplingStrategy::kRaySampler;
  } else {
    throw std::runtime_error(fmt::format(
        "Specified invalid IrisNp2 sampling strategy type: '{}'.", spec));
  }
}

}  // namespace internal

HPolyhedron IrisNp2(const SceneGraphCollisionChecker& checker,
                    const Hyperellipsoid& starting_ellipsoid,
                    const HPolyhedron& domain, const IrisNp2Options& options) {
  auto start = std::chrono::high_resolution_clock::now();

  const bool additional_constraints_threadsafe =
      options.sampled_iris_options.prog_with_additional_constraints
          ? options.sampled_iris_options.prog_with_additional_constraints
                ->IsThreadSafe()
          : true;

  CheckInitialConditions(checker, starting_ellipsoid, domain, options);

  IrisNp2SamplingStrategy sampling_strategy =
      internal::iris_np2_sampling_strategy_from_string(
          options.sampling_strategy);

  const auto& plant = checker.plant();
  const int nq = plant.num_positions();

  const auto& context = checker.UpdatePositions(
      options.parameterization.get_parameterization_double()(
          starting_ellipsoid.center()));
  plant.ValidateContext(context);

  const Eigen::VectorXd seed = starting_ellipsoid.center();

  HPolyhedron P(domain);
  Hyperellipsoid E = starting_ellipsoid;

  // Make all of the convex sets and supporting quantities.
  auto query_object =
      plant.get_geometry_query_input_port().Eval<QueryObject<double>>(context);
  const SceneGraphInspector<double>& inspector = query_object.inspector();
  IrisConvexSetMaker maker(query_object, inspector.world_frame_id());
  std::unordered_map<GeometryId, copyable_unique_ptr<ConvexSet>> sets{};
  std::unordered_map<GeometryId, const multibody::Frame<double>*> frames{};
  const std::vector<GeometryId> geom_ids =
      inspector.GetAllGeometryIds(Role::kProximity);
  copyable_unique_ptr<ConvexSet> temp_set;
  for (GeometryId geom_id : geom_ids) {
    // Make all sets in the local geometry frame.
    FrameId frame_id = inspector.GetFrameId(geom_id);
    maker.set_reference_frame(frame_id);
    maker.set_geometry_id(geom_id);
    inspector.GetShape(geom_id).Reify(&maker, &temp_set);
    sets.emplace(geom_id, std::move(temp_set));
    frames.emplace(geom_id, &plant.GetBodyFromFrameId(frame_id)->body_frame());
  }

  std::set<std::pair<GeometryId, GeometryId>> pairs =
      inspector.GetCollisionCandidates();
  const int n_collision_pairs = static_cast<int>(pairs.size());
  const int parameterization_dimension =
      options.parameterization.get_parameterization_dimension().value_or(
          checker.plant().num_positions());
  auto same_point_constraint =
      std::make_shared<ParameterizedSamePointConstraint>(
          &plant, context,
          options.parameterization.get_parameterization_double(),
          options.parameterization.get_parameterization_autodiff(),
          parameterization_dimension);
  auto points_bounded_distance_constraint =
      std::make_shared<ParameterizedPointsBoundedDistanceConstraint>(
          &plant, context, 0.0,
          options.parameterization.get_parameterization_double(),
          options.parameterization.get_parameterization_autodiff(),
          parameterization_dimension);
  std::map<std::pair<GeometryId, GeometryId>, std::vector<VectorXd>>
      counter_examples;

  // As a surrogate for the true objective, the pairs are sorted by the
  // distance between each collision pair from the seed point configuration.
  // This could improve computation times and produce regions with fewer
  // faces.
  std::vector<GeometryPairWithDistance> sorted_pairs;
  for (const auto& [geomA, geomB] : pairs) {
    const double distance =
        query_object.ComputeSignedDistancePairClosestPoints(geomA, geomB)
            .distance;
    if (distance < 0.0) {
      throw std::runtime_error(fmt::format(
          "Starting ellipsoid center {} is in collision; geometry {} is in "
          "collision with geometry {}",
          fmt_eigen(E.center().transpose()), inspector.GetName(geomA),
          inspector.GetName(geomB)));
    }
    sorted_pairs.emplace_back(geomA, geomB, distance);
  }
  std::sort(sorted_pairs.begin(), sorted_pairs.end());

  if (options.sampled_iris_options.prog_with_additional_constraints) {
    DRAKE_THROW_UNLESS(options.sampled_iris_options
                           .prog_with_additional_constraints->num_vars() ==
                       parameterization_dimension);
  }
  // TODO(cohnt): Allow users to set this parameter if it ever becomes needed.
  const double constraints_tol = 1e-6;
  if (!internal::CheckProgConstraints(
          options.sampled_iris_options.prog_with_additional_constraints,
          E.center(), constraints_tol)) {
    throw std::runtime_error(fmt::format(
        "Starting ellipsoid center {} violates a constraint in "
        "options.sampled_iris_options.prog_with_additional_constraints.",
        fmt_eigen(E.center().transpose())));
  }
  geometry::optimization::VPolytope containment_points_vpolytope =
      internal::ParseAndCheckContainmentPoints(
          checker, options.sampled_iris_options, options.parameterization,
          starting_ellipsoid);

  // On each iteration, we will build the collision-free polytope represented
  // as {x | A * x <= b}.  Here we pre-allocate matrices with a generous
  // maximum size.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      P.A().rows() + 2 * n_collision_pairs, parameterization_dimension);
  VectorXd b(P.A().rows() + 2 * n_collision_pairs);
  A.topRows(P.A().rows()) = P.A();
  b.head(P.A().rows()) = P.b();
  int num_initial_constraints = P.A().rows();

  // Make the additional constraint counterexample programs (if applicable).
  std::shared_ptr<CounterexampleConstraint> counter_example_constraint{};
  std::unique_ptr<CounterexampleProgram> counter_example_prog{};
  std::vector<Binding<Constraint>> additional_constraint_bindings{};
  if (options.sampled_iris_options.prog_with_additional_constraints) {
    counter_example_constraint = std::make_shared<CounterexampleConstraint>(
        options.sampled_iris_options.prog_with_additional_constraints);
    additional_constraint_bindings =
        options.sampled_iris_options.prog_with_additional_constraints
            ->GetAllConstraints();

    // Handle bounding box and linear constraints as a special case
    // (extracting them from the additional_constraint_bindings).
    auto AddConstraint = [&](const Eigen::MatrixXd& new_A,
                             const Eigen::VectorXd& new_b,
                             const solvers::VectorXDecisionVariable& vars) {
      while (num_initial_constraints + new_A.rows() >= A.rows()) {
        // Increase pre-allocated polytope size.
        A.conservativeResize(A.rows() * 2, A.cols());
        b.conservativeResize(b.rows() * 2);
      }
      for (int i = 0; i < new_b.rows(); ++i) {
        if (!std::isinf(new_b[i])) {
          A.row(num_initial_constraints).setZero();
          for (int j = 0; j < vars.rows(); ++j) {
            const int index =
                options.sampled_iris_options.prog_with_additional_constraints
                    ->FindDecisionVariableIndex(vars[j]);
            A(num_initial_constraints, index) = new_A(i, j);
          }
          b[num_initial_constraints++] = new_b[i];
        }
      }
    };
    auto HandleLinearConstraints = [&](const auto& bindings) {
      for (const auto& binding : bindings) {
        AddConstraint(binding.evaluator()->get_sparse_A(),
                      binding.evaluator()->upper_bound(), binding.variables());
        AddConstraint(-binding.evaluator()->get_sparse_A(),
                      -binding.evaluator()->lower_bound(), binding.variables());
        auto pos = std::find(additional_constraint_bindings.begin(),
                             additional_constraint_bindings.end(), binding);
        DRAKE_ASSERT(pos != additional_constraint_bindings.end());
        additional_constraint_bindings.erase(pos);
      }
    };
    HandleLinearConstraints(
        options.sampled_iris_options.prog_with_additional_constraints
            ->bounding_box_constraints());
    HandleLinearConstraints(
        options.sampled_iris_options.prog_with_additional_constraints
            ->linear_constraints());
    counter_example_prog = std::make_unique<CounterexampleProgram>(
        counter_example_constraint, E, A.topRows(num_initial_constraints),
        b.head(num_initial_constraints));

    P = HPolyhedron(A.topRows(num_initial_constraints),
                    b.head(num_initial_constraints));
  }

  DRAKE_THROW_UNLESS(P.PointInSet(seed, 1e-12));

  double last_iteration_volume = E.Volume();
  int iteration = 0;
  VectorXd closest(parameterization_dimension);

  const int num_threads_for_sampling =
      options.sampled_iris_options.sample_particles_in_parallel
          ? options.sampled_iris_options.parallelism.num_threads()
          : 1;
  std::vector<RandomGenerator> generators;
  // This strategy for seeding multiple generators is acceptable, since Drake's
  // RandomGenerator is a Mersenne Twister, where even nearby seeds are
  // practically independent.
  for (int generator_index = 0; generator_index < num_threads_for_sampling;
       ++generator_index) {
    generators.push_back(RandomGenerator(
        options.sampled_iris_options.random_seed + generator_index));
  }

  const SolverInterface* solver;
  std::unique_ptr<SolverInterface> default_solver =
      solvers::MakeFirstAvailableSolver(
          {solvers::SnoptSolver::id(), solvers::IpoptSolver::id()});
  if (options.solver) {
    solver = options.solver;
  } else {
    solver = default_solver.get();
  }

  // For debugging visualization.
  Vector3d point_to_draw = Vector3d::Zero();
  int num_points_drawn = 0;
  bool do_debugging_visualization =
      options.sampled_iris_options.meshcat && nq <= 3;

  const std::string seed_point_msg =
      "IrisNp2: terminating iterations because the seed point is no longer in "
      "the region.";

  // Track maximum relaxation of cspace margin if containment_points are
  // requested.
  double max_relaxation = 0;

  // Set up constants for statistical tests.
  double outer_delta_min =
      internal::calc_delta_min(options.sampled_iris_options.delta,
                               options.sampled_iris_options.max_iterations);

  double delta_min = internal::calc_delta_min(
      outer_delta_min,
      options.sampled_iris_options.max_iterations_separating_planes);

  int N_max = internal::unadaptive_test_samples(
      options.sampled_iris_options.epsilon, delta_min,
      options.sampled_iris_options.tau);

  if (options.sampled_iris_options.verbose) {
    log()->info(
        "IrisNp2 finding region that is {} collision free with {} certainty ",
        1 - options.sampled_iris_options.epsilon,
        1 - options.sampled_iris_options.delta);
    log()->info("IrisNp2 worst case test requires {} samples.", N_max);
  }

  // TODO(cohnt): Do an argsort so we don't have to have two separate copies.
  // TODO(cohnt): Switch to using a single large MatrixXd to avoid repeated
  // VectorXd heap allocations.
  std::vector<Eigen::VectorXd> particles;
  std::vector<Eigen::VectorXd> particles_in_collision;
  particles.reserve(N_max);
  particles_in_collision.reserve(N_max);

  while (true) {
    log()->info("IrisNp2 iteration {}", iteration);
    int num_constraints = num_initial_constraints;
    HPolyhedron P_candidate = HPolyhedron(A.topRows(num_initial_constraints),
                                          b.head(num_initial_constraints));
    DRAKE_ASSERT(last_iteration_volume > 0);

    // Find separating hyperplanes.
    int num_iterations_separating_planes = 0;

    double outer_delta = options.sampled_iris_options.delta * 6 /
                         (M_PI * M_PI * (iteration + 1) * (iteration + 1));

    // There is no need for decaying outer delta if we are guaranteed to
    // terminate after one step. In this case we can be less conservative and
    // set it to our total accepted error probability.
    if (options.sampled_iris_options.max_iterations == 1) {
      outer_delta = options.sampled_iris_options.delta;
    }

    // TODO(cohnt): Rewrite as a for loop for better readability.
    while (num_iterations_separating_planes <
           options.sampled_iris_options.max_iterations_separating_planes) {
      int k_squared = num_iterations_separating_planes + 1;
      k_squared *= k_squared;
      double delta_k = outer_delta * 6 / (M_PI * M_PI * k_squared);
      int N_k = internal::unadaptive_test_samples(
          options.sampled_iris_options.epsilon, delta_k,
          options.sampled_iris_options.tau);

      particles.resize(N_k);  // Entries will be overwritten.
      internal::PopulateParticlesByUniformSampling(
          P_candidate, N_k, options.sampled_iris_options.mixing_steps,
          &generators, &particles);

      // Copy top slice of particles, applying thet parameterization function to
      // each one, due to collision checker only accepting vectors of
      // configurations.
      // TODO(cohnt): Make ambient_particles an Eigen::MatrixXd and don't
      // recreate it on each iteration.
      std::vector<Eigen::VectorXd> ambient_particles(N_k);
      const auto apply_parameterization = [&particles, &ambient_particles,
                                           &options](const int thread_num,
                                                     const int64_t index) {
        unused(thread_num);
        ambient_particles[index] =
            options.parameterization.get_parameterization_double()(
                particles[index]);
      };

      // TODO(cohnt): Rerwrite as a StaticParallelForRangeLoop.
      const int num_threads_for_calling_parameterization =
          options.parameterization.get_parameterization_is_threadsafe()
              ? options.sampled_iris_options.parallelism.num_threads()
              : 1;
      StaticParallelForIndexLoop(
          DegreeOfParallelism(num_threads_for_calling_parameterization), 0, N_k,
          apply_parameterization, ParallelForBackend::BEST_AVAILABLE);

      for (int i = 0; i < ssize(ambient_particles); ++i) {
        // Only run this check in debug mode, because it's expensive.
        DRAKE_ASSERT(ambient_particles[i].size() == nq);
      }

      // Find all particles in collision or violating additional user-specified
      // constraints.
      std::vector<uint8_t> particle_col_free =
          checker.CheckConfigsCollisionFree(
              ambient_particles, options.sampled_iris_options.parallelism);
      int number_particles_in_collision = 0;

      std::vector<uint8_t> particle_satisfies_additional_constraints =
          internal::CheckProgConstraintsParallel(
              options.sampled_iris_options.prog_with_additional_constraints,
              particles,
              additional_constraints_threadsafe
                  ? options.sampled_iris_options.parallelism
                  : Parallelism::None(),
              constraints_tol, N_k);

      particles_in_collision.clear();
      for (size_t i = 0; i < particle_col_free.size(); ++i) {
        if (particle_col_free.at(i) == 0 ||
            particle_satisfies_additional_constraints[i] == 0) {
          particles_in_collision.push_back(particles.at(i));
          ++number_particles_in_collision;
        }
      }

      // Sort collision order. Only used for kGreedySampler.
      if (sampling_strategy == IrisNp2SamplingStrategy::kGreedySampler) {
        auto my_comparator = [](const VectorXd& t1, const VectorXd& t2,
                                const Hyperellipsoid& E_comparator) {
          return (t1 - E_comparator.center()).squaredNorm() <
                 (t2 - E_comparator.center()).squaredNorm();
          // TODO(cohnt): Support a custom comparator function?
        };
        std::sort(
            std::begin(particles_in_collision),
            std::begin(particles_in_collision) + number_particles_in_collision,
            std::bind(my_comparator, std::placeholders::_1,
                      std::placeholders::_2, E));
      }

      if (options.sampled_iris_options.verbose) {
        log()->info("IrisNp2 N_k {}, N_col {}, thresh {}", N_k,
                    number_particles_in_collision,
                    (1 - options.sampled_iris_options.tau) *
                        options.sampled_iris_options.epsilon * N_k);
      }

      const bool probabilistic_test_passed =
          number_particles_in_collision <=
          (1 - options.sampled_iris_options.tau) *
              options.sampled_iris_options.epsilon * N_k;

      if (options.sampled_iris_options.verbose) {
        if (!options.sampled_iris_options.remove_all_collisions_possible &&
            probabilistic_test_passed) {
          log()->info(
              "IrisNp2 probabilistic test passed! Finished computing "
              "hyperplanes.");
          break;
        } else if (probabilistic_test_passed) {
          log()->info(
              "IrisNp2 probabilistic test passed! Computing hyperplanes for "
              "remaining particles, then this iteration is finished.");
        } else {
          log()->info(
              "IrisNp2 probabilistic test failed! Continuing to compute "
              "hyperplanes.");
        }
      }
      if (!options.sampled_iris_options.remove_all_collisions_possible &&
          probabilistic_test_passed) {
        break;
      }

      // Warn user if test fails on last iteration.
      const bool is_last_iteration =
          (num_iterations_separating_planes + 1) >=
          options.sampled_iris_options.max_iterations_separating_planes;
      if (is_last_iteration && !probabilistic_test_passed) {
        log()->warn(
            "IrisNp2 WARNING, separating planes hit max iterations without "
            "passing the bernoulli test, this voids the probabilistic "
            "guarantees!");
      }

      int num_hyperplanes_added = 0;
      int num_prog_failures = 0;
      int num_prog_successes = 0;
      constexpr double kSolverFailRateWarning = 0.1;

      // TODO(cohnt): Comment on why there's two possible sets of particles to
      // work on.
      bool process_collisions_only =
          sampling_strategy == IrisNp2SamplingStrategy::kGreedySampler ||
          options.ray_sampler_options.only_walk_toward_collisions;
      int num_particles_to_walk_toward =
          process_collisions_only
              ? ssize(particles_in_collision)
              : options.ray_sampler_options.num_particles_to_walk_towards;
      std::vector<VectorXd>& particles_to_work_on =
          process_collisions_only ? particles_in_collision : particles;

      for (int particle_index = 0;
           particle_index < num_particles_to_walk_toward; ++particle_index) {
        auto& particle = particles_to_work_on[particle_index];
        if (num_hyperplanes_added >=
            options.sampled_iris_options.max_separating_planes_per_iteration) {
          break;
        }
        if (!P_candidate.PointInSet(particle)) {
          continue;
        }

        if (sampling_strategy == IrisNp2SamplingStrategy::kRaySampler) {
          bool ray_sampler_found_collision =
              RaySamplerProcess(checker, E.center(), P_candidate, options,
                                constraints_tol, &particle);
          if (!ray_sampler_found_collision) {
            continue;
          }
        }

        std::pair<Eigen::VectorXd, int> closest_collision_info;
        closest_collision_info = std::make_pair(
            particle,
            FindCollisionPairIndex(
                checker, inspector,
                checker.UpdatePositions(
                    options.parameterization.get_parameterization_double()(
                        particle)),
                sorted_pairs));

        bool solve_succeeded;
        if (closest_collision_info.second >= 0) {
          // We have found a collision pair corresponding to this particle, so
          // the particle is in collision. Thus, we solve the corresponding
          // collision counterexample search program.
          DRAKE_ASSERT(closest_collision_info.second < ssize(sorted_pairs));

          auto pair_iterator =
              std::next(sorted_pairs.begin(), closest_collision_info.second);
          DRAKE_THROW_UNLESS(pair_iterator != sorted_pairs.end());
          const auto collision_pair = *pair_iterator;
          const double padding = GetPaddingBetweenGeometries(
              checker, inspector, collision_pair.geomA, collision_pair.geomB);
          if (padding > 0) {
            points_bounded_distance_constraint->set_max_distance(padding);
          }
          ClosestCollisionProgram::AcceptableConstraint constraint =
              padding > 0 ? ClosestCollisionProgram::AcceptableConstraint(
                                points_bounded_distance_constraint)
                          : ClosestCollisionProgram::AcceptableConstraint(
                                same_point_constraint);
          ClosestCollisionProgram prog(
              constraint, *frames.at(collision_pair.geomA),
              *frames.at(collision_pair.geomB), *sets.at(collision_pair.geomA),
              *sets.at(collision_pair.geomB), E, A.topRows(num_constraints),
              b.head(num_constraints));

          if (do_debugging_visualization) {
            ++num_points_drawn;
            Eigen::VectorXd ambient_particle =
                options.parameterization.get_parameterization_double()(
                    particle);
            point_to_draw.head(nq) = ambient_particle;
            std::string path = fmt::format("iteration{:02}/{:03}/particle",
                                           iteration, num_points_drawn);
            options.sampled_iris_options.meshcat->SetObject(
                path, Sphere(0.01), geometry::Rgba(0.1, 0.1, 0.1, 1.0));
            options.sampled_iris_options.meshcat->SetTransform(
                path, RigidTransform<double>(point_to_draw));
          }

          // TODO(cohnt): Allow the user to specify the solver options used
          // here.
          solve_succeeded =
              prog.Solve(*solver, particle, options.solver_options, &closest);
        } else {
          // We did not find a collision pair corresponding to this particle, so
          // the particle must be violating one of the constraints from
          // options.sampled_iris_options.prog_with_additional_constraints.
          DRAKE_THROW_UNLESS(
              options.sampled_iris_options.prog_with_additional_constraints !=
              nullptr);

          // Find a constraint in prog_with_additional_constraints that is
          // violated. If more than one constraint is violated, we still only
          // pick one to find counterexamples for. (If we required the
          // counterexample to violate multiple constraints, it might be further
          // from the center, requiring us to solve more programs.)

          // TODO(cohnt): Consider allowing other strategies for picking which
          // violated constraint we find counterexamples for if multiple
          // constraints are violated. (For example, choose randomly, or pick
          // whichever constraint has the highest magnitude violation.)
          bool found_violated_constraint = false;
          for (const auto& binding : additional_constraint_bindings) {
            VectorXd value;
            binding.evaluator()->Eval(particle, &value);
            for (int index = 0; index < binding.evaluator()->num_constraints();
                 ++index) {
              if (value[index] >
                  binding.evaluator()->upper_bound()[index] + constraints_tol) {
                found_violated_constraint = true;
                counter_example_constraint->set(
                    &binding, index,
                    /* falsify_lower_bound */ false);
              } else if (value[index] <
                         binding.evaluator()->lower_bound()[index] -
                             constraints_tol) {
                found_violated_constraint = true;
                counter_example_constraint->set(&binding, index,
                                                /* falsify_lower_bound */ true);
              }

              if (found_violated_constraint) {
                break;
              }
            }
            if (found_violated_constraint) {
              break;
            }
          }

          // We must have found a violated constraint.
          DRAKE_THROW_UNLESS(found_violated_constraint);

          // TODO(cohnt): Allow the user to specify the solver options used
          // here.
          solve_succeeded = counter_example_prog->Solve(
              *solver, particle, options.solver_options, &closest);
        }

        bool add_hyperplane =
            solve_succeeded || options.add_hyperplane_if_solve_fails;
        ++num_hyperplanes_added;
        VectorXd* point_to_add_hyperplane{nullptr};

        if (solve_succeeded) {
          ++num_prog_successes;
          point_to_add_hyperplane = &closest;
          if (do_debugging_visualization) {
            point_to_draw.head(nq) =
                options.parameterization.get_parameterization_double()(closest);
            std::string path = fmt::format("iteration{:02}/{:03}/found",
                                           iteration, num_points_drawn);
            options.sampled_iris_options.meshcat->SetObject(
                path, Sphere(0.01), geometry::Rgba(0.8, 0.1, 0.8, 1.0));
            options.sampled_iris_options.meshcat->SetTransform(
                path, RigidTransform<double>(point_to_draw));
          }
        } else {
          ++num_prog_failures;
          point_to_add_hyperplane = &particle;
          if (do_debugging_visualization) {
            point_to_draw.head(nq) =
                options.parameterization.get_parameterization_double()(closest);
            std::string path = fmt::format("iteration{:02}/{:03}/closest",
                                           iteration, num_points_drawn);
            options.sampled_iris_options.meshcat->SetObject(
                path, Sphere(0.01), geometry::Rgba(0.1, 0.8, 0.8, 1.0));
            options.sampled_iris_options.meshcat->SetTransform(
                path, RigidTransform<double>(point_to_draw));
          }
        }

        if (add_hyperplane) {
          DRAKE_DEMAND(point_to_add_hyperplane != nullptr);
          if (options.sampled_iris_options.containment_points.has_value()) {
            internal::AddTangentToPolytope(
                E, *point_to_add_hyperplane, containment_points_vpolytope,
                *solver,
                options.sampled_iris_options.configuration_space_margin, &A, &b,
                &num_constraints, &max_relaxation);
          } else {
            internal::AddTangentToPolytope(
                E, *point_to_add_hyperplane,
                options.sampled_iris_options.configuration_space_margin,
                options.sampled_iris_options.relax_margin, &A, &b,
                &num_constraints);
          }
          P_candidate =
              HPolyhedron(A.topRows(num_constraints), b.head(num_constraints));
          if (options.sampled_iris_options.require_sample_point_is_contained) {
            const bool seed_point_requirement =
                A.row(num_constraints - 1) * seed <= b(num_constraints - 1);
            if (!seed_point_requirement) {
              log()->info(seed_point_msg);
              return P;
            }
          }

          // Update the counterexample search program. No need to update the
          // closest collision program, since it is re-created each time.
          if (counter_example_prog != nullptr) {
            counter_example_prog->UpdatePolytope(A.topRows(num_constraints),
                                                 b.head(num_constraints));
          }
        }
      }

      const double failure_rate =
          static_cast<double>(num_prog_failures) /
          static_cast<double>(num_prog_failures + num_prog_successes);
      if (failure_rate >= kSolverFailRateWarning) {
        log()->warn(fmt::format(
            "IrisNp2 WARNING, only {} out of {} closest collision "
            "programs solved successfully ({}% failure rate). If you are "
            "using SnoptSolver or NloptSolver, consider using IpoptSolver "
            "instead.",
            num_prog_successes, num_prog_successes + num_prog_failures,
            100 * failure_rate));  // Multiply by 100 to make it a percentage.
      }

      if (options.sampled_iris_options.verbose && max_relaxation > 0) {
        log()->info(
            fmt::format("IrisNp2 Warning relaxing cspace margin by {:03} to "
                        "ensure point containment",
                        max_relaxation));
      }

      if (probabilistic_test_passed) {
        break;
      }

      ++num_iterations_separating_planes;
    }

    iteration++;
    P = HPolyhedron(A.topRows(num_constraints), b.head(num_constraints));
    E = P.MaximumVolumeInscribedEllipsoid();
    const double volume = E.Volume();
    const double delta_volume = volume - last_iteration_volume;

    if (CheckTerminationConditions(iteration, delta_volume,
                                   last_iteration_volume, options)) {
      break;
    } else {
      last_iteration_volume = volume;
    }
  }
  auto stop = std::chrono::high_resolution_clock::now();
  if (options.sampled_iris_options.verbose) {
    log()->info(
        "IrisNp2 execution time : {} ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start)
            .count());
  }

  return P;
}  // NOLINT(readability/fn_size)

}  // namespace planning
}  // namespace drake
