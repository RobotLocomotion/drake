#include "drake/planning/iris/iris_zo.h"

#include <algorithm>
#include <string>
#include <vector>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::DynamicParallelForIndexLoop;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForIndexLoop;
using geometry::Sphere;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::VPolytope;
using math::RigidTransform;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::SolverInterface;

namespace {

using values_t = std::vector<double>;
using index_t = std::vector<uint8_t>;

index_t argsort(values_t const& values) {
  index_t index(values.size());
  std::iota(index.begin(), index.end(), 0);
  std::sort(index.begin(), index.end(), [&values](uint8_t a, uint8_t b) {
    return values[a] < values[b];
  });
  return index;
}

}  // namespace

HPolyhedron IrisZo(const planning::CollisionChecker& checker,
                   const Hyperellipsoid& starting_ellipsoid,
                   const HPolyhedron& domain, const IrisZoOptions& options) {
  auto start = std::chrono::high_resolution_clock::now();
  const bool additional_constraints_threadsafe =
      options.sampled_iris_options.prog_with_additional_constraints
          ? options.sampled_iris_options.prog_with_additional_constraints
                ->IsThreadSafe()
          : true;
  const int num_threads_to_use =
      checker.SupportsParallelChecking() && additional_constraints_threadsafe &&
              options.parameterization.get_parameterization_is_threadsafe()
          ? std::min(options.sampled_iris_options.parallelism.num_threads(),
                     checker.num_allocated_contexts())
          : 1;
  log()->info("IrisZo using {} threads.", num_threads_to_use);

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

  const Eigen::VectorXd starting_ellipsoid_center = starting_ellipsoid.center();

  Hyperellipsoid current_ellipsoid = starting_ellipsoid;
  Eigen::VectorXd current_ellipsoid_center = starting_ellipsoid.center();
  Eigen::MatrixXd current_ellipsoid_A = starting_ellipsoid.A();

  // Prevent directly terminating if the ellipsoid is too large.
  double previous_volume = 0;

  const int ambient_dimension = checker.plant().num_positions();
  const int parameterized_dimension =
      options.parameterization.get_parameterization_dimension().value_or(
          ambient_dimension);

  DRAKE_THROW_UNLESS(num_threads_to_use > 0);
  DRAKE_THROW_UNLESS(starting_ellipsoid.ambient_dimension() ==
                     parameterized_dimension);
  DRAKE_THROW_UNLESS(domain.ambient_dimension() == parameterized_dimension);
  DRAKE_THROW_UNLESS(domain.IsBounded());
  DRAKE_THROW_UNLESS(domain.PointInSet(current_ellipsoid_center));

  if (options.sampled_iris_options.prog_with_additional_constraints) {
    DRAKE_THROW_UNLESS(options.sampled_iris_options
                           .prog_with_additional_constraints->num_vars() ==
                       parameterized_dimension);
  }
  // TODO(cohnt): Allow users to set this parameter if it ever becomes needed.
  const double constraints_tol = 1e-6;

  const Eigen::VectorXd starting_ellipsoid_center_ambient =
      options.parameterization.get_parameterization_double()(
          starting_ellipsoid_center);
  const int computed_ambient_dimension =
      starting_ellipsoid_center_ambient.size();
  if (computed_ambient_dimension != ambient_dimension) {
    throw std::runtime_error(fmt::format(
        "The plant has {} positions, but the given parameterization "
        "returned a point with the wrong dimension (its size was "
        "{}) when called on {}.",
        ambient_dimension, computed_ambient_dimension,
        fmt_eigen(starting_ellipsoid_center.transpose())));
  }

  bool starting_ellipsoid_center_valid =
      checker.CheckConfigCollisionFree(starting_ellipsoid_center_ambient) &&
      internal::CheckProgConstraints(
          options.sampled_iris_options.prog_with_additional_constraints,
          starting_ellipsoid_center, constraints_tol);
  if (!starting_ellipsoid_center_valid) {
    throw std::runtime_error(fmt::format(
        "Starting ellipsoid center {} is in collision, or violates "
        "a constraint in "
        "options.sampled_iris_options.prog_with_additional_constraints.",
        fmt_eigen(starting_ellipsoid_center.transpose())));
  }

  int current_num_faces = domain.A().rows();

  if (options.sampled_iris_options.max_iterations_separating_planes <= 0) {
    throw std::runtime_error(
        "The maximum number of iterations for separating planes "
        "'options.sampled_iris_options.max_iterations_separating_planes' must "
        "be larger than zero.");
  }
  VPolytope containment_points_vpolytope =
      internal::ParseAndCheckContainmentPoints(
          checker, options.sampled_iris_options, options.parameterization,
          starting_ellipsoid);

  // For debugging visualization.
  Eigen::Vector3d point_to_draw = Eigen::Vector3d::Zero();
  if (options.sampled_iris_options.meshcat && ambient_dimension <= 3) {
    std::string path = "seedpoint";
    options.sampled_iris_options.meshcat->SetObject(
        path, Sphere(0.06), geometry::Rgba(0.1, 1, 1, 1.0));
    Eigen::VectorXd conf_ambient =
        options.parameterization.get_parameterization_double()(
            current_ellipsoid_center);
    DRAKE_ASSERT(conf_ambient.size() == ambient_dimension);
    point_to_draw.head(ambient_dimension) = conf_ambient;
    options.sampled_iris_options.meshcat->SetTransform(
        path, RigidTransform<double>(point_to_draw));
  }

  // Upper bound on number of particles required if we hit max iterations.
  double outer_delta_min =
      internal::calc_delta_min(options.sampled_iris_options.delta,
                               options.sampled_iris_options.max_iterations);
  double delta_min = internal::calc_delta_min(
      outer_delta_min,
      options.sampled_iris_options.max_iterations_separating_planes);
  int N_test_max = internal::unadaptive_test_samples(
      options.sampled_iris_options.epsilon, delta_min,
      options.sampled_iris_options.tau);
  int N_max = std::max(N_test_max, options.sampled_iris_options.num_particles);

  if (options.sampled_iris_options.verbose) {
    log()->info(
        "IrisZo finding region that is {} collision free with {} certainty "
        "using up to {} particles.",
        options.sampled_iris_options.epsilon,
        1 - options.sampled_iris_options.delta, N_max);
    log()->info("IrisZo worst case test requires {} samples.", N_test_max);
  }

  std::vector<Eigen::VectorXd> particles(
      N_max, Eigen::VectorXd::Zero(parameterized_dimension));

  int iteration = 0;
  HPolyhedron P = domain;
  HPolyhedron P_prev = domain;

  // Pre-allocate memory for the polyhedron we are going to construct.
  // TODO(wernerpe): Find a better solution than hardcoding 300.
  constexpr int kNumFacesToPreAllocate = 300;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      P.A().rows() + kNumFacesToPreAllocate, parameterized_dimension);
  Eigen::VectorXd b(P.A().rows() + kNumFacesToPreAllocate);

  // Preallocate the solver as we will be solving a lot of QPs in the loop,
  // specifically in ComputeFaceTangentToDistCvxh.
  std::unique_ptr<SolverInterface> solver = solvers::MakeFirstAvailableSolver(
      {solvers::GurobiSolver::id(), solvers::ClarabelSolver::id(),
       solvers::MosekSolver::id(), solvers::OsqpSolver::id()});
  // TODO(cohnt): Use solvers::ChooseBestSolver.
  while (true) {
    log()->info("IrisZo outer iteration {}", iteration);

    Eigen::MatrixXd ATA = current_ellipsoid_A.transpose() * current_ellipsoid_A;

    // Initialize polytope with domain.
    A.topRows(domain.A().rows()) = domain.A();
    b.head(domain.A().rows()) = domain.b();

    int num_iterations_separating_planes = 0;

    // Track maximum relaxation of cspace margin if containment_points are
    // requested.
    double max_relaxation = 0;

    double outer_delta = options.sampled_iris_options.delta * 6 /
                         (M_PI * M_PI * (iteration + 1) * (iteration + 1));

    // No need for decaying outer delta if we are guaranteed to terminate after
    // one step. In this case we can be less conservative and set it to our
    // total accepted error probability.
    if (options.sampled_iris_options.max_iterations == 1) {
      outer_delta = options.sampled_iris_options.delta;
    }

    // Separating Planes Step.
    // TODO(cohnt): Rewrite as a for loop for better readability.
    while (num_iterations_separating_planes <
           options.sampled_iris_options.max_iterations_separating_planes) {
      int k_squared = num_iterations_separating_planes + 1;
      k_squared *= k_squared;
      double delta_k = outer_delta * 6 / (M_PI * M_PI * k_squared);
      int N_test = internal::unadaptive_test_samples(
          options.sampled_iris_options.epsilon, delta_k,
          options.sampled_iris_options.tau);
      int N_samples_to_draw =
          std::max(N_test, options.sampled_iris_options.num_particles);

      // TODO(cohnt): Switch to using a single large MatrixXd to avoid repeated
      // VectorXd heap allocations.
      internal::PopulateParticlesByUniformSampling(
          P, N_samples_to_draw, options.sampled_iris_options.mixing_steps,
          &generators, &particles);

      // Copy top slice of particles, applying thet parameterization function to
      // each one, due to collision checker only accepting vectors of
      // configurations.
      // TODO(cohnt): Make ambient_particles an Eigen::MatrixXd and don't
      // recreate it on each iteration.
      std::vector<Eigen::VectorXd> ambient_particles(N_samples_to_draw);
      const auto apply_parameterization = [&particles, &ambient_particles,
                                           &options](const int thread_num,
                                                     const int64_t index) {
        unused(thread_num);
        ambient_particles[index] =
            options.parameterization.get_parameterization_double()(
                particles[index]);
      };

      // TODO(cohnt): Rewrite as a StaticParallelForRangeLoop.
      StaticParallelForIndexLoop(DegreeOfParallelism(num_threads_to_use), 0,
                                 N_samples_to_draw, apply_parameterization,
                                 ParallelForBackend::BEST_AVAILABLE);

      for (int i = 0; i < ssize(ambient_particles); ++i) {
        // Only run this check in debug mode, because it's expensive.
        DRAKE_ASSERT(ambient_particles[i].size() == ambient_dimension);
      }

      // Find all particles in collision.
      std::vector<uint8_t> particle_col_free =
          checker.CheckConfigsCollisionFree(
              ambient_particles, options.sampled_iris_options.parallelism);
      std::vector<uint8_t> particle_satisfies_additional_constraints =
          internal::CheckProgConstraintsParallel(
              options.sampled_iris_options.prog_with_additional_constraints,
              particles, Parallelism(num_threads_to_use), constraints_tol,
              N_samples_to_draw);
      DRAKE_ASSERT(particle_col_free.size() ==
                   particle_satisfies_additional_constraints.size());

      std::vector<Eigen::VectorXd> particles_in_collision;
      int number_particles_in_collision_unadaptive_test = 0;
      int number_particles_in_collision = 0;
      for (size_t i = 0; i < particle_col_free.size(); ++i) {
        if (particle_col_free[i] == 0 ||
            particle_satisfies_additional_constraints[i] == 0) {
          // Only push back a maximum of num_particles for optimization of the
          // faces.
          if (options.sampled_iris_options.num_particles >
              number_particles_in_collision) {
            particles_in_collision.push_back(particles[i]);
            ++number_particles_in_collision;
          }
          if (N_test > number_particles_in_collision_unadaptive_test) {
            ++number_particles_in_collision_unadaptive_test;
          }
        }
      }
      if (options.sampled_iris_options.verbose) {
        log()->info("IrisZo N_test {}, N_col {}, thresh {}", N_test,
                    number_particles_in_collision_unadaptive_test,
                    (1 - options.sampled_iris_options.tau) *
                        options.sampled_iris_options.epsilon * N_test);
      }

      const bool probabilistic_test_passed =
          number_particles_in_collision_unadaptive_test <=
          (1 - options.sampled_iris_options.tau) *
              options.sampled_iris_options.epsilon * N_test;

      if (options.sampled_iris_options.verbose) {
        if (!options.sampled_iris_options.remove_all_collisions_possible &&
            probabilistic_test_passed) {
          log()->info(
              "IrisZo probabilistic test passed! Finished computing "
              "hyperplanes.");
          break;
        } else if (probabilistic_test_passed) {
          log()->info(
              "IrisZo probabilistic test passed! Computing hyperplanes for "
              "remaining particles, then this iteration is finished.");
        } else {
          log()->info(
              "IrisZo probabilistic test failed! Continuing to compute "
              "hyperplanes.");
        }
      }
      if (!options.sampled_iris_options.remove_all_collisions_possible &&
          probabilistic_test_passed) {
        break;
      }

      const bool is_last_iteration =
          (num_iterations_separating_planes + 1) >=
          options.sampled_iris_options.max_iterations_separating_planes;
      if (is_last_iteration && !probabilistic_test_passed) {
        log()->warn(
            "IrisZo WARNING, separating planes hit max iterations without "
            "passing the unadaptive test, this voids the probabilistic "
            "guarantees!");
      }

      // Update particle positions.
      std::vector<Eigen::VectorXd> particles_in_collision_updated;
      particles_in_collision_updated.reserve(particles_in_collision.size());
      for (auto p : particles_in_collision) {
        particles_in_collision_updated.emplace_back(p);
      }

      // For each particle in collision, we run a bisection search to find a
      // configuration on the boundary of the obstacle.
      const auto particle_update_work = [&checker,
                                         &particles_in_collision_updated,
                                         &particles_in_collision,
                                         &current_ellipsoid_center, &options,
                                         &constraints_tol](
                                            const int thread_num,
                                            const int64_t index) {
        auto start_point = particles_in_collision[index];

        Eigen::VectorXd current_point = start_point;
        Eigen::VectorXd curr_pt_lower = current_ellipsoid_center;

        // Update current point using a fixed number of bisection steps.
        Eigen::VectorXd current_point_ambient =
            options.parameterization.get_parameterization_double()(
                curr_pt_lower);
        DRAKE_ASSERT(current_point_ambient.size() ==
                     checker.plant().num_positions());
        if (!checker.CheckConfigCollisionFree(current_point_ambient,
                                              thread_num) ||
            !internal::CheckProgConstraints(
                options.sampled_iris_options.prog_with_additional_constraints,
                curr_pt_lower, constraints_tol)) {
          current_point = curr_pt_lower;
        } else {
          Eigen::VectorXd curr_pt_upper = current_point;
          for (int i = 0; i < options.bisection_steps; ++i) {
            Eigen::VectorXd query = 0.5 * (curr_pt_upper + curr_pt_lower);
            Eigen::VectorXd query_ambient =
                options.parameterization.get_parameterization_double()(query);
            DRAKE_ASSERT(query_ambient.size() ==
                         checker.plant().num_positions());
            if (checker.CheckConfigCollisionFree(query_ambient, thread_num) &&
                internal::CheckProgConstraints(
                    options.sampled_iris_options
                        .prog_with_additional_constraints,
                    query, constraints_tol)) {
              // The query point is collision free and satisfies the
              // constraints.
              curr_pt_lower = query;
            } else {
              curr_pt_upper = query;
              current_point = query;
            }
          }
        }

        particles_in_collision_updated[index] = current_point;
      };

      // Update all particles in parallel.
      DynamicParallelForIndexLoop(DegreeOfParallelism(num_threads_to_use), 0,
                                  number_particles_in_collision,
                                  particle_update_work,
                                  ParallelForBackend::BEST_AVAILABLE);

      // Resampling particles around found collisions.
      // TODO(wernerpe): Implement optional resampling step.

      // Place Hyperplanes.
      std::vector<double> particle_distances;
      particle_distances.reserve(number_particles_in_collision);

      for (auto particle : particles_in_collision_updated) {
        particle_distances.emplace_back(
            (particle - current_ellipsoid_center).transpose() * ATA *
            (particle - current_ellipsoid_center));
      }

      // The indices are returned in ascending order.
      auto indices_sorted = argsort(particle_distances);

      // Type std::vector<Bool> is not threadsafe - using uint8_t instead to
      // accommodate for parallel checking.
      std::vector<uint8_t> particle_is_redundant;

      for (int i = 0; i < number_particles_in_collision; ++i) {
        particle_is_redundant.push_back(0);
      }

      // Add separating planes.
      int hyperplanes_added = 0;
      for (auto i : indices_sorted) {
        auto nearest_particle = particles_in_collision_updated[i];
        if (!particle_is_redundant[i]) {
          if (options.sampled_iris_options.containment_points.has_value()) {
            internal::AddTangentToPolytope(
                current_ellipsoid, nearest_particle,
                containment_points_vpolytope, *solver,
                options.sampled_iris_options.configuration_space_margin, &A, &b,
                &current_num_faces, &max_relaxation);
          } else {
            internal::AddTangentToPolytope(
                current_ellipsoid, nearest_particle,
                options.sampled_iris_options.configuration_space_margin,
                options.sampled_iris_options.relax_margin, &A, &b,
                &current_num_faces);
          }
          ++hyperplanes_added;

          // Fill in meshcat if added for debugging.
          if (options.sampled_iris_options.meshcat && ambient_dimension <= 3) {
            for (int pt_to_draw = 0; pt_to_draw < number_particles_in_collision;
                 ++pt_to_draw) {
              std::string path = fmt::format(
                  "face_pt/iteration{:02}/sepit{:02}/{:03}/pt", iteration,
                  num_iterations_separating_planes, current_num_faces);
              options.sampled_iris_options.meshcat->SetObject(
                  path, Sphere(0.03), geometry::Rgba(1, 1, 0.1, 1.0));
              Eigen::VectorXd ambient_particle =
                  options.parameterization.get_parameterization_double()(
                      nearest_particle);
              DRAKE_ASSERT(ambient_particle.size() == ambient_dimension);
              point_to_draw.head(ambient_dimension) = ambient_particle;
              options.sampled_iris_options.meshcat->SetTransform(
                  path, RigidTransform<double>(point_to_draw));
            }
          }

          if (hyperplanes_added == options.sampled_iris_options
                                       .max_separating_planes_per_iteration &&
              options.sampled_iris_options.max_separating_planes_per_iteration >
                  0) {
            break;
          }

          particle_is_redundant.at(i) = true;

          // Loop over remaining non-redundant particles and check for
          // redundancy.
          const auto mark_redundant_particles = [&](const int thread_num,
                                                    const int64_t index) {
            unused(thread_num);
            if (!particle_is_redundant[index]) {
              const double margin = A.row(current_num_faces - 1) *
                                        particles_in_collision_updated[index] -
                                    b(current_num_faces - 1);
              if (margin >= 0) {
                particle_is_redundant[index] = 1;
              }
            }
          };

          DynamicParallelForIndexLoop(DegreeOfParallelism(num_threads_to_use),
                                      0, number_particles_in_collision,
                                      mark_redundant_particles,
                                      ParallelForBackend::BEST_AVAILABLE);
        }
      }

      P = HPolyhedron(A.topRows(current_num_faces), b.head(current_num_faces));
      if (max_relaxation > 0) {
        log()->info(
            fmt::format("IrisZo Warning relaxing cspace margin by {:03} to "
                        "ensure point containment",
                        max_relaxation));
      }
      if (probabilistic_test_passed) {
        break;
      }
      ++num_iterations_separating_planes;

      // Log updates at 20-percent intervals of
      // max_iterations_separating_planes.
      int divisor =
          static_cast<int>(
              0.2 *
              options.sampled_iris_options.max_iterations_separating_planes) +
          1;
      if ((num_iterations_separating_planes - 1) % divisor == 0 &&
          options.sampled_iris_options.verbose) {
        log()->info("SeparatingPlanes iteration: {} faces: {}",
                    num_iterations_separating_planes, current_num_faces);
      }
    }

    current_ellipsoid = P.MaximumVolumeInscribedEllipsoid();
    current_ellipsoid_A = current_ellipsoid.A();
    current_ellipsoid_center = current_ellipsoid.center();

    const double volume = current_ellipsoid.Volume();
    const double delta_volume = volume - previous_volume;
    if (delta_volume <= options.sampled_iris_options.termination_threshold) {
      log()->info("IrisZo delta vol {}, threshold {}", delta_volume,
                  options.sampled_iris_options.termination_threshold);
      break;
    }
    if (delta_volume / (previous_volume + 1e-6) <=
        options.sampled_iris_options.relative_termination_threshold) {
      log()->info("IrisZo reldelta vol {}, threshold {}",
                  delta_volume / previous_volume,
                  options.sampled_iris_options.relative_termination_threshold);
      break;
    }
    ++iteration;
    if (!(iteration < options.sampled_iris_options.max_iterations)) {
      log()->info("IrisZo iter {}, iter limit {}", iteration,
                  options.sampled_iris_options.max_iterations);
      break;
    }
    if (!checker.CheckConfigCollisionFree(
            options.parameterization.get_parameterization_double()(
                current_ellipsoid_center)) ||
        !internal::CheckProgConstraints(
            options.sampled_iris_options.prog_with_additional_constraints,
            current_ellipsoid_center, constraints_tol)) {
      log()->info(fmt::format(
          "IrisZo terminating early because new ellipsoid center {} is in "
          "collision, or violates a constraint in "
          "options.sampled_iris_options.prog_with_additional_constraints. "
          "Consider decreasing "
          "options.sampled_iris_options.epsilon (which was {}) to require less "
          "of the region be in collision, or decreasing "
          "options.sampled_iris_options.delta (which was {}) to require a "
          "higher confidence in how much of the region is in collision.",
          fmt_eigen(current_ellipsoid_center.transpose()),
          options.sampled_iris_options.epsilon,
          options.sampled_iris_options.delta));
      break;
    }

    if (options.sampled_iris_options.require_sample_point_is_contained) {
      if (!(P.PointInSet(starting_ellipsoid_center))) {
        log()->info("IrisZo ERROR initial seed point not contained in region.");
        return P_prev;
      }
    }
    previous_volume = volume;
    // Reset polytope to domain, store previous iteration.
    P_prev = P;
    P = domain;
    current_num_faces = P.A().rows();
  }
  auto stop = std::chrono::high_resolution_clock::now();
  if (options.sampled_iris_options.verbose) {
    log()->info(
        "IrisZo execution time : {} ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start)
            .count());
  }
  return P;
}

}  // namespace planning
}  // namespace drake
