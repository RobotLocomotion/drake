#include "drake/planning/iris/iris_zo.h"

#include <algorithm>
#include <string>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/common/fmt_eigen.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::DynamicParallelForIndexLoop;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForIndexLoop;
using geometry::Meshcat;
using geometry::Sphere;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::VPolytope;
using math::RigidTransform;
using solvers::MathematicalProgram;

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

Eigen::VectorXd compute_face_tangent_to_dist_cvxh(
    const Hyperellipsoid& E, const Eigen::Ref<const Eigen::VectorXd>& point,
    const VPolytope& cvxh_vpoly) {
  Eigen::VectorXd a_face = E.A().transpose() * E.A() * (point - E.center());
  double b_face = a_face.transpose() * point;

  // Return standard iris face if either the face does not chop off any
  // containment points or collision lies inside of the convex hull of the
  // containment points.
  if (cvxh_vpoly.PointInSet(point) ||
      (a_face.transpose() * cvxh_vpoly.vertices()).maxCoeff() - b_face <= 0) {
    return a_face;
  } else {
    MathematicalProgram prog;
    int dim = cvxh_vpoly.ambient_dimension();
    std::vector<solvers::SolverId> preferred_solvers{
        solvers::MosekSolver::id(), solvers::ClarabelSolver::id()};
    auto x = prog.NewContinuousVariables(dim);
    cvxh_vpoly.AddPointInSetConstraints(&prog, x);
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(dim, dim);
    prog.AddQuadraticErrorCost(identity, point, x);
    auto result = solvers::Solve(prog);
    DRAKE_THROW_UNLESS(result.is_success());
    a_face = point - result.GetSolution(x);
    return a_face;
  }
}

// See Definition 1 in the paper.
int unadaptive_test_samples(double epsilon, double delta, double tau) {
  return static_cast<int>(-2 * std::log(delta) / (tau * tau * epsilon) + 0.5);
}

}  // namespace

HPolyhedron IrisZo(const planning::CollisionChecker& checker,
                   const Hyperellipsoid& starting_ellipsoid,
                   const HPolyhedron& domain, const IrisZoOptions& options) {
  auto start = std::chrono::high_resolution_clock::now();
  const int num_threads_to_use =
      checker.SupportsParallelChecking()
          ? std::min(options.parallelism.num_threads(),
                     checker.num_allocated_contexts())
          : 1;
  log()->info("IrisZo using {} threads.", num_threads_to_use);
  RandomGenerator generator(options.random_seed);

  const Eigen::VectorXd starting_ellipsoid_center = starting_ellipsoid.center();

  Hyperellipsoid current_ellipsoid = starting_ellipsoid;
  Eigen::VectorXd current_ellipsoid_center = starting_ellipsoid.center();
  Eigen::MatrixXd current_ellipsoid_A = starting_ellipsoid.A();

  // Prevent directly terminating if the ellipsoid is too large.
  double previous_volume = 0;

  const int dim = starting_ellipsoid.ambient_dimension();
  int current_num_faces = domain.A().rows();

  DRAKE_THROW_UNLESS(num_threads_to_use > 0);
  DRAKE_THROW_UNLESS(domain.ambient_dimension() == dim);
  DRAKE_THROW_UNLESS(domain.IsBounded());
  DRAKE_THROW_UNLESS(domain.PointInSet(current_ellipsoid_center));

  if (options.max_iterations_separating_planes <= 0) {
    throw std::runtime_error(
        "The maximum number of iterations for separating planes "
        "'options.max_iterations_separating_planes' must be larger than zero.");
  }
  VPolytope cvxh_vpoly;
  if (options.containment_points.has_value()) {
    cvxh_vpoly = VPolytope(options.containment_points.value());
    DRAKE_THROW_UNLESS(domain.ambient_dimension() ==
                       options.containment_points->rows());

    constexpr float kPointInSetTol = 1e-5;
    if (!cvxh_vpoly.PointInSet(starting_ellipsoid.center(), kPointInSetTol)) {
      throw std::runtime_error(
          "The center of the starting ellipsoid lies outside of the convex "
          "hull of the containment points.");
    }

    cvxh_vpoly = cvxh_vpoly.GetMinimalRepresentation();

    std::vector<Eigen::VectorXd> cont_vec;
    cont_vec.reserve((options.containment_points->cols()));

    for (int col = 0; col < options.containment_points->cols(); ++col) {
      Eigen::VectorXd conf = options.containment_points->col(col);
      cont_vec.emplace_back(conf);
    }

    std::vector<uint8_t> containment_point_col_free =
        checker.CheckConfigsCollisionFree(cont_vec, options.parallelism);
    for (const auto col_free : containment_point_col_free) {
      if (!col_free) {
        throw std::runtime_error(
            "One or more containment points are in collision!");
      }
    }
  }
  // For debugging visualization.
  Eigen::Vector3d point_to_draw = Eigen::Vector3d::Zero();
  if (options.meshcat && dim <= 3) {
    std::string path = "seedpoint";
    options.meshcat->SetObject(path, Sphere(0.06),
                               geometry::Rgba(0.1, 1, 1, 1.0));
    point_to_draw.head(dim) = current_ellipsoid_center;
    options.meshcat->SetTransform(path, RigidTransform<double>(point_to_draw));
  }

  // Upper bound on number of particles required if we hit max iterations.
  double outer_delta_min =
      options.delta * 6 /
      (M_PI * M_PI * options.max_iterations * options.max_iterations);

  double delta_min = outer_delta_min * 6 /
                     (M_PI * M_PI * options.max_iterations_separating_planes *
                      options.max_iterations_separating_planes);

  int N_max = unadaptive_test_samples(options.epsilon, delta_min, options.tau);

  if (options.verbose) {
    log()->info(
        "IrisZo finding region that is {} collision free with {} certainty "
        "using {} particles.",
        options.epsilon, 1 - options.delta, options.num_particles);
    log()->info("IrisZo worst case test requires {} samples.", N_max);
  }

  std::vector<Eigen::VectorXd> particles(N_max, Eigen::VectorXd::Zero(dim));

  int iteration = 0;
  HPolyhedron P = domain;
  HPolyhedron P_prev = domain;

  // Pre-allocate memory for the polyhedron we are going to construct.
  // TODO(wernerpe): Find a better solution than hardcoding 300.
  constexpr int kNumFacesToPreAllocate = 300;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      P.A().rows() + kNumFacesToPreAllocate, dim);
  Eigen::VectorXd b(P.A().rows() + kNumFacesToPreAllocate);

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

    double outer_delta =
        options.delta * 6 / (M_PI * M_PI * (iteration + 1) * (iteration + 1));

    // No need for decaying outer delta if we are guaranteed to terminate after
    // one step. In this case we can be less conservative and set it to our
    // total accepted error probability.
    if (options.max_iterations == 1) {
      outer_delta = options.delta;
    }

    // Separating Planes Step.
    while (num_iterations_separating_planes <
           options.max_iterations_separating_planes) {
      int k_squared = num_iterations_separating_planes + 1;
      k_squared *= k_squared;
      double delta_k = outer_delta * 6 / (M_PI * M_PI * k_squared);
      int N_k = unadaptive_test_samples(options.epsilon, delta_k, options.tau);

      particles.at(0) = P.UniformSample(&generator, current_ellipsoid_center,
                                        options.mixing_steps);
      // Populate particles by uniform sampling.
      for (int i = 1; i < N_k; ++i) {
        particles.at(i) = P.UniformSample(&generator, particles.at(i - 1),
                                          options.mixing_steps);
      }

      // Copy top slice of particles, due to collision checker only accepting
      // vectors of configurations.
      // TODO(wernerpe): Remove this copy operation.
      std::vector<Eigen::VectorXd> particles_step(particles.begin(),
                                                  particles.begin() + N_k);

      // Find all particles in collision
      std::vector<uint8_t> particle_col_free =
          checker.CheckConfigsCollisionFree(particles_step,
                                            options.parallelism);
      std::vector<Eigen::VectorXd> particles_in_collision;
      int number_particles_in_collision_unadaptive_test = 0;
      int number_particles_in_collision = 0;
      for (size_t i = 0; i < particle_col_free.size(); ++i) {
        if (particle_col_free[i] == 0) {
          // Only push back a maximum of num_particles for optimization of the
          // faces.
          if (options.num_particles > number_particles_in_collision) {
            particles_in_collision.push_back(particles[i]);
            ++number_particles_in_collision;
          }
          ++number_particles_in_collision_unadaptive_test;
        }
      }
      if (options.verbose) {
        log()->info("IrisZo N_k {}, N_col {}, thresh {}", N_k,
                    number_particles_in_collision_unadaptive_test,
                    (1 - options.tau) * options.epsilon * N_k);
      }

      if (number_particles_in_collision_unadaptive_test <=
          (1 - options.tau) * options.epsilon * N_k) {
        break;
      }

      if (num_iterations_separating_planes ==
          options.max_iterations_separating_planes - 1) {
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
      const auto particle_update_work =
          [&checker, &particles_in_collision_updated, &particles_in_collision,
           &current_ellipsoid_center,
           &options](const int thread_num, const int64_t index) {
            const int point_idx = static_cast<int>(index);
            auto start_point = particles_in_collision[point_idx];

            Eigen::VectorXd current_point = start_point;
            Eigen::VectorXd curr_pt_lower = current_ellipsoid_center;

            // Update current point using a fixed number of bisection steps.
            if (!checker.CheckConfigCollisionFree(curr_pt_lower, thread_num)) {
              current_point = curr_pt_lower;
            } else {
              Eigen::VectorXd curr_pt_upper = current_point;
              for (int i = 0; i < options.bisection_steps; ++i) {
                Eigen::VectorXd query = 0.5 * (curr_pt_upper + curr_pt_lower);
                if (checker.CheckConfigCollisionFree(query, thread_num)) {
                  curr_pt_lower = query;
                } else {
                  curr_pt_upper = query;
                  current_point = query;
                }
              }
            }

            particles_in_collision_updated[point_idx] = current_point;
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
          Eigen::VectorXd a_face;
          if (options.containment_points.has_value()) {
            a_face = compute_face_tangent_to_dist_cvxh(
                current_ellipsoid, nearest_particle, cvxh_vpoly);
          } else {
            a_face = ATA * (nearest_particle - current_ellipsoid_center);
          }

          a_face.normalize();
          double b_face = a_face.transpose() * nearest_particle -
                          options.configuration_space_margin;

          // Relax cspace margin to contain points.
          if (options.containment_points.has_value()) {
            Eigen::VectorXd result =
                a_face.transpose() * options.containment_points.value();
            double relaxation = result.maxCoeff() - b_face;
            if (relaxation > 0) {
              b_face += relaxation;
              if (max_relaxation < relaxation) max_relaxation = relaxation;
            }
          }
          A.row(current_num_faces) = a_face.transpose();
          b(current_num_faces) = b_face;
          ++current_num_faces;
          ++hyperplanes_added;

          // Resize A matrix if we need more faces.
          if (A.rows() <= current_num_faces) {
            A.conservativeResize(A.rows() * 2, A.cols());
            b.conservativeResize(b.rows() * 2);
          }

          // Fill in meshcat if added for debugging.
          if (options.meshcat && dim <= 3) {
            for (int pt_to_draw = 0; pt_to_draw < number_particles_in_collision;
                 ++pt_to_draw) {
              std::string path = fmt::format(
                  "face_pt/iteration{:02}/sepit{:02}/{:03}/pt", iteration,
                  num_iterations_separating_planes, current_num_faces);
              options.meshcat->SetObject(path, Sphere(0.03),
                                         geometry::Rgba(1, 1, 0.1, 1.0));
              point_to_draw.head(dim) = nearest_particle;
              options.meshcat->SetTransform(
                  path, RigidTransform<double>(point_to_draw));
            }
          }

          if (hyperplanes_added ==
                  options.max_separating_planes_per_iteration &&
              options.max_separating_planes_per_iteration > 0)
            break;

          particle_is_redundant.at(i) = true;

          // Loop over remaining non-redundant particles and check for
          // redundancy.
          // TODO(cohnt): Revert this back to parallel but with CRU.
          for (int particle_index = 0;
               particle_index < number_particles_in_collision;
               ++particle_index) {
            if (!particle_is_redundant[particle_index]) {
              if (a_face.transpose() *
                          particles_in_collision_updated[particle_index] -
                      b_face >=
                  0) {
                particle_is_redundant[particle_index] = 1;
              }
            }
          }
        }
      }

      P = HPolyhedron(A.topRows(current_num_faces), b.head(current_num_faces));
      if (max_relaxation > 0) {
        log()->info(
            fmt::format("IrisZo Warning relaxing cspace margin by {:03} to "
                        "ensure point containment",
                        max_relaxation));
      }
      // Resampling particles in current polyhedron for next iteration.
      particles[0] = P.UniformSample(&generator, options.mixing_steps);
      for (int j = 1; j < options.num_particles; ++j) {
        particles[j] =
            P.UniformSample(&generator, particles[j - 1], options.mixing_steps);
      }
      ++num_iterations_separating_planes;

      // Log updates at 20-percent intervals of
      // max_iterations_separating_planes.
      int divisor =
          static_cast<int>(0.2 * options.max_iterations_separating_planes) + 1;
      if ((num_iterations_separating_planes - 1) % divisor == 0 &&
          options.verbose) {
        log()->info("SeparatingPlanes iteration: {} faces: {}",
                    num_iterations_separating_planes, current_num_faces);
      }
    }

    current_ellipsoid = P.MaximumVolumeInscribedEllipsoid();
    current_ellipsoid_A = current_ellipsoid.A();
    current_ellipsoid_center = current_ellipsoid.center();

    const double volume = current_ellipsoid.Volume();
    const double delta_volume = volume - previous_volume;
    if (delta_volume <= options.termination_threshold) {
      log()->info("IrisZo delta vol {}, threshold {}", delta_volume,
                  options.termination_threshold);
      break;
    }
    if (delta_volume / (previous_volume + 1e-6) <=
        options.relative_termination_threshold) {
      log()->info("IrisZo reldelta vol {}, threshold {}",
                  delta_volume / previous_volume,
                  options.relative_termination_threshold);
      break;
    }
    ++iteration;
    if (!(iteration < options.max_iterations)) {
      log()->info("IrisZo iter {}, iter limit {}", iteration,
                  options.max_iterations);
      break;
    }

    if (options.require_sample_point_is_contained) {
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
  if (options.verbose) {
    log()->info(
        "IrisZo execution time : {} ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start)
            .count());
  }
  return P;
}

}  // namespace planning
}  // namespace drake
