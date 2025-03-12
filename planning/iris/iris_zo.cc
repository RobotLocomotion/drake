#include "drake/planning/iris/iris_zo.h"

#include <algorithm>
#include <string>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/common/fmt_eigen.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/multibody/plant/multibody_plant.h"
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

IrisZoOptions IrisZoOptions::CreateWithRationalKinematicParameterization(
    const multibody::RationalForwardKinematics* kin,
    const Eigen::Ref<const Eigen::VectorXd>& q_star_val) {
  const int dimension = kin->plant().num_positions();
  DRAKE_DEMAND(dimension > 0);
  IrisZoOptions instance;

  auto evaluate_s_to_q = [kin, q_star_captured = Eigen::VectorXd(q_star_val)](
                             const Eigen::VectorXd& s_val) {
    return kin->ComputeQValue(s_val, q_star_captured);
  };

  instance.set_parameterization(evaluate_s_to_q,
                                /* parameterization_is_threadsafe */ true,
                                /* parameterization_dimension */ dimension);
  return instance;
}

void IrisZoOptions::SetParameterizationFromExpression(
    const Eigen::VectorX<symbolic::Expression>& expression_parameterization,
    const Eigen::VectorX<symbolic::Variable>& variables) {
  // First, we check that the variables in expression_parameterization match the
  // user-supplied variables.
  symbolic::Variables expression_variables;
  for (const auto& expression : expression_parameterization) {
    expression_variables.insert(expression.GetVariables());
  }
  symbolic::Variables user_supplied_variables(variables);
  DRAKE_THROW_UNLESS(expression_variables == user_supplied_variables);

  // Check for duplicates in variables.
  DRAKE_THROW_UNLESS(variables.size() == ssize(user_supplied_variables));

  int dimension = ssize(expression_variables);

  // Note that in this lambda, we copy the shared_ptr variables, ensuring that
  // variables is kept alive without making a copy of the individual Variable
  // objects (which would break the substitution machinery).
  auto evaluate_expression =
      [expression_parameterization_captured =
           Eigen::VectorX<symbolic::Expression>(expression_parameterization),
       variables_captured = Eigen::VectorX<symbolic::Variable>(variables)](
          const Eigen::VectorXd& q) {
        DRAKE_ASSERT(q.size() == variables_captured.size());
        symbolic::Environment env;
        for (int i = 0; i < q.size(); ++i) {
          env.insert(variables_captured[i], q[i]);
        }
        Eigen::VectorXd out = expression_parameterization_captured.unaryExpr(
            [&env](const symbolic::Expression& expression) {
              return expression.Evaluate(env);
            });
        return out;
      };

  set_parameterization(evaluate_expression,
                       /* parameterization_is_threadsafe */ true,
                       /* parameterization_dimension */ dimension);
}

namespace {

using multibody::JointIndex;
using multibody::MultibodyConstraintId;
using multibody::internal::CouplerConstraintSpec;

std::vector<CouplerConstraintSpec> TopologicalSortCouplerConstraints(
    const std::map<multibody::MultibodyConstraintId, CouplerConstraintSpec>&
        coupler_constraints) {
  if (coupler_constraints.empty()) {
    // If there are no constraints (i.e.  is empty), we return an empty vector.
    return {};
  }

  std::map<JointIndex, JointIndex> next_map;             // joint1 -> joint0
  std::map<JointIndex, CouplerConstraintSpec> node_map;  // joint0 -> constraint
  std::map<JointIndex, bool>
      has_predecessor;  // Tracks if a joint has an incoming connection

  // Step 1: Populate maps.
  for (const auto& [_, constraint] : coupler_constraints) {
    next_map[constraint.joint1_index] = constraint.joint0_index;
    node_map[constraint.joint0_index] = constraint;
    has_predecessor[constraint.joint0_index] = true;
    has_predecessor[constraint.joint1_index] =
        has_predecessor[constraint.joint1_index];  // Ensure it exists
  }

  // Step 2: Find the start node (a joint that is never a joint0_index).
  JointIndex start_joint;
  bool found_start_joint = false;
  for (const auto& [_, constraint] : coupler_constraints) {
    if (!has_predecessor[constraint.joint1_index]) {
      start_joint = constraint.joint1_index;
      found_start_joint = true;
      break;
    }
  }

  // It should be impossible to not find a start joint, as this implies every
  // joint has a predecessor, which is not allowed with coupler constraints.
  DRAKE_ASSERT(found_start_joint);

  // Step 3: Follow the chain to construct the sorted order.
  std::vector<CouplerConstraintSpec> sorted_constraints;
  while (next_map.find(start_joint) != next_map.end()) {
    JointIndex next = next_map[start_joint];
    sorted_constraints.push_back(node_map[next]);
    start_joint = next;
  }

  return sorted_constraints;
}

}  // namespace

IrisZoOptions IrisZoOptions::CreateWithMimicJointsParameterization(
    const multibody::MultibodyPlant<double>& plant) {
  if (plant.time_step() == 0.0) {
    drake::log()->warn(
        "The provided MultibodyPlant is continuous-time, so it does not "
        "support constraints. Thus, the coupling between mimic joints cannot "
        "be parsed.");
  }
  const int dimension = plant.num_positions() - plant.num_coupler_constraints();
  DRAKE_DEMAND(dimension > 0);
  const auto& coupler_constraints = plant.get_coupler_constraint_specs();
  const std::vector<multibody::JointIndex>& joint_indices =
      plant.GetJointIndices();

  std::map<multibody::JointIndex, int> joint_index_to_position_start;
  for (const auto& joint_index : joint_indices) {
    int position_start = plant.get_joint(joint_index).position_start();
    joint_index_to_position_start.insert({joint_index, position_start});
  }

  // Sort the coupler constraints, and find the independent degrees of freedom.
  std::set<int> dependent_joint_indices;
  for (const auto& [_, coupler_constraint] : coupler_constraints) {
    dependent_joint_indices.insert(
        joint_index_to_position_start.at(coupler_constraint.joint0_index));
  }
  std::set<int> all_joint_indices;
  for (int i = 0; i < plant.num_positions(); ++i) {
    all_joint_indices.insert(i);
  }
  std::vector<int> independent_joint_indices;
  std::set_difference(all_joint_indices.begin(), all_joint_indices.end(),
                      dependent_joint_indices.begin(),
                      dependent_joint_indices.end(),
                      std::inserter(independent_joint_indices,
                                    independent_joint_indices.end()));

  std::vector<multibody::internal::CouplerConstraintSpec>
      sorted_coupler_constraints =
          TopologicalSortCouplerConstraints(coupler_constraints);

  // Construct the lambda.
  auto evaluate_parameterization =
      [independent_joint_indices, sorted_coupler_constraints,
       joint_index_to_position_start](const Eigen::VectorXd& q_in) {
        DRAKE_ASSERT(ssize(independent_joint_indices) == q_in.size());
        const int dimension_out = ssize(independent_joint_indices) +
                                  ssize(sorted_coupler_constraints);
        Eigen::VectorXd q_out(dimension_out);
        for (int i = 0; i < ssize(independent_joint_indices); ++i) {
          q_out[independent_joint_indices[i]] = q_in[i];
        }
        for (int j = 0; j < ssize(sorted_coupler_constraints); ++j) {
          const auto& coupler_constraint = sorted_coupler_constraints[j];
          int in_index =
              joint_index_to_position_start.at(coupler_constraint.joint1_index);
          int out_index =
              joint_index_to_position_start.at(coupler_constraint.joint0_index);
          q_out[out_index] = q_out[in_index] * coupler_constraint.gear_ratio +
                             coupler_constraint.offset;
        }
        return q_out;
      };

  IrisZoOptions instance;
  instance.set_parameterization(evaluate_parameterization,
                                /* parameterization_is_threadsafe */ true,
                                /* parameterization_dimension */ dimension);
  return instance;
}

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

// Given a pointer to a MathematicalProgram and a single particle, check whether
// the particle satisfies the constraints. If a nullptr is given for the
// program, return true, since the constraints are trivially-satisfied.
bool CheckProgConstraints(const MathematicalProgram* prog_ptr,
                          const Eigen::VectorXd& particle, const double tol) {
  if (!prog_ptr) {
    return true;
  }
  for (const auto& binding : prog_ptr->GetAllConstraints()) {
    DRAKE_ASSERT(binding.evaluator() != nullptr);
    if (!binding.evaluator()->CheckSatisfied(particle, tol)) {
      return false;
    }
  }
  return true;
}

// Given a pointer to a MathematicalProgram and a list of particles (where each
// particle is a choice of values for its decision variables), check in parallel
// which particles satisfy all constraints, and which don't. Each entry in the
// output vector corresponds to the corresponding particle. 1 means it satisfies
// the constraints, 0 means it doesn't. If a nullptr is given for the program,
// return a vector of all 1s, since all particles trivially satisfy the
// constraints. The user can specify a slice of particles to check [0,
// end_index). Default behavior is to check all particles -- when end_index is
// std::nullopt, it is set to ssize(particles).
std::vector<uint8_t> CheckProgConstraints(
    const MathematicalProgram* prog_ptr,
    const std::vector<Eigen::VectorXd>& particles, const int num_threads_to_use,
    const double tol, std::optional<int> end_index = std::nullopt) {
  int actual_end_index = end_index.value_or(ssize(particles));
  DRAKE_DEMAND(actual_end_index >= 0 && actual_end_index <= ssize(particles));
  std::vector<uint8_t> is_valid(actual_end_index, 1);
  if (!prog_ptr) {
    return is_valid;
  }
  DRAKE_DEMAND(prog_ptr->IsThreadSafe() || num_threads_to_use == 1);
  const auto check_particle_work = [&prog_ptr, &particles, &tol, &is_valid](
                                       const int thread_num,
                                       const int64_t index) {
    unused(thread_num);
    is_valid[index] = static_cast<uint8_t>(
        CheckProgConstraints(prog_ptr, particles[index], tol));
  };

  DynamicParallelForIndexLoop(DegreeOfParallelism(num_threads_to_use), 0,
                              actual_end_index, check_particle_work,
                              ParallelForBackend::BEST_AVAILABLE);
  return is_valid;
}

}  // namespace

HPolyhedron IrisZo(const planning::CollisionChecker& checker,
                   const Hyperellipsoid& starting_ellipsoid,
                   const HPolyhedron& domain, const IrisZoOptions& options) {
  auto start = std::chrono::high_resolution_clock::now();
  const bool additional_constraints_threadsafe =
      options.prog_with_additional_constraints
          ? options.prog_with_additional_constraints->IsThreadSafe()
          : true;
  const int num_threads_to_use =
      checker.SupportsParallelChecking() && additional_constraints_threadsafe &&
              options.get_parameterization_is_threadsafe()
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

  const int ambient_dimension = checker.plant().num_positions();
  const int parameterized_dimension =
      options.get_parameterization_dimension().value_or(ambient_dimension);

  DRAKE_THROW_UNLESS(num_threads_to_use > 0);
  DRAKE_THROW_UNLESS(starting_ellipsoid.ambient_dimension() ==
                     parameterized_dimension);
  DRAKE_THROW_UNLESS(domain.ambient_dimension() == parameterized_dimension);
  DRAKE_THROW_UNLESS(domain.IsBounded());
  DRAKE_THROW_UNLESS(domain.PointInSet(current_ellipsoid_center));

  if (options.prog_with_additional_constraints) {
    DRAKE_THROW_UNLESS(options.prog_with_additional_constraints->num_vars() ==
                       parameterized_dimension);
  }
  // TODO(cohnt): Allow users to set this parameter if it ever becomes needed.
  const double constraints_tol = 1e-6;

  const Eigen::VectorXd starting_ellipsoid_center_ambient =
      options.get_parameterization()(starting_ellipsoid_center);
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
      CheckProgConstraints(options.prog_with_additional_constraints,
                           starting_ellipsoid_center, constraints_tol);
  if (!starting_ellipsoid_center_valid) {
    throw std::runtime_error(
        fmt::format("Starting ellipsoid center {} is in collision, or violates "
                    "a constraint in options.prog_with_additional_constraints.",
                    fmt_eigen(starting_ellipsoid_center.transpose())));
  }

  int current_num_faces = domain.A().rows();

  if (options.max_iterations_separating_planes <= 0) {
    throw std::runtime_error(
        "The maximum number of iterations for separating planes "
        "'options.max_iterations_separating_planes' must be larger than zero.");
  }
  VPolytope cvxh_vpoly;
  if (options.containment_points.has_value()) {
    cvxh_vpoly = VPolytope(options.containment_points.value());
    DRAKE_THROW_UNLESS(parameterized_dimension ==
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
      cont_vec.emplace_back(options.get_parameterization()(conf));
      DRAKE_ASSERT(cont_vec.back().size() == ambient_dimension);
    }

    std::vector<uint8_t> containment_point_col_free =
        checker.CheckConfigsCollisionFree(cont_vec, options.parallelism);
    for (const auto col_free : containment_point_col_free) {
      if (!col_free) {
        throw std::runtime_error(
            "One or more containment points are in collision!");
      }
    }
    for (int i = 0; i < options.containment_points->cols(); ++i) {
      if (!CheckProgConstraints(options.prog_with_additional_constraints,
                                options.containment_points->col(i),
                                constraints_tol)) {
        throw std::runtime_error(
            "One or more containment points violates a constraint in "
            "options.prog_with_additional_constraints!");
      }
    }
  }
  // For debugging visualization.
  Eigen::Vector3d point_to_draw = Eigen::Vector3d::Zero();
  if (options.meshcat && ambient_dimension <= 3) {
    std::string path = "seedpoint";
    options.meshcat->SetObject(path, Sphere(0.06),
                               geometry::Rgba(0.1, 1, 1, 1.0));
    Eigen::VectorXd conf_ambient =
        options.get_parameterization()(current_ellipsoid_center);
    DRAKE_ASSERT(conf_ambient.size() == ambient_dimension);
    point_to_draw.head(ambient_dimension) = conf_ambient;
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

      // Copy top slice of particles, applying thet parameterization function to
      // each one, due to collision checker only accepting vectors of
      // configurations.
      // TODO(wernerpe, cohnt): Remove this copy operation.
      // TODO(cohnt): Consider parallelizing the parameterization calls, in case
      // it's an expensive operation.
      std::vector<Eigen::VectorXd> ambient_particles(N_k);
      std::transform(particles.begin(), particles.begin() + N_k,
                     ambient_particles.begin(), options.get_parameterization());

      for (int i = 0; i < ssize(ambient_particles); ++i) {
        // Only run this check in debug mode, because it's expensive.
        DRAKE_ASSERT(ambient_particles[i].size() == ambient_dimension);
      }

      // Find all particles in collision.
      std::vector<uint8_t> particle_col_free =
          checker.CheckConfigsCollisionFree(ambient_particles,
                                            options.parallelism);
      std::vector<uint8_t> particle_satisfies_additional_constraints =
          CheckProgConstraints(options.prog_with_additional_constraints,
                               particles, num_threads_to_use, constraints_tol,
                               N_k);
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
      const auto particle_update_work = [&checker,
                                         &particles_in_collision_updated,
                                         &particles_in_collision,
                                         &current_ellipsoid_center, &options,
                                         &constraints_tol](
                                            const int thread_num,
                                            const int64_t index) {
        const int point_index = static_cast<int>(index);
        auto start_point = particles_in_collision[point_index];

        Eigen::VectorXd current_point = start_point;
        Eigen::VectorXd curr_pt_lower = current_ellipsoid_center;

        // Update current point using a fixed number of bisection steps.
        Eigen::VectorXd current_point_ambient =
            options.get_parameterization()(curr_pt_lower);
        DRAKE_ASSERT(current_point_ambient.size() ==
                     checker.plant().num_positions());
        if (!checker.CheckConfigCollisionFree(current_point_ambient,
                                              thread_num) ||
            !CheckProgConstraints(options.prog_with_additional_constraints,
                                  curr_pt_lower, constraints_tol)) {
          current_point = curr_pt_lower;
        } else {
          Eigen::VectorXd curr_pt_upper = current_point;
          for (int i = 0; i < options.bisection_steps; ++i) {
            Eigen::VectorXd query = 0.5 * (curr_pt_upper + curr_pt_lower);
            Eigen::VectorXd query_ambient =
                options.get_parameterization()(query);
            DRAKE_ASSERT(query_ambient.size() ==
                         checker.plant().num_positions());
            if (checker.CheckConfigCollisionFree(query_ambient, thread_num) &&
                CheckProgConstraints(options.prog_with_additional_constraints,
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

        particles_in_collision_updated[point_index] = current_point;
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
          if (options.meshcat && ambient_dimension <= 3) {
            for (int pt_to_draw = 0; pt_to_draw < number_particles_in_collision;
                 ++pt_to_draw) {
              std::string path = fmt::format(
                  "face_pt/iteration{:02}/sepit{:02}/{:03}/pt", iteration,
                  num_iterations_separating_planes, current_num_faces);
              options.meshcat->SetObject(path, Sphere(0.03),
                                         geometry::Rgba(1, 1, 0.1, 1.0));
              Eigen::VectorXd ambient_particle =
                  options.get_parameterization()(nearest_particle);
              DRAKE_ASSERT(ambient_particle.size() == ambient_dimension);
              point_to_draw.head(ambient_dimension) = ambient_particle;
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
    if (!checker.CheckConfigCollisionFree(
            options.get_parameterization()(current_ellipsoid_center)) ||
        !CheckProgConstraints(options.prog_with_additional_constraints,
                              current_ellipsoid_center, constraints_tol)) {
      log()->info(fmt::format(
          "IrisZo terminating early because new ellipsoid center "
          "{} is in collision, or violates a constraint in "
          "options.prog_with_additional_constraints. Consider decreasing "
          "options.epsilon (which was {}) to require less of the region be in "
          "collision, or decreasing options.delta (which was {}) to require a "
          "higher confidence in how much of the region is in collision.",
          fmt_eigen(current_ellipsoid_center.transpose()), options.epsilon,
          options.delta));
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
