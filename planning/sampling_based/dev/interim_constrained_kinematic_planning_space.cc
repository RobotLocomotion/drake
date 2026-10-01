#include "drake/planning/sampling_based/dev/interim_constrained_kinematic_planning_space.h"

#include <limits>
#include <utility>

#include <fmt/format.h>

#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {
constexpr double kTargetCloseEnough = 1e-6;

namespace {

bool CheckConstraintSatisfied(
    const solvers::Constraint& constraint, const Eigen::VectorXd& q,
    double tolerance,
    const std::function<void(const std::string&)>& log_function) {
  if (constraint.CheckSatisfied(q, tolerance)) {
    return true;
  } else {
    if (log_function != nullptr) {
      log_function(fmt::format("constraint with description [{}] not satisfied",
                               constraint.get_description()));
    }
    return false;
  }
}

bool CheckConstraintsSatisfied(
    const std::vector<std::shared_ptr<solvers::Constraint>>& constraints,
    const Eigen::VectorXd& q, double tolerance,
    spdlog::level::level_enum logging_level = spdlog::level::debug) {
  std::function<void(const std::string&)> log_function = nullptr;
  if (drake::log()->level() <= logging_level) {
    log_function = [logging_level](const std::string& msg) {
      drake::log()->log(logging_level, "CheckConstraintsSatisfied: {}", msg);
    };
  }

  bool constraints_met = true;
  for (const auto& constraint : constraints) {
    if (!CheckConstraintSatisfied(*constraint, q, tolerance, log_function)) {
      constraints_met = false;
      if (log_function == nullptr) {
        break;
      }
    }
  }

  if (constraints_met) {
    if (log_function != nullptr) {
      log_function("all constraints met");
    }
    return true;
  } else {
    return false;
  }
}

enum class ConstrainedEdgeCheckResult : uint8_t {
  EdgeValid = 0,
  EdgeInCollision = 1,
  EdgeViolatesConstraints = 2
};

ConstrainedEdgeCheckResult CheckEdgeCollisionFreeAndSatisfiesConstraints(
    const Eigen::VectorXd& start, const Eigen::VectorXd& end,
    const CollisionChecker& collision_checker,
    const std::vector<std::shared_ptr<solvers::Constraint>>& constraints,
    double tolerance, CollisionCheckerContext* model_context) {
  DRAKE_THROW_UNLESS(model_context != nullptr);

  // Helper to check a single configuration. Note that collision is always
  // checked first.
  const auto check_single_config = [&](const Eigen::VectorXd& q) {
    if (collision_checker.CheckContextConfigCollisionFree(model_context, q)) {
      if (CheckConstraintsSatisfied(constraints, q, tolerance)) {
        return ConstrainedEdgeCheckResult::EdgeValid;
      } else {
        return ConstrainedEdgeCheckResult::EdgeViolatesConstraints;
      }
    } else {
      return ConstrainedEdgeCheckResult::EdgeInCollision;
    }
  };

  // Fail fast if end is invalid. This method is used by motion planners that
  // extend/connect towards some target configuration, and thus require a number
  // of edge collision checks in which start is often known to be valid while
  // end is unknown. Many of these potential extensions/connections will result
  // in an invalid configuration, so failing fast on an invalid end helps reduce
  // the work of checking colliding edges. There is also no need to special case
  // checking start, since it will be the first configuration checked in the
  // loop.
  const ConstrainedEdgeCheckResult end_result = check_single_config(end);
  if (end_result != ConstrainedEdgeCheckResult::EdgeValid) {
    return end_result;
  }

  const double distance =
      collision_checker.ComputeConfigurationDistance(start, end);
  const double step_size = collision_checker.edge_step_size();
  const int num_steps =
      static_cast<int>(std::max(1.0, std::ceil(distance / step_size)));
  for (int step = 0; step < num_steps; ++step) {
    const double ratio =
        static_cast<double>(step) / static_cast<double>(num_steps);
    const Eigen::VectorXd qinterp =
        collision_checker.InterpolateBetweenConfigurations(start, end, ratio);
    const ConstrainedEdgeCheckResult q_result = check_single_config(qinterp);
    if (q_result != ConstrainedEdgeCheckResult::EdgeValid) {
      return q_result;
    }
  }
  return ConstrainedEdgeCheckResult::EdgeValid;
}

}  // namespace

InterimConstrainedKinematicPlanningSpace::
    InterimConstrainedKinematicPlanningSpace(
        std::unique_ptr<drake::planning::CollisionChecker> collision_checker,
        const JointLimits& joint_limits,
        const ConstraintsFactoryFunction& constraints_factory_fn,
        const double propagation_step_size,
        const double minimum_propagation_progress, const double tolerance,
        const uint64_t seed)
    : SymmetricCollisionCheckerPlanningSpace<Eigen::VectorXd>(
          std::move(collision_checker), joint_limits, seed) {
  context_and_constraints_keeper_ = ContextAndConstraintsKeeper(
      &this->collision_checker(), constraints_factory_fn);
  SetPropagationStepSize(propagation_step_size);
  SetMinimumPropagationProgress(minimum_propagation_progress);
  SetTolerance(tolerance);
}

InterimConstrainedKinematicPlanningSpace::
    ~InterimConstrainedKinematicPlanningSpace() = default;

InterimConstrainedKinematicPlanningSpace::
    InterimConstrainedKinematicPlanningSpace(
        const InterimConstrainedKinematicPlanningSpace& other)
    : SymmetricCollisionCheckerPlanningSpace<Eigen::VectorXd>(other) {
  context_and_constraints_keeper_ = ContextAndConstraintsKeeper(
      &this->collision_checker(),
      other.context_and_constraints_keeper_.constraints_factory_function());
  SetPropagationStepSize(other.propagation_step_size());
  SetMinimumPropagationProgress(other.minimum_propagation_progress());
  SetTolerance(other.tolerance());
}

std::unique_ptr<PlanningSpace<Eigen::VectorXd>>
InterimConstrainedKinematicPlanningSpace::DoClone() const {
  return std::unique_ptr<InterimConstrainedKinematicPlanningSpace>(
      new InterimConstrainedKinematicPlanningSpace(*this));
}

bool InterimConstrainedKinematicPlanningSpace::DoCheckStateValidity(
    const Eigen::VectorXd& state, const int thread_number) const {
  const auto& context_and_constraints =
      context_and_constraints_keeper_.context_and_constraints(thread_number);
  const auto& checker_context = context_and_constraints.context();
  const auto& constraints = context_and_constraints.constraints();

  // TODO(calderpg) Incorporate joint limits check into state validity.
  if (!collision_checker().CheckContextConfigCollisionFree(
          checker_context.get(), state)) {
    return false;
  }

  return CheckConstraintsSatisfied(constraints, state, tolerance());
}

bool InterimConstrainedKinematicPlanningSpace::DoCheckEdgeValidity(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to,
    const int thread_number) const {
  const auto& context_and_constraints =
      context_and_constraints_keeper_.context_and_constraints(thread_number);
  const auto& checker_context = context_and_constraints.context();
  const auto& constraints = context_and_constraints.constraints();

  // TODO(calderpg) Incorporate joint limits check into edge validity.
  const auto edge_check = CheckEdgeCollisionFreeAndSatisfiesConstraints(
      from, to, collision_checker(), constraints, tolerance(),
      checker_context.get());

  return edge_check == ConstrainedEdgeCheckResult::EdgeValid;
}

bool InterimConstrainedKinematicPlanningSpace::DoCheckPathValidity(
    const std::vector<Eigen::VectorXd>& path, const int thread_number) const {
  const auto& context_and_constraints =
      context_and_constraints_keeper_.context_and_constraints(thread_number);
  const auto& checker_context = context_and_constraints.context();
  const auto& constraints = context_and_constraints.constraints();

  if (path.size() > 1) {
    for (size_t index = 1; index < path.size(); ++index) {
      const Eigen::VectorXd& previous = path.at(index - 1);
      const Eigen::VectorXd& current = path.at(index);

      // TODO(calderpg) Incorporate joint limits check into path validity.
      const auto edge_check = CheckEdgeCollisionFreeAndSatisfiesConstraints(
          previous, current, collision_checker(), constraints, tolerance(),
          checker_context.get());

      if (edge_check != ConstrainedEdgeCheckResult::EdgeValid) {
        return false;
      }
    }
    return true;
  } else if (path.size() == 1) {
    // TODO(calderpg) Incorporate joint limits check into path validity.
    if (!collision_checker().CheckContextConfigCollisionFree(
            checker_context.get(), path.at(0))) {
      return false;
    }

    return CheckConstraintsSatisfied(constraints, path.at(0), tolerance());
  } else {
    throw std::runtime_error("Cannot check zero-waypoint paths for validity");
  }
}

Eigen::VectorXd InterimConstrainedKinematicPlanningSpace::DoSampleState(
    const int thread_number) {
  const JointLimits& limits = joint_limits();
  Eigen::VectorXd sample = Eigen::VectorXd::Zero(limits.num_positions());
  for (int index = 0; index < sample.size(); ++index) {
    const double lower = limits.position_lower()(index);
    const double upper = limits.position_upper()(index);
    const double ratio = random_source().DrawUniformUnitReal(thread_number);
    sample(index) = std::lerp(lower, upper, ratio);
  }
  return sample;
}

std::optional<Eigen::VectorXd>
InterimConstrainedKinematicPlanningSpace::DoMaybeSampleValidState(
    const int max_attempts, const int thread_number) {
  // Assemble reused projection program components.
  const auto& context_and_constraints =
      context_and_constraints_keeper_.context_and_constraints(thread_number);
  const auto& checker_context = context_and_constraints.context();
  const auto& constraints = context_and_constraints.constraints();

  drake::solvers::MathematicalProgram projection;
  // Make joint limits constraint + quaternion constraint if needed.
  auto q_var = projection.NewContinuousVariables(
      collision_checker().plant().num_positions(), "q");
  projection.AddBoundingBoxConstraint(joint_limits().position_lower(),
                                      joint_limits().position_upper(), q_var);
  drake::multibody::AddUnitQuaternionConstraintOnPlant(
      collision_checker().plant(), q_var, &projection);
  // Add the user-provided constraints.
  for (const auto& constraint : constraints) {
    projection.AddConstraint(constraint, q_var);
  }
  // Set solver options.
  // TODO(calderpg) Should these tolerance values be the same as tolerance
  // parameter?
  projection.SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major optimality tolerance", 1e-5);
  projection.SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major feasibility tolerance", 1e-5);

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    const Eigen::VectorXd initial_q = SampleState(thread_number);
    if (!collision_checker().CheckContextConfigCollisionFree(
            checker_context.get(), initial_q)) {
      continue;
    }

    // Add a cost to the difference between q and projection.
    auto cost = projection.AddQuadraticCost((initial_q - q_var).squaredNorm());
    // Remove the cost at the end of scope.
    drake::ScopeExit guard([&projection, &cost]() {
      projection.RemoveCost(cost);
    });

    // Solve projection.
    const auto result = drake::solvers::Solve(projection, initial_q);
    const Eigen::VectorXd& q_projected = result.GetSolution();

    const bool constraints_satisfied =
        result.is_success() ||
        CheckConstraintsSatisfied(constraints, q_projected, tolerance());
    const bool collision_free =
        collision_checker().CheckContextConfigCollisionFree(
            checker_context.get(), q_projected);

    if (constraints_satisfied && collision_free) {
      drake::log()->trace("Projected {} to {}", fmt_eigen(initial_q),
                          fmt_eigen(q_projected));
      return std::optional<Eigen::VectorXd>(q_projected);
    } else {
      drake::log()->trace("Failed to project");
    }
  }

  return std::nullopt;
}

double InterimConstrainedKinematicPlanningSpace::DoStateDistance(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
  return collision_checker().ComputeConfigurationDistance(from, to);
}

Eigen::VectorXd InterimConstrainedKinematicPlanningSpace::DoInterpolate(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to,
    const double ratio) const {
  return collision_checker().InterpolateBetweenConfigurations(from, to, ratio);
}

std::vector<Eigen::VectorXd>
InterimConstrainedKinematicPlanningSpace::DoPropagate(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to,
    std::map<std::string, double>* propagation_statistics,
    const int thread_number) {
  // Ensure that propagation_statistics contains the right keys. If additional
  // tracking statistics are added below, they must be added here as well. Note
  // that try_emplace only adds a new element if it does not already exist.
  propagation_statistics->try_emplace("targets_considered", 0.0);
  propagation_statistics->try_emplace("projected_targets", 0.0);
  propagation_statistics->try_emplace("projected_target_in_collision", 0.0);
  propagation_statistics->try_emplace("projected_target_insufficient_progress",
                                      0.0);
  propagation_statistics->try_emplace("target_failed_to_project", 0.0);
  propagation_statistics->try_emplace("edges_considered", 0.0);
  propagation_statistics->try_emplace("valid_edges", 0.0);
  propagation_statistics->try_emplace("edges_in_collision", 0.0);
  propagation_statistics->try_emplace("edges_violated_constraints", 0.0);
  propagation_statistics->try_emplace("complete_propagation_successful", 0.0);

  // Assemble reused projection program components.
  const auto& context_and_constraints =
      context_and_constraints_keeper_.context_and_constraints(thread_number);
  const auto& checker_context = context_and_constraints.context();
  const auto& constraints = context_and_constraints.constraints();

  drake::solvers::MathematicalProgram projection;
  // Make joint limits constraint + quaternion constraint if needed.
  auto q_var = projection.NewContinuousVariables(
      collision_checker().plant().num_positions(), "q");
  projection.AddBoundingBoxConstraint(joint_limits().position_lower(),
                                      joint_limits().position_upper(), q_var);
  drake::multibody::AddUnitQuaternionConstraintOnPlant(
      collision_checker().plant(), q_var, &projection);
  // Add the user-provided constraints.
  for (const auto& constraint : constraints) {
    projection.AddConstraint(constraint, q_var);
  }
  // Set solver options.
  // TODO(calderpg) Should these tolerance values be the same as tolerance
  // parameter?
  projection.SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major optimality tolerance", 1e-5);
  projection.SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major feasibility tolerance", 1e-5);

  // Helper for the projection operation.
  const auto project_to_constraints =
      [&](const Eigen::VectorXd& current_q) -> std::optional<Eigen::VectorXd> {
    // Add a cost to the difference between q and projection.
    auto cost = projection.AddQuadraticCost((current_q - q_var).squaredNorm());
    // Remove the cost at the end of scope.
    drake::ScopeExit guard([&projection, &cost]() {
      projection.RemoveCost(cost);
    });

    // Solve projection.
    const auto result = drake::solvers::Solve(projection, current_q);
    const Eigen::VectorXd& q_projected = result.GetSolution();

    const bool constraints_satisfied =
        result.is_success() ||
        CheckConstraintsSatisfied(constraints, q_projected, tolerance());

    if (constraints_satisfied) {
      return q_projected;
    } else {
      return std::nullopt;
    }
  };

  std::vector<Eigen::VectorXd> propagated_states;

  // Compute a maximum number of steps to take.
  const double total_distance = StateDistance(from, to);
  const int total_steps =
      static_cast<int>(std::ceil(total_distance / propagation_step_size()));
  Eigen::VectorXd current = from;
  int steps = 0;
  bool complete_propagation_successful = false;
  while (steps < total_steps) {
    // Compute the next intermediate target state.
    Eigen::VectorXd current_target = to;
    const double target_distance = StateDistance(current, current_target);

    if (std::abs(target_distance) <= kTargetCloseEnough) {
      // If we've reached the target state, stop.
      complete_propagation_successful = true;
      break;
    } else if (target_distance > propagation_step_size()) {
      // If we have more than one stop left, interpolate a target state.
      const double step_fraction = propagation_step_size() / target_distance;
      const Eigen::VectorXd interpolated_target =
          Interpolate(current, to, step_fraction);
      current_target = interpolated_target;
    }

    // Try projecting target to meet constraints.
    (*propagation_statistics)["targets_considered"] += 1.0;
    const auto projected = project_to_constraints(current_target);

    if (!projected) {
      drake::log()->trace(
          "Constrained propagation stopped because current_target failed to "
          "project to meet constraints");
      (*propagation_statistics)["target_failed_to_project"] += 1.0;
      break;
    }

    (*propagation_statistics)["projected_targets"] += 1.0;
    current_target = projected.value();

    const bool projection_collision_free =
        collision_checker().CheckContextConfigCollisionFree(
            checker_context.get(), current_target);

    if (!projection_collision_free) {
      drake::log()->trace(
          "Constrained propagation stopped due to projected target in "
          "collision (pre edge check)");
      (*propagation_statistics)["projected_target_in_collision"] += 1.0;
      break;
    }

    const double propagation_progress = StateDistance(current, current_target);

    if (propagation_progress < minimum_propagation_progress()) {
      drake::log()->trace(
          "Constrained propagation stopped due to projected target not "
          "making enough forward progress (pre edge check)");
      (*propagation_statistics)["projected_target_stalled"] += 1.0;
      break;
    }

    (*propagation_statistics)["edges_considered"] += 1.0;

    const ConstrainedEdgeCheckResult edge_check_result =
        CheckEdgeCollisionFreeAndSatisfiesConstraints(
            current, current_target, collision_checker(), constraints,
            tolerance(), checker_context.get());

    switch (edge_check_result) {
      case ConstrainedEdgeCheckResult::EdgeValid: {
        propagated_states.push_back(current_target);
        current = current_target;
        ++steps;
        // If this is the last step, record that it was successful.
        if (steps == total_steps) {
          complete_propagation_successful = true;
        }
        (*propagation_statistics)["valid_edges"] += 1.0;
        break;
      }
      case ConstrainedEdgeCheckResult::EdgeInCollision: {
        drake::log()->trace(
            "Constrained propagation stopped due to edge in collision");
        (*propagation_statistics)["edges_in_collision"] += 1.0;
        break;
      }
      case ConstrainedEdgeCheckResult::EdgeViolatesConstraints: {
        drake::log()->trace(
            "Constrained propagation stopped due to edge violating "
            "constraints");
        (*propagation_statistics)["edges_violated_constraints"] += 1.0;
        break;
      }
    }

    if (edge_check_result != ConstrainedEdgeCheckResult::EdgeValid) {
      break;
    }
  }

  if (complete_propagation_successful) {
    (*propagation_statistics)["complete_propagation_successful"] += 1.0;
  }
  return propagated_states;
}

double InterimConstrainedKinematicPlanningSpace::DoMotionCost(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
  return StateDistance(from, to);
}

std::optional<Eigen::VectorXd>
InterimConstrainedKinematicPlanningSpace::TryProjectStateToConstraints(
    const Eigen::VectorXd& state,
    const std::optional<int> thread_number) const {
  // Make context and constraints.
  const auto& context_and_constraints =
      context_and_constraints_keeper_.context_and_constraints(
          ResolveThreadNumber(thread_number));
  const auto& checker_context = context_and_constraints.context();
  const auto& constraints = context_and_constraints.constraints();

  drake::solvers::MathematicalProgram projection;
  // Make joint limits constraint + quaternion constraint if needed.
  auto q_var = projection.NewContinuousVariables(
      collision_checker().plant().num_positions(), "q");
  projection.AddBoundingBoxConstraint(joint_limits().position_lower(),
                                      joint_limits().position_upper(), q_var);
  drake::multibody::AddUnitQuaternionConstraintOnPlant(
      collision_checker().plant(), q_var, &projection);
  // Add the user-provided constraints.
  for (const auto& constraint : constraints) {
    projection.AddConstraint(constraint, q_var);
  }
  // Set solver options.
  // TODO(calderpg) Should these tolerance values be the same as tolerance
  // parameter?
  projection.SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major optimality tolerance", 1e-5);
  projection.SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major feasibility tolerance", 1e-5);

  // Add a cost to the difference between q and projection.
  projection.AddQuadraticCost((state - q_var).squaredNorm());

  // Solve projection.
  const auto result = drake::solvers::Solve(projection, state);
  const Eigen::VectorXd& q_projected = result.GetSolution();

  const bool constraints_satisfied =
      result.is_success() ||
      CheckConstraintsSatisfied(constraints, q_projected, tolerance());
  const bool collision_free =
      collision_checker().CheckContextConfigCollisionFree(checker_context.get(),
                                                          q_projected);

  if (constraints_satisfied && collision_free) {
    drake::log()->trace("Projected {} to {}", fmt_eigen(state),
                        fmt_eigen(q_projected));
    return std::optional<Eigen::VectorXd>(q_projected);
  } else {
    drake::log()->trace("Failed to project");
    return std::nullopt;
  }
}

}  // namespace planning
}  // namespace drake
