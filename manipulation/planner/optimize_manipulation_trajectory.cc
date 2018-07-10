#include "drake/manipulation/planner/optimize_manipulation_trajectory.h"

#include <algorithm>

#include "drake/common/text_logging.h"
#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/manipulation/planner/kinematic_tree.h"
#include "drake/math/bspline_basis.h"
#include "drake/math/bspline_curve.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"

using drake::math::BsplineBasis;
using drake::math::BsplineCurve;
using drake::math::Transform;
using drake::solvers::Constraint;

namespace drake {
namespace manipulation {
namespace planner {
namespace {

std::vector<double> GuessDurations(
    const KinematicTree& robot,
    const std::vector<VectorX<double>>& desired_configurations) {
  std::vector<double> durations;
  durations.reserve(desired_configurations.size() - 1);
  for (auto q = std::next(desired_configurations.begin());
       q != desired_configurations.end(); ++q) {
    DRAKE_ASSERT(q->size() == robot.num_velocities());
    VectorX<double> diff{q - std::prev(q)};

    for (int j = 0; j < diff.size(); j++) {
      if (diff(j) < 0) {
        diff(j) /= robot.joint_velocity_lower_limit()(j);
      } else {
        diff(j) /= robot.joint_velocity_upper_limit()(j);
      }
      // Either bound is 0.
      if (std::isnan(diff(j))) diff(j) = 0;
    }
    DRAKE_DEMAND(diff.minCoeff() >= 0);
    // Don't return 0 duration
    durations.push_back(std::max(diff.maxCoeff(), 0.1));
  }
  return durations;
}

std::pair<Eigen::VectorXd, double> DurationsToPlanFractionAndTotalDuration(
    std::vector<double> durations) {
  const int num_via_points = static_cast<int>(durations.size());
  if (num_via_points == 0) {
    return {Eigen::VectorXd(0), 0.0};
  }
  Eigen::VectorXd plan_fraction(num_via_points);
  plan_fraction(0) = durations.at(0);
  for (int i = 1; i < num_via_points; ++i) {
    plan_fraction(i) = plan_fraction(i - 1) + durations.at(i);
  }
  const double total_duration = plan_fraction(num_via_points - 1);
  if (total_duration > 0) {
    plan_fraction /= total_duration;
  }
  return {plan_fraction, total_duration};
}

KinematicTrajectoryOptimization MakeKinematicTrajectoryOptimization(
    const KinematicTree& robot, const drake::optional<VectorX<double>>& q0,
    TimedConstraintVector user_constraints = {},
    const drake::optional<OptimizeManipulationTrajectoryParameters>&
        parameters = {}) {
  const OptimizeManipulationTrajectoryParameters actual_parameters{
      parameters.value_or(OptimizeManipulationTrajectoryParameters())};
  const int num_positions = robot.num_positions();
  const int num_velocities = robot.num_velocities();
  KinematicTrajectoryOptimization program(BsplineCurve<double>(
      math::BsplineBasis(actual_parameters.spline_order,
                         actual_parameters.num_control_points),
      std::vector<Eigen::MatrixXd>(actual_parameters.num_control_points,
                                   VectorX<double>::Zero(num_positions))));

  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Linesearch tolerance", 0.95);
  double t = 1e-8;
  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Function precision", t);
  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major optimality tolerance", std::sqrt(t));
  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major feasibility tolerance", std::sqrt(t));

  // Add min-time objective.
  if (actual_parameters.duration_weight > 0) {
    program.AddDurationCost(actual_parameters.duration_weight);
  }
  if (actual_parameters.velocity_weight > 0) {
    program.AddVelocityCost(actual_parameters.velocity_weight);
  }
  if (actual_parameters.acceleration_weight > 0) {
    program.AddAccelerationCost(actual_parameters.acceleration_weight);
  }
  if (actual_parameters.jerk_weight > 0) {
    program.AddJerkCost(actual_parameters.jerk_weight);
  }

  // Add bounds on duration.
  program.AddDurationBounds(actual_parameters.min_duration,
                            actual_parameters.max_duration);

  // Add position limits.
  program.AddPositionBounds(robot.joint_position_lower_limit(),
                            robot.joint_position_upper_limit());

  // Add velocity limits.
  program.AddVelocityBounds(robot.joint_velocity_lower_limit(),
                            robot.joint_velocity_upper_limit());

  // Constrain initial position if specified.
  if (q0) {
    program.AddFixedPositionConstraint(*q0, 0.);
  }

  // Constrain intial and final velocity
  program.AddFixedVelocityConstraint(VectorX<double>::Zero(num_velocities), 0.);
  program.AddFixedVelocityConstraint(VectorX<double>::Zero(num_velocities), 1.);

  // Add collision-avoidance constraint.
  if (actual_parameters.collision_avoidance_threshold > 0) {
    program.AddGenericPositionConstraint(
        robot.MakeCollisionAvoidanceConstraint(
            2 * actual_parameters.collision_avoidance_threshold),
        {{0., 1.}},
        robot.MakeCollisionAvoidanceConstraint(
            actual_parameters.collision_avoidance_threshold));
  }

  // Add user specified constraints.
  const int num_user_constraints = user_constraints.size();
  for (int i = 0; i < num_user_constraints; ++i) {
    program.AddGenericPositionConstraint(user_constraints.at(i).first,
                                         user_constraints.at(i).second);
  }

  return program;
}

optional<BsplineCurve<double>> SolveKinematicTrajectoryOptimization(
    const drake::optional<double>& total_duration,
    const KinematicTrajectoryOptimization& program,
    const std::vector<math::BsplineCurve<double>>& seed_trajectories) {
  DRAKE_ASSERT(seed_trajectories.size() > 0);
  std::vector<KinematicTrajectoryOptimization> programs;
  programs.reserve(seed_trajectories.size());
  for (const auto& seed : seed_trajectories) {
    programs.push_back(program);
    programs.back().SetPositionCurve(seed);
  }
  return KinematicTrajectoryOptimization::ComputeFirstSolution(&programs,
                                                               total_duration);
}

}  // namespace

drake::optional<math::BsplineCurve<double>> OptimizeTrajectory(
    const KinematicTree& robot, const drake::optional<Eigen::VectorXd>& q0,
    const drake::optional<OptimizeManipulationTrajectoryParameters>& parameters,
    const TimedConstraintVector& user_constraints,
    const std::vector<math::BsplineCurve<double>> seed_trajectories) {
  // Create the KinematicTrajectoryOptimization object. This constrains the
  // inital configuration.
  KinematicTrajectoryOptimization program{MakeKinematicTrajectoryOptimization(
      robot, q0 /* q0 */, user_constraints, parameters)};

  return SolveKinematicTrajectoryOptimization(drake::nullopt, program,
                                              seed_trajectories);
}

drake::optional<BsplineCurve<double>>
OptimizeTrajectoryThroughDesiredConfigurations(
    const KinematicTree& robot,
    const std::vector<VectorX<double>>& desired_configurations,
    const drake::optional<std::vector<double>>& durations,
    const drake::optional<OptimizeManipulationTrajectoryParameters>& parameters,
    const TimedConstraintVector& user_constraints,
    const std::vector<math::BsplineCurve<double>> seed_trajectories) {
  const int num_desired_configurations =
      static_cast<int>(desired_configurations.size());
  for (size_t i = 0; i < desired_configurations.size(); i++) {
    const auto q = desired_configurations[i];
    if (!robot.SatisfiesJointPositionLimits(q)) {
      drake::log()->warn(
          "Requested desired_configurations[{}] = [{}] is out of bound.", i,
          q.transpose());
      return nullopt;
    }
    drake::log()->debug("Requested desired_configurations[{}] = [{}]\n.", i,
                        q.transpose());
  }

  std::vector<double> actual_durations =
      durations ? durations.value()
                : GuessDurations(robot, desired_configurations);
  DRAKE_THROW_UNLESS(actual_durations.size() ==
                     desired_configurations.size() - 1);

  // Create the KinematicTrajectoryOptimization object. This constrains the
  // inital configuration.
  KinematicTrajectoryOptimization program{MakeKinematicTrajectoryOptimization(
      robot, desired_configurations.front() /* q0 */, user_constraints,
      parameters)};

  // Constrain the remaining configuration waypoints.
  VectorX<double> plan_fraction;
  double total_duration;
  std::tie(plan_fraction, total_duration) =
      DurationsToPlanFractionAndTotalDuration(actual_durations);

  for (int i = 1; i < num_desired_configurations; ++i) {
    program.AddFixedPositionConstraint(desired_configurations.at(i),
                                       plan_fraction(i - 1));
  }

  // Check user constraints at desired configurations.
  const int num_user_constraints = user_constraints.size();
  for (int i = 0; i < num_user_constraints; ++i) {
    const double t_start = user_constraints.at(i).second[0];
    const double t_end = user_constraints.at(i).second[1];
    for (int j = 0; j < num_desired_configurations; ++j) {
      const double t = plan_fraction(j);
      if (t >= t_start && t <= t_end &&
          !user_constraints.at(i).first->CheckSatisfied(
              desired_configurations.at(j))) {
        drake::log()->warn(
            "Requested desired_configurations[{}] = [{}]\nviolates "
            "user_constraints[{}].",
            j, desired_configurations.at(j).transpose(), i);
        return nullopt;
      }
    }
  }

  // Check collision avoidance constraint at desired configurations.
  if (parameters.value_or(OptimizeManipulationTrajectoryParameters())
          .collision_avoidance_threshold > 0) {
    std::shared_ptr<Constraint> min_distance_constraint =
        robot.MakeCollisionAvoidanceConstraint(
            parameters.value_or(OptimizeManipulationTrajectoryParameters())
                .collision_avoidance_threshold);
    for (int i = 0; i < num_desired_configurations; ++i) {
      if (!min_distance_constraint->CheckSatisfied(
              desired_configurations.at(i))) {
        drake::log()->warn(
            "Requested desired_configurations[{}] = [{}]\nis in collision", i,
            desired_configurations.at(i).transpose());
        return nullopt;
      }
    }
  }

  std::vector<math::BsplineCurve<double>> seed_trajectories_local =
      seed_trajectories;
  seed_trajectories_local.reserve(seed_trajectories.size() + 1);
  seed_trajectories_local.emplace_back(
      math::BsplineBasis(program.spline_order(), program.num_control_points()),
      std::vector<Eigen::MatrixXd>(program.num_control_points(),
                                   desired_configurations.front()));

  return SolveKinematicTrajectoryOptimization(
      durations ? drake::optional<double>(total_duration) : drake::nullopt,
      program, seed_trajectories_local);
}

optional<BsplineCurve<double>> OptimizeTrajectoryThroughDesiredToolPoses(
    const KinematicTree& robot,
    const std::vector<Transform<double>>& desired_tool_poses,
    const std::string& tool_frame_name,
    const drake::optional<Eigen::VectorXd>& q0,
    const drake::optional<std::vector<double>>& durations,
    const drake::optional<std::string>& reference_frame_name,
    const drake::optional<OptimizeManipulationTrajectoryParameters>& parameters,
    const TimedConstraintVector& user_constraints,
    const std::vector<math::BsplineCurve<double>> seed_trajectories) {
  if (q0) {
    drake::log()->debug("Requested q0 = [{}]\n", q0->transpose());
  }
  const int num_desired_tool_poses =
      static_cast<int>(desired_tool_poses.size());
  for (int i = 0; i < num_desired_tool_poses; i++) {
    const auto X_RT = desired_tool_poses[i];
    drake::log()->debug("Requested desired_tool_poses[{}] =\n{}.", i,
                        X_RT.GetAsMatrix4());
  }

  std::vector<double> actual_durations =
      durations ? durations.value()
                : std::vector<double>(desired_tool_poses.size(), 1.0);
  DRAKE_THROW_UNLESS(static_cast<int>(actual_durations.size()) ==
                     num_desired_tool_poses);

  Eigen::VectorXd plan_fraction;
  double total_duration;
  std::tie(plan_fraction, total_duration) =
      DurationsToPlanFractionAndTotalDuration(actual_durations);

  const OptimizeManipulationTrajectoryParameters actual_parameters{
      parameters.value_or(OptimizeManipulationTrajectoryParameters())};

  // Create the KinematicTrajectoryOptimization object. This constrains the
  // inital configuration (if one was specified).
  KinematicTrajectoryOptimization program{MakeKinematicTrajectoryOptimization(
      robot, q0, user_constraints, actual_parameters)};

  // Add constraints for desired tool pose waypoints.
  TimedConstraintVector constraint_vector = user_constraints;
  if (actual_parameters.orientation_tolerance > 0 ||
      actual_parameters.position_tolerance > 0) {
    for (int i = 0; i < num_desired_tool_poses; ++i) {
      program.AddGenericPositionConstraint(
          robot.MakeRelativePoseConstraint(
              reference_frame_name.value_or("world"), tool_frame_name,
              Transform<double>(desired_tool_poses[i]),
              actual_parameters.orientation_tolerance,
              actual_parameters.position_tolerance),
          {{plan_fraction(i), plan_fraction(i)}});
    }
  }

  std::default_random_engine rand_generator{1234};
  int num_random_seeds = 5;
  std::vector<Eigen::VectorXd> q_seeds{
      q0.value_or(robot.GetZeroConfiguration())};
  for (int i = 0; i < num_random_seeds; ++i) {
    q_seeds.push_back(robot.GetRandomConfiguration(&rand_generator));
  }
  std::vector<math::BsplineCurve<double>> seed_trajectories_local =
      seed_trajectories;
  seed_trajectories_local.reserve(seed_trajectories.size() + q_seeds.size());
  for (const auto& q_seed : q_seeds) {
    seed_trajectories_local.emplace_back(
        math::BsplineBasis(actual_parameters.spline_order,
                           actual_parameters.num_control_points),
        std::vector<Eigen::MatrixXd>(actual_parameters.num_control_points,
                                     q_seed));
  }

  return SolveKinematicTrajectoryOptimization(
      durations ? drake::optional<double>(total_duration) : drake::nullopt,
      program, seed_trajectories_local);
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
