#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/planner/kinematic_tree.h"
#include "drake/math/bspline_curve.h"
#include "drake/math/transform.h"

namespace drake {
namespace manipulation {
namespace planner {

/// Represents a collection of constraints, each of which is active over some
/// interval within [0, 1].
typedef std::vector<std::pair<std::shared_ptr<drake::solvers::Constraint>,
                              std::array<double, 2>>>
    TimedConstraintVector;

/// Struct that stores the parameters of OptimizeManipulationTrajectory() that a
/// user may or may not want to change.
struct OptimizeManipulationTrajectoryParameters {
  OptimizeManipulationTrajectoryParameters() {}
  /// Order of the Bspline curve used to represent the position trajectory.
  int spline_order{4};
  /// Initial number of control points in the Bspline curve used to represent
  /// the position trajectory. Additional control points will be added if
  /// necessary.
  int num_control_points{7};
  /// Number of evenly spaced points along the position trajectory at which
  /// generic position contstraints will be validated.
  int num_validation_points{100};
  /// Weight to be applied to the quadratic cost on normalized velocity (first
  /// derivative of the position with respect to dimensionless time). Negative
  /// values indicate that this term should be omitted from the cost.
  double velocity_weight{-1};
  /// Weight to be applied to the quadratic cost on normalized acceleration
  /// (second derivative of the position with respect to dimensionless time).
  /// Negative values indicate that this term should be omitted from the cost.
  double acceleration_weight{1};
  /// Weight to be applied to the quadratic cost on normalized jerk (third
  /// derivative of the position with respect to dimensionless time). Negative
  /// values indicate that this term should be omitted from the cost.
  double jerk_weight{-1};
  /// Weight to be applied to the linear cost on the duration of the
  /// trajectory.Negative values indicate that this term should be omitted from
  /// the cost.
  double duration_weight{1};
  /// Minimum distance to maintain between the robot and the environment [m]
  double collision_avoidance_threshold{0.01};
  /// Orientation tolerance for grasps [radians]
  double orientation_tolerance{1 * M_PI / 180};
  /// Position tolerance for grasps [m]
  double position_tolerance{1e-3};
  /// Minimum duration of the position trajectory [s].
  double min_duration{0.5};
  /// Maximum duration of the position trajectory [s].
  double max_duration{5};
};

/// Attempts to find an optimal solution for the problem specified by `q0`,
/// `parameters`, and `user_constraints`.
/// @param[in] tree
///   KinematicTree object representing the robot and its environment.
/// @param[in] q0
///   Optional initial configuration of `tree`. If specified, the solution must
///   start at this configuration. If nullopt, the optimizer may choose the
///   initial configuration.
/// @param[in] parameters
///   Optional OptimizeManipulationTrajectoryParameters object. If nullopt, a
///   default constructed parameter object will be used.
/// @param[in] user_constraints
///   Collection of position constraints, with associated active
///   intervals on [0, 1], where 0 is the start of the trajectory and 1 is the
///   end.The number of variables for each constraint must be
///   `tree.num_positions()`)
/// @param[in] seed_trajectories
///   Collection of B-spline curves to be used as seeds for solving the
///   optimization problem.
/// @returns a B-spline curve if the optimization is successful, and nullopt
/// otherwise.
drake::optional<math::BsplineCurve<double>> OptimizeTrajectory(
    const KinematicTree& tree, const drake::optional<Eigen::VectorXd>& q0 = {},
    const drake::optional<OptimizeManipulationTrajectoryParameters>&
        parameters = {},
    const TimedConstraintVector& user_constraints = {},
    const std::vector<math::BsplineCurve<double>> seed_trajectories = {});

/// Attempts to find a trajectory that hits the joint-space waypoints in
/// `desired_configurations`.
/// @param[in] tree
///   KinematicTree object representing the robot and its environment.
/// @param[in] desired_configurations
///   Vector of joint-space waypoints through which the trajectory must pass.
/// @param[in] durations
///   Optional vector specifying the minimum duration between the waypoints in
///   `desired_configurations`.
/// @param[in] parameters
///   Optional OptimizeManipulationTrajectoryParameters object. If nullopt, a
///   default constructed parameter object will be used.
/// @param[in] user_constraints
///   Collection of position constraints, with associated active
///   intervals on [0, 1], where 0 is the start of the trajectory and 1 is the
///   end.The number of variables for each constraint must be
///   `tree.num_positions()`)
/// @param[in] seed_trajectories
///   Collection of B-spline curves to be used as seeds for solving the
///   optimization problem.
/// @returns a B-spline curve if the optimization is successful, and nullopt
/// otherwise.
drake::optional<math::BsplineCurve<double>>
OptimizeTrajectoryThroughDesiredConfigurations(
    const KinematicTree& tree,
    const std::vector<Eigen::VectorXd>& desired_configurations,
    const drake::optional<std::vector<double>>& durations = {},
    const drake::optional<OptimizeManipulationTrajectoryParameters>&
        parameters = {},
    const TimedConstraintVector& user_constraints = {},
    const std::vector<math::BsplineCurve<double>> seed_trajectories = {});

/// Attempts to find a trajectory for which the pose of the tool frame relative
/// to the reference frame hits the waypoints in `X_RT_desired_sequence`.
/// @param[in] tree
///   KinematicTree object representing the robot and its environment.
/// @param[in] X_RT_desired_sequence
///   Vector of pose waypoints for the tool frame, relative to the reference
///   frame.
/// @param[in] tool_frame_name
///   Name of the tool frame, T.
/// @param[in] q0
///   Optional initial configuration of `tree`. If specified, the solution must
///   start at this configuration. If nullopt, the optimizer may choose the
///   initial configuration.
/// @param[in] durations
///   Optional vector specifying the minimum duration between the waypoints in
///   `X_RT_desired_sequence`.
/// @param[in] reference_frame_name
///   Name of the reference frame, R. If nullopt, R is the world frame.
/// @param[in] parameters
///   Optional OptimizeManipulationTrajectoryParameters object. If nullopt, a
///   default constructed parameter object will be used.
/// @param[in] user_constraints
///   Collection of position constraints, with associated active
///   intervals on [0, 1], where 0 is the start of the trajectory and 1 is the
///   end.The number of variables for each constraint must be
///   `tree.num_positions()`)
/// @param[in] seed_trajectories
///   Collection of B-spline curves to be used as seeds for solving the
///   optimization problem.
/// @returns a B-spline curve if the optimization is successful, and nullopt
/// otherwise.
drake::optional<math::BsplineCurve<double>>
OptimizeTrajectoryThroughDesiredToolPoses(
    const KinematicTree& tree,
    const std::vector<drake::math::Transform<double>>& X_RT_desired_sequence,
    const std::string& tool_frame_name,
    const drake::optional<Eigen::VectorXd>& q0 = {},
    const drake::optional<std::vector<double>>& durations = {},
    const drake::optional<std::string>& reference_frame_name = {},
    const drake::optional<OptimizeManipulationTrajectoryParameters>&
        parameters = {},
    const TimedConstraintVector& user_constraints = {},
    const std::vector<math::BsplineCurve<double>> seed_trajectories = {});

namespace test {
/** Struct containing the information necessary to reproduce a call to
 * `OptimizeTrajectoryThroughDesiredConfigurations()`.
 */
struct OptimizeManipulationTrajectoryTestParameter {
  std::string tool_frame_body_name{};
  Eigen::Isometry3d X_BT{Eigen::Isometry3d::Identity()};  // Pose of Tool frame
                                                          // relative to Body
                                                          // frame.
  Eigen::VectorXd q0;                  // Initial configuration.
  std::vector<Eigen::VectorXd> q_des;  // Desired configurations.
  std::vector<double> durations;  // Desired duration between configurations.
};
}  // namespace test
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
