#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace planning {
/**
 * Wrapper for a user-provided constraint and influence + safety thresholds.
 * Enforces that the provided constraint owns its own contexts to avoid thread
 * safety issues.
 */
class UserConstraintAndThresholds {
 private:
  std::shared_ptr<solvers::Constraint> constraint_;
  double influence_threshold_ = 0.0;
  double safety_threshold_ = 0.0;

 public:
  UserConstraintAndThresholds(
      const std::shared_ptr<solvers::Constraint>& constraint,
      const double influence_threshold, const double safety_threshold)
      : constraint_(constraint), influence_threshold_(influence_threshold),
        safety_threshold_(safety_threshold) {
    DRAKE_THROW_UNLESS(!!constraint_);
    DRAKE_THROW_UNLESS(safety_threshold_ >= 0.0);
    DRAKE_THROW_UNLESS(influence_threshold_ > safety_threshold_);
  }

  const std::shared_ptr<solvers::Constraint>& constraint() const {
    return constraint_;
  }

  double influence_threshold() const { return influence_threshold_; }

  double safety_threshold() const { return safety_threshold_; }
};

class AnytimeBsplineSmoother {
 public:
  /**
   * Callback type for position constraints. The input is a position vector. The
   * first element of the returned pair should a vector of constraint values.
   * The second element of the returned pair should be the Jacobian of the
   * constraint values with respect to the position vector.
   */
  using PositionConstraintCallback =
      std::function<std::pair<VectorX<double>, MatrixX<double>>(
          const VectorX<double>&)>;

  /**
   * Type for callbacks that are called with the results of a smoothing step.
   */
  using PostSmoothingStepCallback = std::function<void(
      const trajectories::BsplineTrajectory<double>&)>;

  /**
   * Constructs an AnytimeBsplineSmoother for vector-valued curves with
   * `num_positions` elements.
   */
  explicit AnytimeBsplineSmoother(int num_positions);

  /**
   * Adds a position constraint that the smoother will enforce on all smoothing
   * steps. The constraint is considered to be satisfied for a position vector
   * `q` if `(callback(q).array() >= safety_threshold).all() == true`. Position
   * constraints are potentially enforced at `num_validation_points()` points,
   * linearly spaced between the start and end times of the curve. A given
   * constraint is enforced at the validation point `t_i` if
   * `(callback(curve.value(t_i)).array() <= influence_threshold).any() ==
   * true`, where `curve` is the input to the smoothing step.
   */
  void AddPositionConstraint(PositionConstraintCallback callback,
                             double influence_threshold,
                             double safety_threshold);

  void AddPositionConstraint(
      const UserConstraintAndThresholds& constraint);

  /**
   * Sets the post-smoothing-step callback, replacing any previously set
   * callback.
   */
  void SetPostSmoothingStepCallback(PostSmoothingStepCallback callback);

  /**
   * Number of elements in the position vector.
   */
  int num_positions() const;

  /**
   * Number of elements in the velocity vector. This is an alias for
   * `num_positions().
   */
  int num_velocities() const;

  /**
   * Number of points at which position constraints may be enforced.
   * @see AddPositionConstraint()
   */
  int num_validation_points() const;

  /**
   * Weight for quadratic costs on the control points of the first-derivative
   * curve.
   */
  double first_derivative_cost_weight() const;

  /**
   * Weight for quadratic costs on the control points of the second-derivative
   * curve.
   */
  double second_derivative_cost_weight() const;

  /**
   * Weight for quadratic costs on the control points of the third-derivative
   * curve.
   */
  double third_derivative_cost_weight() const;

  /**
   * Weight for a linear cost on the duration of the curve.
   */
  double duration_cost_weight() const;

  /**
   * Smoothing steps for which all changes are below this threshold are
   * considered to have converged.
   */
  double convergence_threshold() const;

  /**
   * Minimum allowed duration.
   */
  double min_duration() const;

  /**
   * Maximum allowed duration.
   */
  double max_duration() const;

  /**
   * Tolerance for violations of position limits.
   */
  double position_limit_tolerance() const;

  /**
   * Maximum allowed smoothing steps for SmoothCurveBlocking().
   */
  int max_smoothing_iterations() const;

  /**
   * Lower bound on position for all points on the curve.
   */
  const VectorX<double>& position_lower_limit() const;

  /**
   * Upper bound on position for all points on the curve.
   */
  const VectorX<double>& position_upper_limit() const;

  /**
   * Lower bound on velocity for all points on the curve.
   */
  const VectorX<double>& velocity_lower_limit() const;

  /**
   * Upper bound on velocity for all points on the curve.
   */
  const VectorX<double>& velocity_upper_limit() const;

  const VectorX<double>& acceleration_lower_limit() const;

  const VectorX<double>& acceleration_upper_limit() const;

  void SetPositionLimits(const VectorX<double>& lower_limit,
                         const VectorX<double>& upper_limit);

  void SetVelocityLimits(const VectorX<double>& lower_limit,
                         const VectorX<double>& upper_limit);

  void SetAccelerationLimits(const VectorX<double>& lower_limit,
                             const VectorX<double>& upper_limit);

  void SetDurationLimits(double min_duration, double max_duration);

  void set_convergence_threshold(double convergence_threshold);

  void set_first_derivative_cost_weight(double first_derivative_cost_weight);

  void set_second_derivative_cost_weight(double second_derivative_cost_weight);

  void set_third_derivative_cost_weight(double third_derivative_cost_weight);

  void set_duration_cost_weight(double duration_cost_weight);

  void set_synthetic_timestep(double synthetic_timestep);

  void set_position_limit_tolerance(double position_limit_tolerance);

  void set_num_validation_points(int num_validation_points);

  void set_max_smoothing_iterations(int max_smoothing_iterations);

  /**
   * Performs a single smoothing step on `curve`. All smoothed curve is
   * guaranteed to be the same as the original curve up to `lock_time`.
   */
  std::shared_ptr<const trajectories::BsplineTrajectory<double>>
  DoSingleSmoothingStep(
      const trajectories::BsplineTrajectory<double>& curve,
      double lock_time = -1) const;

  /**
   * Iteratively smooths `curve` until all changes fall below the convergence
   * threshold or the maximum number of iterations is reached.
   * @returns the smoothed curve.
   */
  std::shared_ptr<const trajectories::BsplineTrajectory<double>>
  SmoothCurveBlocking(
      const trajectories::BsplineTrajectory<double>& curve) const;

  /**
   * Iteratively smooths `curve` until the estimated time for the end of the
   * next smoothing step is after the end time of the curve or `current_time` is
   * less than the start time of the curve. Meant for asynchronous use, in which
   * another thread updates `current_time` and reads `curve` and `derivative`.
   */
  void SmoothCurve(
      const std::atomic<double>& current_time,
      std::shared_ptr<const trajectories::BsplineTrajectory<double>>*
          curve,
      std::unique_ptr<const trajectories::Trajectory<double>>*
          derivative,
      std::mutex* curve_update_mutex,
      double estimated_smoothing_step_duration = 0) const;

  /**
   * Generate a B-spline curve that interpolates between the waypoints in
   * `path`, stopping at each waypoint.
   * @param spline_order Order of the returned B-spline curve. Note that a
   *   B-spline curve of order k is a piecewise polynomial of degree k - 1.
   * @param num_intermediate_control_points Number of control points inserted
   *   along the line segment between each pair of waypoints in `path`.
   * @param max_duration A suggested upper bound for the duration of the curve.
   *   If a curve cannot be found that interpolates the waypoints while
   *   satisfying this bound along with the position, velocity, and
   *   acceleration limits, the bound will be increased until a solution is
   *   found.
   */
  trajectories::BsplineTrajectory<double> StraightLineBsplineFromPath(
      std::vector<VectorX<double>> path, int spline_order,
      int num_intermediate_control_points = 2, double max_duration = 10) const;

  /**
   * Checks whether `curve` satisfies the constraints defined by the smoother.
   */
  bool CheckConstraints(
      const trajectories::BsplineTrajectory<double>& curve,
      double lock_time = -1) const;

 private:
  std::vector<double> GuessDurations(
      const std::vector<VectorX<double>>& desired_configurations) const;

  struct PositionConstraint {
    PositionConstraintCallback callback;
    double influence_threshold;
    double safety_threshold;
  };

  std::vector<double> ComputeActualSafetyThresholds(
      const PositionConstraint& position_constraint,
      const trajectories::BsplineTrajectory<double>& original_curve)
      const;

  const int num_positions_;

  PostSmoothingStepCallback post_smoothing_step_callback_;
  std::vector<PositionConstraint> position_constraints_;
  std::vector<UserConstraintAndThresholds> user_constraints_;
  int num_validation_points_{100};
  int max_smoothing_iterations_{100};
  double convergence_threshold_{1e-3};
  double synthetic_timestep_{0.125};
  double min_duration_{0.01};
  double max_duration_{10};
  bool fix_initial_point_{true};
  bool fix_final_point_{true};
  double first_derivative_cost_weight_{-1};
  double second_derivative_cost_weight_{-1};
  double third_derivative_cost_weight_{-1};
  double duration_cost_weight_{-1};
  double position_limit_tolerance_{1e-6};

  VectorX<double> position_lower_limit_{};
  VectorX<double> position_upper_limit_{};
  VectorX<double> velocity_lower_limit_{};
  VectorX<double> velocity_upper_limit_{};
  VectorX<double> acceleration_lower_limit_{};
  VectorX<double> acceleration_upper_limit_{};
};

}  // namespace planning
}  // namespace drake
