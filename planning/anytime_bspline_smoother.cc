#include "drake/planning/anytime_bspline_smoother.h"

#include <algorithm>
#include <mutex>

#include "drake/common/drake_throw.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/bspline_basis.h"
#include "drake/math/knot_vector_type.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {

using math::BsplineBasis;
using math::initializeAutoDiff;
using math::KnotVectorType;
using solvers::MathematicalProgram;
using solvers::SolutionResult;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using trajectories::BsplineTrajectory;

std::vector<double> AnytimeBsplineSmoother::GuessDurations(
    const std::vector<VectorX<double>>& desired_configurations) const {
  std::vector<double> durations;
  durations.reserve(desired_configurations.size() - 1);
  for (auto q = std::next(desired_configurations.begin());
       q != desired_configurations.end(); ++q) {
    DRAKE_ASSERT(q->size() == num_positions_);
    VectorX<double> diff{q - std::prev(q)};

    for (int j = 0; j < diff.size(); j++) {
      if (diff(j) < 0) {
        diff(j) /= velocity_lower_limit_(j);
      } else {
        diff(j) /= velocity_upper_limit_(j);
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

AnytimeBsplineSmoother::AnytimeBsplineSmoother(int num_positions)
    : num_positions_(num_positions) {
  position_lower_limit_ = VectorX<double>::Zero(this->num_positions());
  position_upper_limit_ = VectorX<double>::Zero(this->num_positions());
  velocity_lower_limit_ = VectorX<double>::Zero(this->num_velocities());
  velocity_upper_limit_ = VectorX<double>::Zero(this->num_velocities());
  acceleration_lower_limit_ = VectorX<double>::Zero(this->num_velocities());
  acceleration_upper_limit_ = VectorX<double>::Zero(this->num_velocities());
}

void AnytimeBsplineSmoother::set_convergence_threshold(
    double convergence_threshold) {
  convergence_threshold_ = convergence_threshold;
}

void AnytimeBsplineSmoother::SetDurationLimits(double min_duration,
                                               double max_duration) {
  DRAKE_THROW_UNLESS(0 <= min_duration);
  DRAKE_THROW_UNLESS(min_duration <= max_duration);
  min_duration_ = min_duration;
  max_duration_ = max_duration;
}

void AnytimeBsplineSmoother::set_first_derivative_cost_weight(
    double first_derivative_cost_weight) {
  first_derivative_cost_weight_ = first_derivative_cost_weight;
}

void AnytimeBsplineSmoother::set_second_derivative_cost_weight(
    double second_derivative_cost_weight) {
  second_derivative_cost_weight_ = second_derivative_cost_weight;
}

void AnytimeBsplineSmoother::set_third_derivative_cost_weight(
    double third_derivative_cost_weight) {
  third_derivative_cost_weight_ = third_derivative_cost_weight;
}

void AnytimeBsplineSmoother::set_duration_cost_weight(
    double duration_cost_weight) {
  duration_cost_weight_ = duration_cost_weight;
}

void AnytimeBsplineSmoother::set_synthetic_timestep(double synthetic_timestep) {
  synthetic_timestep_ = synthetic_timestep;
}

void AnytimeBsplineSmoother::set_position_limit_tolerance(
    double position_limit_tolerance) {
  position_limit_tolerance_ = position_limit_tolerance;
}

void AnytimeBsplineSmoother::set_num_validation_points(
    int num_validation_points) {
  num_validation_points_ = num_validation_points;
}

void AnytimeBsplineSmoother::set_max_smoothing_iterations(
    int max_smoothing_iterations) {
  max_smoothing_iterations_ = max_smoothing_iterations;
}

int AnytimeBsplineSmoother::num_positions() const { return num_positions_; }

int AnytimeBsplineSmoother::num_velocities() const { return num_positions(); }

int AnytimeBsplineSmoother::num_validation_points() const {
  return num_validation_points_;
}

double AnytimeBsplineSmoother::first_derivative_cost_weight() const {
  return first_derivative_cost_weight_;
}

double AnytimeBsplineSmoother::second_derivative_cost_weight() const {
  return second_derivative_cost_weight_;
}

double AnytimeBsplineSmoother::third_derivative_cost_weight() const {
  return third_derivative_cost_weight_;
}

double AnytimeBsplineSmoother::duration_cost_weight() const {
  return duration_cost_weight_;
}

double AnytimeBsplineSmoother::convergence_threshold() const {
  return convergence_threshold_;
}

double AnytimeBsplineSmoother::min_duration() const { return min_duration_; }

double AnytimeBsplineSmoother::max_duration() const { return max_duration_; }

double AnytimeBsplineSmoother::position_limit_tolerance() const {
  return position_limit_tolerance_;
}

int AnytimeBsplineSmoother::max_smoothing_iterations() const {
  return max_smoothing_iterations_;
}

const VectorX<double>& AnytimeBsplineSmoother::position_lower_limit()
    const {
  return position_lower_limit_;
}

const VectorX<double>& AnytimeBsplineSmoother::position_upper_limit()
    const {
  return position_upper_limit_;
}

const VectorX<double>& AnytimeBsplineSmoother::velocity_lower_limit()
    const {
  return velocity_lower_limit_;
}

const VectorX<double>& AnytimeBsplineSmoother::velocity_upper_limit()
    const {
  return velocity_upper_limit_;
}

const VectorX<double>& AnytimeBsplineSmoother::acceleration_lower_limit()
    const {
  return acceleration_lower_limit_;
}

const VectorX<double>& AnytimeBsplineSmoother::acceleration_upper_limit()
    const {
  return acceleration_upper_limit_;
}

void AnytimeBsplineSmoother::AddPositionConstraint(
    PositionConstraintCallback callback, double influence_threshold,
    double safety_threshold) {
  position_constraints_.push_back(
      {callback, influence_threshold, safety_threshold});
}

void AnytimeBsplineSmoother::AddPositionConstraint(
    const UserConstraintAndThresholds& constraint) {
  user_constraints_.push_back(constraint);
}

void AnytimeBsplineSmoother::SetPostSmoothingStepCallback(
    PostSmoothingStepCallback callback) {
  post_smoothing_step_callback_ = callback;
}

void AnytimeBsplineSmoother::SetPositionLimits(
    const VectorX<double>& lower_limit,
    const VectorX<double>& upper_limit) {
  DRAKE_THROW_UNLESS(lower_limit.size() == num_positions());
  DRAKE_THROW_UNLESS(upper_limit.size() == num_positions());
  DRAKE_THROW_UNLESS((lower_limit.array() <= upper_limit.array()).all());
  position_lower_limit_ = lower_limit;
  position_upper_limit_ = upper_limit;
  log()->trace("Set position lower limit to {}",
                      position_lower_limit_.transpose());
  log()->trace("Set position upper limit to {}",
                      position_upper_limit_.transpose());
}

void AnytimeBsplineSmoother::SetVelocityLimits(
    const VectorX<double>& lower_limit,
    const VectorX<double>& upper_limit) {
  DRAKE_THROW_UNLESS(lower_limit.size() == num_velocities());
  DRAKE_THROW_UNLESS(upper_limit.size() == num_velocities());
  DRAKE_THROW_UNLESS((lower_limit.array() <= upper_limit.array()).all());
  velocity_lower_limit_ = lower_limit;
  velocity_upper_limit_ = upper_limit;
  log()->trace("Set velocity lower limit to {}",
                      velocity_lower_limit_.transpose());
  log()->trace("Set velocity upper limit to {}",
                      velocity_upper_limit_.transpose());
}

void AnytimeBsplineSmoother::SetAccelerationLimits(
    const VectorX<double>& lower_limit,
    const VectorX<double>& upper_limit) {
  DRAKE_THROW_UNLESS(lower_limit.size() == num_velocities());
  DRAKE_THROW_UNLESS(upper_limit.size() == num_velocities());
  DRAKE_THROW_UNLESS((lower_limit.array() <= upper_limit.array()).all());
  acceleration_lower_limit_ = lower_limit;
  acceleration_upper_limit_ = upper_limit;
  log()->trace("Set acceleration lower limit to {}",
                      acceleration_lower_limit_.transpose());
  log()->trace("Set acceleration upper limit to {}",
                      acceleration_upper_limit_.transpose());
}

BsplineTrajectory<double> AnytimeBsplineSmoother::StraightLineBsplineFromPath(
    std::vector<VectorX<double>> path, const int spline_order,
    const int num_intermediate_control_points,
    const double max_duration) const {
  log()->trace("StraightLineBsplineFromPath START");
  const int num_waypoints = static_cast<int>(path.size());
  const int num_control_points =
      num_waypoints * (spline_order - 1) +
      num_intermediate_control_points * (num_waypoints - 1);
  std::vector<MatrixX<double>> control_points{};
  control_points.reserve(num_control_points);
  int waypoint_index = 0;
  for (const auto& waypoint : path) {
    DRAKE_THROW_UNLESS(static_cast<int>(waypoint.size()) == num_positions());
    for (int i = 0; i < spline_order - 1; ++i) {
      control_points.push_back(waypoint);
    }
    if (waypoint_index < num_waypoints - 1) {
      for (int i = 0; i < num_intermediate_control_points; ++i) {
        const double s =
            static_cast<double>(i + 1) /
            static_cast<double>(num_intermediate_control_points + 1);
        control_points.push_back((1 - s) * waypoint +
                                 s * path[waypoint_index + 1]);
      }
    }
    ++waypoint_index;
  }
  log()->trace("StraightLineBsplineFromPath Added {} control points",
                      control_points.size());
  BsplineTrajectory<double> curve{
      BsplineBasis<double>(spline_order, num_control_points,
                           KnotVectorType::kUniform),
      control_points};
  const int num_knots = static_cast<int>(curve.basis().knots().size());
  const int order = curve.basis().order();
  double limit_scaling_factor = 0.95;
  MathematicalProgram knot_optimization{};
  VectorXDecisionVariable knot_vars =
      knot_optimization.NewContinuousVariables(num_knots, 1, "t");
  // Knots must be monotonic.
  const double minimum_knot_spacing{0.3};
  const double maximum_knot_spacing{max_duration / (num_knots - 1)};
  for (int i = 0; i < num_knots - 1; ++i) {
    knot_optimization.AddLinearConstraint(knot_vars(i + 1) - knot_vars(i) >=
                                          minimum_knot_spacing);
    knot_optimization.AddLinearConstraint(knot_vars(i + 1) - knot_vars(i) <=
                                          maximum_knot_spacing);
    // Minimize inter-knot spacing.
    knot_optimization.AddQuadraticCost((knot_vars(i + 1) - knot_vars(i)) *
                                       (knot_vars(i + 1) - knot_vars(i)));
  }
  log()->trace("StraightLineBsplineFromPath Made knots monotonic");
  // Fix start time to 0.
  symbolic::Variable& start_time = knot_vars(order - 1);
  knot_optimization.AddLinearConstraint(start_time == 0);
  log()->trace("StraightLineBsplineFromPath Fixed start time to 0");

  // Velocity limits.
  for (int i = 0; i < num_control_points - 1; ++i) {
    knot_optimization.AddLinearConstraint(
        limit_scaling_factor * velocity_lower_limit() *
            (knot_vars(i + order) - knot_vars(i + 1)) <=
        (order - 1) *
            (curve.control_points()[i + 1] - curve.control_points()[i]));
    knot_optimization.AddLinearConstraint(
        limit_scaling_factor * velocity_upper_limit() *
            (knot_vars(i + order) - knot_vars(i + 1)) >=
        (order - 1) *
            (curve.control_points()[i + 1] - curve.control_points()[i]));
  }
  log()->trace("StraightLineBsplineFromPath Added velocity limits");

  // Acceleration limits.
  const int v_order = order - 1;
  for (int i = 1; i < num_control_points - 1; ++i) {
    // Velocity control point i if the spacing between adjacent knots is
    // uniformly minimum_knot_spacing.
    VectorX<double> v_i_minimum_knot_spacing =
        (curve.control_points()[i] - curve.control_points()[i - 1]) /
        minimum_knot_spacing;
    // Velocity control point i if the spacing between adjacent knots is
    // uniformly maximum_knot_spacing.
    VectorX<double> v_i_maximum_knot_spacing =
        (curve.control_points()[i] - curve.control_points()[i - 1]) /
        maximum_knot_spacing;
    // Velocity control point i + 1 if the spacing between adjacent knots is
    // uniformly minimum_knot_spacing.
    VectorX<double> v_i_plus_minimum_knot_spacing =
        (curve.control_points()[i + 1] - curve.control_points()[i]) /
        minimum_knot_spacing;
    // Velocity control point i + 1 if the spacing between adjacent knots is
    // uniformly maximum_knot_spacing.
    VectorX<double> v_i_plus_maximum_knot_spacing =
        (curve.control_points()[i + 1] - curve.control_points()[i]) /
        maximum_knot_spacing;

    // Lower bound on velocity control point i given the constraints on velocity
    // and inter-knot spacing.
    VectorX<double> v_i_min = velocity_lower_limit().cwiseMax(
        v_i_minimum_knot_spacing.cwiseMin(v_i_maximum_knot_spacing));

    // Lower bound on velocity control point i + 1 given the constraints on
    // velocity and inter-knot spacing.
    VectorX<double> v_i_plus_min = velocity_lower_limit().cwiseMax(
        v_i_plus_minimum_knot_spacing.cwiseMin(v_i_plus_maximum_knot_spacing));

    // Lower bound on velocity control point i given the constraints on velocity
    // and inter-knot spacing.
    VectorX<double> v_i_max = velocity_upper_limit().cwiseMin(
        v_i_minimum_knot_spacing.cwiseMax(v_i_maximum_knot_spacing));

    // Lower bound on velocity control point i + 1 given the constraints on
    // velocity and inter-knot spacing.
    VectorX<double> v_i_plus_max = velocity_upper_limit().cwiseMin(
        v_i_plus_minimum_knot_spacing.cwiseMax(v_i_plus_maximum_knot_spacing));

    log()->trace("Adding acceleration limits. i = {}", i);
    log()->trace("  v_i_min      = {}", v_i_min.transpose());
    log()->trace("  v_i_plus_min = {}", v_i_plus_min.transpose());
    log()->trace("  v_i_max      = {}", v_i_max.transpose());
    log()->trace("  v_i_plus_max = {}", v_i_plus_max.transpose());
    log()->trace("  {}",
                        limit_scaling_factor * acceleration_lower_limit() *
                                (knot_vars(i + v_order) - knot_vars(i + 1)) <=
                            (v_order - 1) * (v_i_plus_min - v_i_max));
    log()->trace("  {}",
                        limit_scaling_factor * acceleration_upper_limit() *
                                (knot_vars(i + v_order) - knot_vars(i + 1)) >=
                            (v_order - 1) * (v_i_plus_max - v_i_min));
    knot_optimization.AddLinearConstraint(
        limit_scaling_factor * acceleration_lower_limit() *
            (knot_vars(i + v_order) - knot_vars(i + 1)) <=
        (v_order - 1) * (v_i_plus_min - v_i_max));
    knot_optimization.AddLinearConstraint(
        limit_scaling_factor * acceleration_upper_limit() *
            (knot_vars(i + v_order) - knot_vars(i + 1)) >=
        (v_order - 1) * (v_i_plus_max - v_i_min));
  }
  log()->trace("StraightLineBsplineFromPath Added acceleration limits");

  log()->debug("Optimizing knot vector. START");
  solvers::MathematicalProgramResult result =
      solvers::Solve(knot_optimization);
  log()->debug("Solver called: {}", result.get_solver_id().name());
  log()->debug("result: {}", result.get_solution_result());
  log()->debug("Optimizing knot vector. DONE");
  if (!result.is_success()) {
    return StraightLineBsplineFromPath(
        path, spline_order, num_intermediate_control_points, 2 * max_duration);
  }
  std::vector<double> optimized_knots{};
  for (int i = 0; i < num_knots; ++i) {
    optimized_knots.push_back(result.GetSolution(knot_vars(i)));
  }
  log()->debug("optimized_knots: {}",
                      result.GetSolution(knot_vars).transpose());
  BsplineTrajectory<double> optimized_curve{
      BsplineBasis<double>(spline_order, optimized_knots), control_points};
  if (post_smoothing_step_callback_) {
    post_smoothing_step_callback_(optimized_curve);
  }
  return optimized_curve;
}

std::shared_ptr<const BsplineTrajectory<double>>
AnytimeBsplineSmoother::DoSingleSmoothingStep(
    const BsplineTrajectory<double>& curve, double lock_time) const {
  log()->debug("DoSingleSmoothingStep(). START");
  const int num_control_points = curve.num_control_points();
  const int num_knots = static_cast<int>(curve.basis().knots().size());
  const int num_velocities = num_positions();
  const int order = curve.basis().order();
  const double start_time = curve.start_time();
  double old_duration = curve.end_time() - start_time;

  // Set up locked control points and knots.
  std::vector<int> locked_control_point_indices{};
  std::vector<int> locked_knot_indices{};
  log()->trace("lock_time_ = {}", lock_time);
  int last_locked_control_point_index = -1;
  if (lock_time > curve.start_time()) {
    double clamped_lock_time = std::min<double>(lock_time, curve.end_time());
    last_locked_control_point_index =
        curve.basis()
            .ComputeActiveBasisFunctionIndices(clamped_lock_time)
            .back();
    for (int i = 0; i < last_locked_control_point_index + 1; ++i) {
      locked_control_point_indices.push_back(i);
    }
    for (int i = 0; i < last_locked_control_point_index + order + 1; ++i) {
      locked_knot_indices.push_back(i);
    }
  }
  int last_locked_knot_index =
      locked_knot_indices.empty() ? 0 : locked_knot_indices.back();
  double last_locked_knot_time = std::max<double>(
      curve.start_time(), curve.basis().knots()[last_locked_knot_index]);
  double remaining_duration =
      std::max<double>(0, curve.end_time() - last_locked_knot_time);
  log()->trace("Remaining duration: {}", remaining_duration);

  MathematicalProgram program;
  std::vector<VectorXDecisionVariable> control_point_delta_vars;
  for (int i = 0; i < num_control_points; ++i) {
    control_point_delta_vars.push_back(program.NewContinuousVariables(
        num_positions(), 1, "p_" + std::to_string(i)));
  }

  // Add duration variables. The variables representing different powers of T
  // are not constrained to match eachother. Rather the one corresponding to the
  // longest duration is chosen once the problem is solved.
  symbolic::Variable duration_var =
      program.NewContinuousVariables(1, 1, "T")(0, 0);
  program.AddLinearConstraint(min_duration_ <= duration_var &&
                              duration_var <= max_duration_);
  symbolic::Variable duration_squared_var =
      program.NewContinuousVariables(1, 1, "T_squared")(0, 0);
  program.AddLinearConstraint(
      min_duration_ * min_duration_ <= duration_squared_var &&
      duration_squared_var <= max_duration_ * max_duration_);

  log()->trace("Locking control points. START");
  for (const auto& index : locked_control_point_indices) {
    log()->trace("Locking control point {}.", index);
    program.AddBoundingBoxConstraint(VectorX<double>::Zero(num_positions()),
                                     VectorX<double>::Zero(num_positions()),
                                     control_point_delta_vars[index]);
  }
  log()->trace("Locking control points. DONE");

  if (fix_initial_point_) {
    for (int i = 0; i < order - 1; ++i) {
      program.AddBoundingBoxConstraint(VectorX<double>::Zero(num_velocities),
                                       VectorX<double>::Zero(num_velocities),
                                       control_point_delta_vars[i]);
    }
  }
  if (fix_final_point_) {
    for (int i = 0; i < order - 1; ++i) {
      program.AddBoundingBoxConstraint(
          VectorX<double>::Zero(num_velocities),
          VectorX<double>::Zero(num_velocities),
          control_point_delta_vars[num_control_points - 1 - i]);
    }
  }
  std::vector<MatrixX<Expression>> symbolic_control_points{};
  for (int i = 0; i < num_control_points; ++i) {
    symbolic_control_points.push_back(control_point_delta_vars[i] +
                                      curve.control_points()[i]);
  }
  BsplineTrajectory<Expression> symbolic_curve{curve.basis(),
                                               symbolic_control_points};

  std::unique_ptr<BsplineTrajectory<Expression>>
      symbolic_first_derivative_curve{
          dynamic_pointer_cast_or_throw<BsplineTrajectory<Expression>>(
              symbolic_curve.MakeDerivative())};

  // Set position limits
  for (const auto& control_point : symbolic_curve.control_points()) {
    program.AddLinearConstraint(control_point >= position_lower_limit_);
    program.AddLinearConstraint(control_point <= position_upper_limit_);
  }

  // Set velocity limits
  auto velocity_scaling_factor = 0.95 / (curve.end_time() - curve.start_time());
  for (int i = 0; i < num_velocities; ++i) {
    const double v_mid =
        0.5 * (velocity_upper_limit_(i) + velocity_lower_limit_(i));
    const double v_max = 0.5 * velocity_scaling_factor *
                         (velocity_upper_limit_(i) - velocity_lower_limit_(i));
    for (int j = 0; j < symbolic_first_derivative_curve->num_control_points();
         ++j) {
      if (symbolic_first_derivative_curve->basis()
              .knots()[j + symbolic_first_derivative_curve->basis().order()] >
          lock_time) {
        const Expression v =
            symbolic_first_derivative_curve->control_points().at(j)(i) - v_mid;
        program.AddLinearConstraint(-v_max * duration_var <= v &&
                                    v <= v_max * duration_var);
      }
    }
  }

  // Minimize duration. Use AddQuadraticCost() to indicate that we want a QP
  // solver.
  program.AddQuadraticCost(std::max<double>(0, duration_cost_weight_) *
                           duration_var);
  program.AddQuadraticCost(std::max<double>(0, duration_cost_weight_) *
                           duration_squared_var);

  if (first_derivative_cost_weight_ > 0 && curve.basis().order() > 1) {
    for (const auto control_point :
         symbolic_first_derivative_curve->control_points()) {
      Expression cost_term = (first_derivative_cost_weight_ *
                              control_point.transpose() * control_point)(0, 0);
      log()->trace("Adding cost term: {}", cost_term);
      program.AddQuadraticCost(cost_term);
    }
  }

  if (curve.basis().order() > 2) {
    std::unique_ptr<BsplineTrajectory<Expression>>
        symbolic_second_derivative_curve{
            dynamic_pointer_cast_or_throw<BsplineTrajectory<Expression>>(
                symbolic_curve.MakeDerivative(2))};
    auto acceleration_scaling_factor =
        velocity_scaling_factor * velocity_scaling_factor;
    for (int i = 0; i < num_velocities; ++i) {
      const double a_mid =
          0.5 * (acceleration_upper_limit_(i) + acceleration_lower_limit_(i));
      const double a_max =
          0.5 * acceleration_scaling_factor *
          (acceleration_upper_limit_(i) - acceleration_lower_limit_(i));
      for (int j = 0;
           j < symbolic_second_derivative_curve->num_control_points(); ++j) {
        if (symbolic_second_derivative_curve->basis()
                .knots()[j +
                         symbolic_second_derivative_curve->basis().order()] >
            lock_time) {
          const Expression a =
              symbolic_second_derivative_curve->control_points().at(j)(i) -
              a_mid;
          log()->trace("{}", duration_squared_var * a_max >= a);
          log()->trace("{}", -duration_squared_var * a_max <= a);
          program.AddLinearConstraint(duration_squared_var * a_max >= a);
          program.AddLinearConstraint(-duration_squared_var * a_max <= a);
        }
      }
    }
    for (const auto control_point :
         symbolic_second_derivative_curve->control_points()) {
      if (second_derivative_cost_weight_ > 0) {
        Expression cost_term =
            (second_derivative_cost_weight_ * control_point.transpose() *
             control_point)(0, 0);
        log()->trace("Adding cost term: {}", cost_term);
        program.AddQuadraticCost(cost_term);
      }
    }
  }

  if (third_derivative_cost_weight_ > 0 && curve.basis().order() > 3) {
    std::unique_ptr<BsplineTrajectory<Expression>>
        symbolic_third_derivative_curve{
            dynamic_pointer_cast_or_throw<BsplineTrajectory<Expression>>(
                symbolic_curve.MakeDerivative(3))};
    for (const auto control_point :
         symbolic_third_derivative_curve->control_points()) {
      Expression cost_term = (third_derivative_cost_weight_ *
                              control_point.transpose() * control_point)(0, 0);
      log()->trace("Adding cost term: {}", cost_term);
      program.AddQuadraticCost(cost_term);
    }
  }

  for (const auto& position_constraint : position_constraints_) {
    const auto tt = VectorX<double>::LinSpaced(
        num_validation_points(), curve.start_time(), curve.end_time());
    for (int i = 0; i < num_validation_points(); ++i) {
      VectorX<double> d;
      MatrixX<double> dd_dq;
      const double t = tt(i);
      MatrixX<double> q = curve.value(t);
      std::tie(d, dd_dq) = position_constraint.callback(q);
      if ((d.array() < position_constraint.influence_threshold).any()) {
        const auto d_s = VectorX<double>::Constant(
            d.size(), 1.01 * position_constraint.safety_threshold);
        VectorX<Expression> delta_q_symbolic = symbolic_curve.value(t) - q;
        const auto f = (d + dd_dq * delta_q_symbolic >= d_s);
        if (!symbolic::is_true(f)) {
          program.AddLinearConstraint(f);
        }
      }
    }
  }

  for (const auto& user_constraint : user_constraints_) {
    const auto tt = VectorX<double>::LinSpaced(
        num_validation_points(), curve.start_time(), curve.end_time());
    for (int i = 0; i < num_validation_points(); ++i) {
      const double t = tt(i);
      MatrixX<AutoDiffXd> q = initializeAutoDiff(curve.value(t));
      VectorX<AutoDiffXd> constraint_value{};
      user_constraint.constraint()->Eval(q, &constraint_value);
      const VectorX<double>& lower_bound =
          user_constraint.constraint()->lower_bound();
      const VectorX<double>& upper_bound =
          user_constraint.constraint()->upper_bound();
      VectorX<double> d{2 * constraint_value.size()};
      d << math::autoDiffToValueMatrix(constraint_value) - lower_bound,
          upper_bound - math::autoDiffToValueMatrix(constraint_value);
      MatrixX<double> dd_dq{d.size(), q.size()};
      dd_dq << math::autoDiffToGradientMatrix(constraint_value),
          -math::autoDiffToGradientMatrix(constraint_value);
      if ((d.array() < user_constraint.influence_threshold()).any()) {
        const auto d_s = VectorX<double>::Constant(
            d.size(), 1.01 * user_constraint.safety_threshold());
        VectorX<Expression> delta_q_symbolic =
            symbolic_curve.value(t) - curve.value(t);
        const auto f = (d + dd_dq * delta_q_symbolic >= d_s);
        if (!symbolic::is_true(f)) {
          program.AddLinearConstraint(f);
        }
      }
    }
  }

  log()->trace("Calling Solve(). START");
  solvers::MathematicalProgramResult result =
      solvers::Solve(program);
  log()->debug("Solver called: {}", result.get_solver_id().name());
  log()->debug("result: {}", result.get_solution_result());
  log()->trace("Calling Solve(). DONE");

  if (result.is_success()) {
    double new_duration =
        std::max<double>(result.GetSolution(duration_var),
                         std::sqrt(result.GetSolution(duration_squared_var)));
    log()->trace(
        "result.GetSolution(duration_var):                    {}",
        result.GetSolution(duration_var));
    log()->trace(
        "std::sqrt(result.GetSolution(duration_squared_var)): {}",
        std::sqrt(result.GetSolution(duration_squared_var)));
    log()->trace("new_duration: {}", new_duration);
    double duration_scaling =
        (new_duration - old_duration) / synthetic_timestep_;
    log()->trace("duration_scaling: {}", duration_scaling);
    double max_control_point_delta = 0;
    // Choose step size.
    double alpha = 0.5;
    double step_size = synthetic_timestep_ / alpha;
    const double minimum_step_size = 1e-3 * synthetic_timestep_;
    std::shared_ptr<const BsplineTrajectory<double>> candidate_curve{};
    do {
      max_control_point_delta = 0;
      step_size *= alpha;
      log()->trace("Updating curve. START");
      std::vector<MatrixX<double>> new_control_points{};
      for (int i = 0; i < num_control_points; ++i) {
        MatrixX<double> control_point_deltas =
            result.GetSolution(control_point_delta_vars[i]);
        MatrixX<double> control_point_velocity =
            control_point_deltas / synthetic_timestep_;
        new_control_points.push_back(curve.control_points()[i] +
                                     +step_size * control_point_velocity);
        double local_max =
            (step_size * control_point_velocity).cwiseAbs().maxCoeff();
        if (local_max > max_control_point_delta) {
          max_control_point_delta = local_max;
        }
      }

      new_duration = old_duration + step_size * duration_scaling;
      log()->trace("Updating knots ...");
      log()->trace("last_locked_knot_time: {}", last_locked_knot_time);
      std::vector<double> scaled_knots = curve.basis().knots();
      if (remaining_duration > 0) {
        for (int i = last_locked_knot_index; i < num_knots; ++i) {
          log()->trace("Old knot: {}", curve.basis().knots()[i]);
          scaled_knots[i] = last_locked_knot_time +
                            (curve.basis().knots()[i] - last_locked_knot_time) *
                                new_duration / old_duration;
          log()->trace("New knot: {}", scaled_knots[i]);
        }
      }
      log()->trace("Done updating knots ...");
      candidate_curve = std::make_shared<const BsplineTrajectory<double>>(
          BsplineBasis<double>(curve.basis().order(), scaled_knots),
          new_control_points);
    } while (!CheckConstraints(*candidate_curve, lock_time) &&
             step_size > minimum_step_size);
    if (step_size <= minimum_step_size) {
      log()->warn(
          "Step size ({}) is less than the minimum step size ({}).", step_size,
          minimum_step_size);
      log()->debug("DoSingleSmoothingStep(). DONE");
      return nullptr;
    }
    log()->debug("Step size: {}", step_size);
    log()->trace("Updating curve. DONE");
    if (max_control_point_delta <
            synthetic_timestep_ * convergence_threshold_ &&
        std::abs<double>(new_duration - old_duration) <
            synthetic_timestep_ * convergence_threshold_) {
      log()->debug("DoSingleSmoothingStep(). DONE (CONVERGED)");
      if (post_smoothing_step_callback_) {
        post_smoothing_step_callback_(*candidate_curve);
      }
      return nullptr;
    }
    log()->debug("DoSingleSmoothingStep(). DONE (IMPROVED)");
    if (post_smoothing_step_callback_) {
      post_smoothing_step_callback_(*candidate_curve);
    }
    return candidate_curve;
  }
  log()->debug("DoSingleSmoothingStep(). DONE");
  return nullptr;
}

bool AnytimeBsplineSmoother::CheckConstraints(
    const BsplineTrajectory<double>& curve, double lock_time) const {
  for (const auto& control_point : curve.control_points()) {
    if ((control_point.array() <
         position_lower_limit_.array() - position_limit_tolerance_)
            .any() ||
        (control_point.array() >
         position_upper_limit_.array() + position_limit_tolerance_)
            .any()) {
      log()->warn("Curve violates postition limits.");
      return false;
    }
  }
  const std::unique_ptr<BsplineTrajectory<double>> first_derivative_curve{
      dynamic_pointer_cast_or_throw<BsplineTrajectory<double>>(
          curve.MakeDerivative())};
  int last_affected_knot_index = first_derivative_curve->basis().order();
  for (const auto& control_point : first_derivative_curve->control_points()) {
    if (first_derivative_curve->basis().knots()[last_affected_knot_index] >
            lock_time &&
        ((control_point.array() < velocity_lower_limit_.array()).any() ||
         (control_point.array() > velocity_upper_limit_.array()).any())) {
      log()->warn("Curve violates velocity limits:");
      log()->warn("\tv_max: {}", velocity_upper_limit_.transpose());
      log()->warn("\tv    : {}", control_point.transpose());
      log()->warn("\tv_min: {}", velocity_lower_limit_.transpose());
      return false;
    }
    ++last_affected_knot_index;
  }
  if (curve.basis().order() > 2) {
    const std::unique_ptr<BsplineTrajectory<double>> second_derivative_curve{
        dynamic_pointer_cast_or_throw<BsplineTrajectory<double>>(
            first_derivative_curve->MakeDerivative())};
    last_affected_knot_index = second_derivative_curve->basis().order();
    for (const auto& control_point :
         second_derivative_curve->control_points()) {
      if (second_derivative_curve->basis().knots()[last_affected_knot_index] >
              lock_time &&
          ((control_point.array() < acceleration_lower_limit_.array()).any() ||
           (control_point.array() > acceleration_upper_limit_.array()).any())) {
        log()->warn("Curve violates acceleration limits:");
        log()->warn("\ta_max: {}",
                           acceleration_upper_limit_.transpose());
        log()->warn("\ta    : {}", control_point.transpose());
        log()->warn("\ta_min: {}",
                           acceleration_lower_limit_.transpose());
        return false;
      }
      ++last_affected_knot_index;
    }
  }
  for (const auto& position_constraint : position_constraints_) {
    const auto tt = VectorX<double>::LinSpaced(
        num_validation_points(), curve.start_time(), curve.end_time());
    for (int i = 0; i < num_validation_points(); ++i) {
      const double& t = tt(i);
      if (t < lock_time) continue;
      MatrixX<double> q = curve.value(t);
      if ((position_constraint.callback(q).first.array() <
           0.99 * position_constraint.safety_threshold)
              .any()) {
        log()->warn("Curve violates position constraints at t({}) = {}.",
                           i, t);
        log()->warn("q: {}", q.transpose());
        log()->warn("d: {}",
                           position_constraint.callback(q).first.transpose());
        log()->warn("d_s: {}", position_constraint.safety_threshold);
        return false;
      }
    }
  }

  for (const auto& user_constraint : user_constraints_) {
    const auto tt = VectorX<double>::LinSpaced(
        num_validation_points(), curve.start_time(), curve.end_time());
    for (int i = 0; i < num_validation_points(); ++i) {
      const double& t = tt(i);
      if (t < lock_time) continue;
      MatrixX<double> q = curve.value(t);
      VectorX<double> constraint_value{};
      user_constraint.constraint()->Eval(q, &constraint_value);
      const VectorX<double>& lower_bound =
          user_constraint.constraint()->lower_bound();
      const VectorX<double>& upper_bound =
          user_constraint.constraint()->upper_bound();
      VectorX<double> constraint_violation{2 * constraint_value.size()};
      constraint_violation << constraint_value - lower_bound,
          upper_bound - constraint_value;
      if ((constraint_violation.array() <
           0.99 * user_constraint.safety_threshold())
              .any()) {
        log()->warn("Curve violates user constraints at t({}) = {}.", i,
                           t);
        log()->warn("q: {}", q.transpose());
        log()->warn("d: {}", constraint_violation.transpose());
        log()->warn("d_s: {}", user_constraint.safety_threshold());
        return false;
      }
    }
  }
  return true;
}

std::shared_ptr<const BsplineTrajectory<double>>
AnytimeBsplineSmoother::SmoothCurveBlocking(
    const BsplineTrajectory<double>& curve) const {
  if (!CheckConstraints(curve)) {
    log()->warn("SmoothCurve(): Initial curve violates constraints!");
  }
  std::shared_ptr<const BsplineTrajectory<double>> curve_local =
      std::make_shared<const BsplineTrajectory<double>>(curve);
  for (int i = 0; i < max_smoothing_iterations_; ++i) {
    log()->info("Starting smoothing iteration {}", i);
    std::shared_ptr<const BsplineTrajectory<double>> new_curve{};
    new_curve = DoSingleSmoothingStep(*curve_local);
    if (new_curve) {
      curve_local.swap(new_curve);
    } else {
      break;
    }
  }
  return curve_local;
}

void AnytimeBsplineSmoother::SmoothCurve(
    const std::atomic<double>& current_time,
    std::shared_ptr<const BsplineTrajectory<double>>* curve,
    std::unique_ptr<const trajectories::Trajectory<double>>* derivative,
    std::mutex* curve_update_mutex,
    double estimated_smoothing_step_duration) const {
  DRAKE_THROW_UNLESS(curve != nullptr);
  const double start_time = current_time.load();
  if (!CheckConstraints(**curve)) {
    log()->warn("SmoothCurve(): Initial curve violates constraints!");
  }
  while (true) {
    const double iteration_start_time = current_time.load();
    // A current time prior to the start time indicates that the method should
    // return.
    if (iteration_start_time < start_time) {
      return;
    }
    double lock_time = iteration_start_time + estimated_smoothing_step_duration;
    // If the projected end time is after the end of the curve, return
    // immediately.
    if (lock_time > (*curve)->end_time()) {
      return;
    }
    log()->debug("lock_time: {}", lock_time);
    std::shared_ptr<const BsplineTrajectory<double>> new_curve{};
    new_curve = DoSingleSmoothingStep(**curve, lock_time);
    if (new_curve) {
      const double iteration_end_time = current_time.load();
      log()->debug("iteration_end_time: {}", iteration_end_time);
      if (iteration_end_time < start_time) {
        // A current time prior to the start time indicates that the method
        // should return.
        return;
      } else if (iteration_end_time >= lock_time) {
        estimated_smoothing_step_duration += iteration_end_time - lock_time;
        log()->warn("Smoothing step took too long. Discarding result.");
        log()->warn("Updated estimated_smoothing_step_duration to {} s.",
                           estimated_smoothing_step_duration);
      } else {
        std::lock_guard<std::mutex> lock(*curve_update_mutex);
        *derivative = (new_curve->MakeDerivative());
        curve->swap(new_curve);
      }
    }
  }
}

}  // namespace planning
}  // namespace drake
