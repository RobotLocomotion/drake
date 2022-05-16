#include "drake/multibody/optimization/toppra.h"

#include <algorithm>
#include <forward_list>
#include <limits>

#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace multibody {

using trajectories::PiecewisePolynomial;

Eigen::VectorXd Toppra::CalcGridPoints(const Trajectory<double>& path,
                                       const CalcGridPointsOptions& options) {
  std::forward_list<double> gridpts{path.start_time(), path.end_time()};
  int num_grid_points = 2;
  for (int iter = 0; iter < options.max_iter; iter++) {
    bool add_points{false};
    for (auto point = gridpts.begin(); std::next(point, 1) != gridpts.end();
         point++) {
      const double mid_pt = 0.5 * (*point + *(std::next(point, 1)));
      const double dist = *(std::next(point, 1)) - *point;
      if (dist > options.max_seg_length) {
        add_points = true;
        gridpts.emplace_after(point, mid_pt);
        num_grid_points++;
        point++;
        continue;
      }

      const double error = 0.5 * std::pow(dist, 2) *
                           path.EvalDerivative(mid_pt, 2).cwiseAbs().maxCoeff();
      if (error > options.max_err) {
        add_points = true;
        gridpts.emplace_after(point, mid_pt);
        num_grid_points++;
        point++;
        continue;
      }
    }
    if (!add_points) {
      break;
    }
  }
  while (num_grid_points < options.min_points) {
    for (auto point = gridpts.begin(); std::next(point, 1) != gridpts.end();
         std::advance(point, 2)) {
      const double mid_pt = 0.5 * (*point + *(std::next(point, 1)));
      gridpts.emplace_after(point, mid_pt);
      num_grid_points++;
    }
  }
  std::vector<double> result(gridpts.begin(), gridpts.end());
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result.data(),
                                                       result.size());
}

Toppra::Toppra(const Trajectory<double>& path,
               const MultibodyPlant<double>& plant,
               const Eigen::Ref<const Eigen::VectorXd>& gridpoints)
    : backward_prog_{new solvers::MathematicalProgram()},
      backward_x_{backward_prog_->NewContinuousVariables<1>("x")},
      backward_u_{backward_prog_->NewContinuousVariables<1>("u")},
      backward_cost_{backward_prog_->AddLinearCost(Eigen::Vector2d::Zero(), 0,
                                                   {backward_x_, backward_u_})},
      backward_continuity_con_{backward_prog_->AddLinearConstraint(
          Eigen::RowVector2d::Zero(), Vector1d(0), Vector1d(0),
          {backward_x_, backward_u_})},
      forward_prog_{new solvers::MathematicalProgram()},
      // Variables for the forward pass are u
      forward_u_{forward_prog_->NewContinuousVariables<1>("u")},
      forward_cost_{forward_prog_->AddLinearCost(-Vector1d(1), 0, forward_u_)},
      forward_continuity_con_{forward_prog_->AddLinearConstraint(
          Vector1d(0), Vector1d(0), Vector1d(0), forward_u_)},
      path_{path},
      plant_{plant},
      plant_context_{plant.CreateDefaultContext()},
      gridpoints_{gridpoints} {
  DRAKE_DEMAND(gridpoints(0) == path.start_time());
  DRAKE_DEMAND(gridpoints(gridpoints.size() - 1) == path.end_time());
  DRAKE_DEMAND(path.rows() == plant.num_positions());
  DRAKE_DEMAND(path.cols() == 1);
  DRAKE_DEMAND(path.has_derivative());
  for (int ii = 0; ii < gridpoints.size() - 1; ii++) {
    if (gridpoints(ii + 1) <= gridpoints(ii)) {
      throw std::runtime_error("Gridpoints must be monotonically increasing.");
    }
  }
  // Quaternions are not yet fully supported.
  for (BodyIndex body_idx{0}; body_idx < plant.num_bodies(); body_idx++) {
    if (plant.get_body(body_idx).has_quaternion_dofs()) {
      throw std::runtime_error(
          "Toppra does not support plants containing bodies with quaternion "
          "degrees of freedom.");
    }
  }
  // Add constraint on path velocity to ensure reachable set caluclated in
  // backward pass is always bounded. Maximum path velocity selected such that
  // trajectories with zero velocity segments do not cause the backward pass to
  // fail.
  backward_prog_->AddBoundingBoxConstraint(0, 1e16, backward_x_);
}

Binding<BoundingBoxConstraint> Toppra::AddJointVelocityLimit(
    const Eigen::Ref<const Eigen::VectorXd>& lower_limit,
    const Eigen::Ref<const Eigen::VectorXd>& upper_limit) {
  const int N = gridpoints_.size() - 1;
  const int n_dof = path_.rows();
  DRAKE_DEMAND(lower_limit.size() == n_dof);
  DRAKE_DEMAND(upper_limit.size() == n_dof);
  Eigen::VectorXd x_lower_bound(N);
  Eigen::VectorXd x_upper_bound(N);

  for (int knot = 0; knot < N; knot++) {
    const Eigen::VectorXd qs_dot = path_.EvalDerivative(gridpoints_(knot), 1);

    double sd_max = std::numeric_limits<double>::infinity();
    double sd_min = -std::numeric_limits<double>::infinity();
    for (int dof = 0; dof < n_dof; dof++) {
      if (qs_dot(dof) > 0) {
        sd_max = std::min(sd_max, upper_limit(dof) / qs_dot(dof));
        sd_min = std::max(sd_min, lower_limit(dof) / qs_dot(dof));
      } else if (qs_dot(dof) < 0) {
        sd_max = std::min(sd_max, lower_limit(dof) / qs_dot(dof));
        sd_min = std::max(sd_min, upper_limit(dof) / qs_dot(dof));
      }
    }
    x_lower_bound(knot) = std::pow(std::max(sd_min, 0.), 2);
    x_upper_bound(knot) = std::pow(sd_max, 2);
  }
  auto x_bbox = backward_prog_->AddBoundingBoxConstraint(0, 1, backward_x_);
  auto bounds = ToppraBoundingBoxConstraint(x_lower_bound, x_upper_bound);
  x_bounds_.emplace(x_bbox, bounds);
  return x_bbox;
}

std::pair<Binding<LinearConstraint>, Binding<LinearConstraint>>
Toppra::AddJointAccelerationLimit(
    const Eigen::Ref<const Eigen::VectorXd>& lower_limit,
    const Eigen::Ref<const Eigen::VectorXd>& upper_limit,
    ToppraDiscretization discretization) {
  const int N = gridpoints_.size() - 1;
  const int n_dof = path_.rows();
  DRAKE_DEMAND(lower_limit.size() == n_dof);
  DRAKE_DEMAND(upper_limit.size() == n_dof);
  const int n_con = discretization == ToppraDiscretization::kInterpolation
                        ? 2 * n_dof
                        : n_dof;
  Eigen::MatrixXd con_A(n_con, 2 * N);
  Eigen::MatrixXd con_lb(n_con, N);
  Eigen::MatrixXd con_ub(n_con, N);

  for (int knot = 0; knot < N; knot++) {
    const Eigen::VectorXd qs_dot = path_.EvalDerivative(gridpoints_(knot), 1);
    const Eigen::VectorXd qs_ddot = path_.EvalDerivative(gridpoints_(knot), 2);

    con_A.block(0, 2 * knot, n_dof, 1) << qs_ddot;
    con_A.block(0, 2 * knot + 1, n_dof, 1) << qs_dot;
    con_lb.block(0, knot, n_dof, 1) << lower_limit;
    con_ub.block(0, knot, n_dof, 1) << upper_limit;
  }
  if (discretization == ToppraDiscretization::kInterpolation) {
    CalcInterpolationConstraint(&con_A, &con_lb, &con_ub);
  }

  auto backward_con = backward_prog_->AddLinearConstraint(
      Eigen::MatrixXd::Zero(n_con, 2), Eigen::VectorXd::Zero(n_con),
      Eigen::VectorXd::Zero(n_con), {backward_x_, backward_u_});
  auto forward_con = forward_prog_->AddLinearConstraint(
      Eigen::VectorXd::Zero(n_con), Eigen::VectorXd::Zero(n_con),
      Eigen::VectorXd::Zero(n_con), forward_u_);
  auto coefficients = ToppraLinearConstraint(con_A, con_lb, con_ub);
  backward_lin_constraint_.emplace(backward_con, coefficients);
  forward_lin_constraint_.emplace(forward_con, coefficients);
  return std::make_pair(backward_con, forward_con);
}

std::pair<Binding<LinearConstraint>, Binding<LinearConstraint>>
Toppra::AddJointTorqueLimit(
    const Eigen::Ref<const Eigen::VectorXd>& lower_limit,
    const Eigen::Ref<const Eigen::VectorXd>& upper_limit,
    ToppraDiscretization discretization) {
  const int N = gridpoints_.size() - 1;
  const int n_dof = path_.rows();
  DRAKE_DEMAND(lower_limit.size() == n_dof);
  DRAKE_DEMAND(upper_limit.size() == n_dof);
  const int n_con = discretization == ToppraDiscretization::kInterpolation
                        ? 2 * n_dof
                        : n_dof;
  Eigen::MatrixXd con_A(n_con, 2 * N);
  Eigen::MatrixXd con_lb(n_con, N);
  Eigen::MatrixXd con_ub(n_con, N);
  Eigen::MatrixXd M(n_dof, n_dof);
  Eigen::VectorXd Cv(n_dof);
  Eigen::VectorXd G(n_dof);

  for (int knot = 0; knot < N; knot++) {
    // The equation of motion is M(q)v̇ + C(q, v) = g(q) + τ.
    // Since v = dq/ds * ṡ and v̇ = dq/ds * s̈ + d²q/ds² * ṡ²,
    // the constraint becomes
    // M * dq/ds * s̈ + ( M * d²q/ds² + C(q, dq/ds)) * ṡ² = g(q) + τ.
    // Toppra assumes q̇ = v and will have undefined behavior if it does not.
    const Eigen::VectorXd qs = path_.value(gridpoints_(knot));
    const Eigen::VectorXd qs_dot = path_.EvalDerivative(gridpoints_(knot), 1);
    const Eigen::VectorXd qs_ddot = path_.EvalDerivative(gridpoints_(knot), 2);

    plant_.SetPositions(plant_context_.get(), qs);
    plant_.SetVelocities(plant_context_.get(), qs_dot);
    plant_.CalcMassMatrix(*plant_context_, &M);
    plant_.CalcBiasTerm(*plant_context_, &Cv);
    G = plant_.CalcGravityGeneralizedForces(*plant_context_);

    con_A.block(0, 2 * knot, n_dof, 1) << M * qs_ddot + Cv;
    con_A.block(0, 2 * knot + 1, n_dof, 1) << M * qs_dot;
    con_lb.block(0, knot, n_dof, 1) << lower_limit + G;
    con_ub.block(0, knot, n_dof, 1) << upper_limit + G;
  }
  if (discretization == ToppraDiscretization::kInterpolation) {
    CalcInterpolationConstraint(&con_A, &con_lb, &con_ub);
  }

  auto backward_con = backward_prog_->AddLinearConstraint(
      Eigen::MatrixXd::Zero(n_con, 2), Eigen::VectorXd::Zero(n_con),
      Eigen::VectorXd::Zero(n_con), {backward_x_, backward_u_});
  auto forward_con = forward_prog_->AddLinearConstraint(
      Eigen::MatrixXd::Zero(n_con, 1), Eigen::VectorXd::Zero(n_con),
      Eigen::VectorXd::Zero(n_con), forward_u_);
  auto coefficients = ToppraLinearConstraint(con_A, con_lb, con_ub);
  backward_lin_constraint_.emplace(backward_con, coefficients);
  forward_lin_constraint_.emplace(forward_con, coefficients);
  return std::make_pair(backward_con, forward_con);
}

Binding<BoundingBoxConstraint> Toppra::AddFrameVelocityLimit(
    const Frame<double>& constraint_frame,
    const Eigen::Ref<const Vector6d>& lower_limit,
    const Eigen::Ref<const Vector6d>& upper_limit) {
  const int N = gridpoints_.size() - 1;
  Eigen::VectorXd x_lower_bound(N);
  Eigen::VectorXd x_upper_bound(N);
  Eigen::VectorXd velocity(6);

  for (int knot = 0; knot < N; knot++) {
    const Eigen::VectorXd qs = path_.value(gridpoints_(knot));
    const Eigen::VectorXd qs_dot = path_.EvalDerivative(gridpoints_(knot), 1);

    plant_.SetPositions(plant_context_.get(), qs);
    plant_.SetVelocities(plant_context_.get(), qs_dot);
    velocity = constraint_frame.CalcSpatialVelocityInWorld(*plant_context_)
                   .get_coeffs();

    double sd_max = std::numeric_limits<double>::infinity();
    double sd_min = -std::numeric_limits<double>::infinity();
    for (int dof = 0; dof < 6; dof++) {
      if (velocity(dof) > 0) {
        sd_max = std::min(sd_max, upper_limit(dof) / velocity(dof));
        sd_min = std::max(sd_min, lower_limit(dof) / velocity(dof));
      } else if (velocity(dof) < 0) {
        sd_max = std::min(sd_max, lower_limit(dof) / velocity(dof));
        sd_min = std::max(sd_min, upper_limit(dof) / velocity(dof));
      }
    }
    x_lower_bound(knot) = std::pow(std::max(sd_min, 0.), 2);
    x_upper_bound(knot) = std::pow(sd_max, 2);
  }
  auto x_bbox = backward_prog_->AddBoundingBoxConstraint(0, 1, backward_x_);
  auto bounds = ToppraBoundingBoxConstraint(x_lower_bound, x_upper_bound);
  x_bounds_.emplace(x_bbox, bounds);
  return x_bbox;
}

Binding<BoundingBoxConstraint> Toppra::AddFrameTranslationalSpeedLimit(
    const Frame<double>& constraint_frame, const double& upper_limit) {
  const int N = gridpoints_.size() - 1;
  Eigen::VectorXd x_lower_bound = Eigen::VectorXd::Zero(N);
  Eigen::VectorXd x_upper_bound(N);

  for (int knot = 0; knot < N; knot++) {
    const Eigen::VectorXd qs = path_.value(gridpoints_(knot));
    const Eigen::VectorXd qs_dot = path_.EvalDerivative(gridpoints_(knot), 1);

    plant_.SetPositions(plant_context_.get(), qs);
    plant_.SetVelocities(plant_context_.get(), qs_dot);
    const SpatialVelocity<double> velocity =
        constraint_frame.CalcSpatialVelocityInWorld(*plant_context_);
    const double speed_squared = velocity.translational().squaredNorm();

    // Check to avoid performing division by zero.
    if (speed_squared > 0) {
      x_upper_bound(knot) = std::pow(upper_limit, 2) / speed_squared;
    } else {
      x_upper_bound(knot) = std::numeric_limits<double>::infinity();
    }
  }
  auto x_bbox = backward_prog_->AddBoundingBoxConstraint(0, 1, backward_x_);
  auto bounds = ToppraBoundingBoxConstraint(x_lower_bound, x_upper_bound);
  x_bounds_.emplace(x_bbox, bounds);
  return x_bbox;
}

std::pair<Binding<LinearConstraint>, Binding<LinearConstraint>>
Toppra::AddFrameAccelerationLimit(
    const Frame<double>& constraint_frame,
    const Eigen::Ref<const Vector6d>& lower_limit,
    const Eigen::Ref<const Vector6d>& upper_limit,
    ToppraDiscretization discretization) {
  const int N = gridpoints_.size() - 1;
  const int n_dof = path_.rows();
  const int n_con =
      discretization == ToppraDiscretization::kInterpolation ? 12 : 6;
  Eigen::MatrixXd con_A(n_con, 2 * N);
  Eigen::MatrixXd con_lb(n_con, N);
  Eigen::MatrixXd con_ub(n_con, N);
  Eigen::MatrixXd J(6, n_dof);
  Vector6d dJds_times_v;

  for (int knot = 0; knot < N; knot++) {
    // The constraint equation is Jv_WF * v̇ + J̇v_WF * v = a_WF.
    // Since v = dq/ds * ṡ and v̇ = dq/ds * s̈ + d²q/ds² * ṡ²,
    // the constraint becomes
    // Jv_WF * dq/ds * s̈ + (Jv_WF * d²q/ds² + J̇v_WF) * ṡ² = a_WF
    // Toppra assumes q̇ = v and will have undefined behavior if it does not.
    const Eigen::VectorXd qs = path_.value(gridpoints_(knot));
    const Eigen::VectorXd qs_dot = path_.EvalDerivative(gridpoints_(knot), 1);
    const Eigen::VectorXd qs_ddot = path_.EvalDerivative(gridpoints_(knot), 2);

    plant_.SetPositions(plant_context_.get(), qs);
    plant_.SetVelocities(plant_context_.get(), qs_dot);
    plant_.CalcJacobianSpatialVelocity(
        *plant_context_, JacobianWrtVariable::kQDot, constraint_frame,
        Eigen::Vector3d::Zero(), plant_.world_frame(), plant_.world_frame(),
        &J);
    dJds_times_v = plant_
                       .CalcBiasSpatialAcceleration(
                           *plant_context_, JacobianWrtVariable::kV,
                           constraint_frame, Eigen::Vector3d::Zero(),
                           plant_.world_frame(), plant_.world_frame())
                       .get_coeffs();

    con_A.block(0, 2 * knot, 6, 1) << J * qs_ddot + dJds_times_v;
    con_A.block(0, 2 * knot + 1, 6, 1) << J * qs_dot;
    con_lb.block(0, knot, 6, 1) << lower_limit;
    con_ub.block(0, knot, 6, 1) << upper_limit;
  }
  if (discretization == ToppraDiscretization::kInterpolation) {
    CalcInterpolationConstraint(&con_A, &con_lb, &con_ub);
  }

  auto backward_con = backward_prog_->AddLinearConstraint(
      Eigen::MatrixXd::Zero(n_con, 2), Eigen::VectorXd::Zero(n_con),
      Eigen::VectorXd::Zero(n_con), {backward_x_, backward_u_});
  auto forward_con = forward_prog_->AddLinearConstraint(
      Eigen::MatrixXd::Zero(n_con, 1), Eigen::VectorXd::Zero(n_con),
      Eigen::VectorXd::Zero(n_con), forward_u_);
  auto coefficients = ToppraLinearConstraint(con_A, con_lb, con_ub);
  backward_lin_constraint_.emplace(backward_con, coefficients);
  forward_lin_constraint_.emplace(forward_con, coefficients);
  return std::make_pair(backward_con, forward_con);
}

void Toppra::CalcInterpolationConstraint(Eigen::MatrixXd* A,
                                         Eigen::MatrixXd* lower_bound,
                                         Eigen::MatrixXd* upper_bound) {
  const int N = gridpoints_.size() - 1;
  const int n_con = A->rows() / 2;

  // For each gridpoint with constraint lbᵢ ≤ aᵢuᵢ + bᵢxᵢ ≤ ubᵢ, adds the
  // interpolated constraint lbᵢ₊₁ ≤ (aᵢ₊₁ + 2Δᵢbᵢ₊₁)uᵢ + bᵢ₊₁xᵢ ≤ ubᵢ₊₁ to the
  // gridpoint. For the final gridpoint the original constraint is repeated to
  // maintain constraint size consistency without running off the end of the
  // path.
  for (int knot = 1; knot < N; knot++) {
    const double delta = gridpoints_(knot) - gridpoints_(knot - 1);
    A->block(n_con, 2 * (knot - 1), n_con, 1)
        << A->block(0, 2 * knot, n_con, 1);
    A->block(n_con, 2 * (knot - 1) + 1, n_con, 1)
        << A->block(0, 2 * knot + 1, n_con, 1) +
               2 * delta * A->block(0, 2 * knot, n_con, 1);
    lower_bound->block(n_con, knot - 1, n_con, 1)
        << lower_bound->block(0, knot, n_con, 1);
    upper_bound->block(n_con, knot - 1, n_con, 1)
        << upper_bound->block(0, knot, n_con, 1);
  }
  A->block(n_con, 2 * (N - 1), n_con, 1) << A->block(0, 2 * (N - 1), n_con, 1);
  A->block(n_con, 2 * (N - 1) + 1, n_con, 1)
      << A->block(0, 2 * (N - 1) + 1, n_con, 1);
  lower_bound->block(n_con, N - 1, n_con, 1)
      << lower_bound->block(0, N - 1, n_con, 1);
  upper_bound->block(n_con, N - 1, n_con, 1)
      << upper_bound->block(0, N - 1, n_con, 1);
}

std::optional<Eigen::Matrix2Xd> Toppra::ComputeBackwardPass(
    double s_dot_0, double s_dot_N, const SolverInterface& solver) {
  DRAKE_DEMAND(s_dot_0 >= 0);
  DRAKE_DEMAND(s_dot_N >= 0);
  const Eigen::Vector2d min_cost_A(1, 0);
  const Eigen::Vector2d max_cost_A(-1, 0);
  const int N = gridpoints_.size() - 1;

  // Compute controllable set.
  Eigen::Matrix2Xd K(2, N + 1);
  K.col(N) << std::pow(s_dot_N, 2), std::pow(s_dot_N, 2);
  // Setup and solve sequence of one-step problems
  for (int knot = N - 1; knot > -1; knot--) {
    // Setup constraints for both max & min
    const double delta = gridpoints_(knot + 1) - gridpoints_(knot);
    backward_continuity_con_.evaluator()->UpdateCoefficients(
        Eigen::Vector2d(1, 2 * delta).transpose(), Vector1d(K(0, knot + 1)),
        Vector1d(K(1, knot + 1)));
    for (auto& [constraint, bounds] : x_bounds_) {
      constraint.evaluator()->set_bounds(Vector1d(bounds.lb(knot)),
                                         Vector1d(bounds.ub(knot)));
    }
    for (auto& [constraint, coefficients] : backward_lin_constraint_) {
      constraint.evaluator()->UpdateCoefficients(
          coefficients.coeffs.middleCols<2>(2 * knot),
          coefficients.lb.col(knot), coefficients.ub.col(knot));
    }
    // Solve minimum
    {
      backward_cost_.evaluator()->UpdateCoefficients(min_cost_A);
      solvers::MathematicalProgramResult result;
      solver.Solve(*backward_prog_, {}, {}, &result);
      if (!result.is_success()) {
        drake::log()->error(
            fmt::format("Toppra failed to find lower bound of controllable set "
                        "at knot {}/{}.",
                        knot, N));
        return std::nullopt;
      } else {
        K(0, knot) = result.GetSolution(backward_x_)(0);
      }
    }
    // Solve maximum
    {
      backward_cost_.evaluator()->UpdateCoefficients(max_cost_A);
      solvers::MathematicalProgramResult result;
      solver.Solve(*backward_prog_, {}, {}, &result);
      if (!result.is_success()) {
        drake::log()->error(
            fmt::format("Toppra failed to find upper bound of controllable set "
                        "at knot {}/{}.",
                        knot, N));
        return std::nullopt;
      } else {
        K(1, knot) = result.GetSolution(backward_x_)(0);
      }
    }

    if (K.col(knot).hasNaN()) {
      drake::log()->error(
          fmt::format("Toppra hit numerical issues. Controllable set "
                      "at knot {}/{} "
                      "can't be computed.",
                      knot, N));
      return std::nullopt;
    }
    if (K(0, knot) < 0) {
      K(0, knot) = 0;
    }
  }

  // Check problem is controllable
  const double x_start = std::pow(s_dot_0, 2);
  const double eps = 1e-8;
  if (x_start < K(0, 0) - eps || x_start > K(1, 0) + eps) {
    drake::log()->error(
        fmt::format("Toppra: Initial velocity not controllable. {} not in "
                    "({}, {}).",
                    x_start, K(0, 0), K(1, 0)));
    return std::nullopt;
  }

  return K;
}

std::optional<std::pair<Eigen::VectorXd, Eigen::VectorXd>>
Toppra::ComputeForwardPass(double s_dot_0,
                           const Eigen::Ref<const Eigen::Matrix2Xd>& K,
                           const SolverInterface& solver) {
  const int N = gridpoints_.size() - 1;
  DRAKE_DEMAND(s_dot_0 >= 0);
  DRAKE_DEMAND(K.cols() == N + 1);

  // Compute controls greedily.
  Eigen::VectorXd xstar(N + 1);
  Eigen::VectorXd ustar(N);
  xstar(0) = std::pow(s_dot_0, 2);
  for (int knot = 0; knot < N; knot++) {
    const double delta = gridpoints_(knot + 1) - gridpoints_(knot);
    forward_continuity_con_.evaluator()->UpdateCoefficients(
        Vector1d(2 * delta), Vector1d(K(0, knot + 1) - xstar(knot)),
        Vector1d(K(1, knot + 1) - xstar(knot)));
    for (auto& [constraint, coefficients] : forward_lin_constraint_) {
      constraint.evaluator()->UpdateCoefficients(
          coefficients.coeffs.col(2 * knot + 1),
          coefficients.lb.col(knot) -
              xstar(knot) * coefficients.coeffs.col(2 * knot),
          coefficients.ub.col(knot) -
              xstar(knot) * coefficients.coeffs.col(2 * knot));
    }
    solvers::MathematicalProgramResult result;
    solver.Solve(*forward_prog_, {}, {}, &result);
    if (!result.is_success()) {
      drake::log()->error(fmt::format(
          "Toppra failed to find the maximum path acceleration at knot {}/{}.",
          knot, N));
      return std::nullopt;
    } else {
      ustar(knot) = result.GetSolution()(0);
    }
    double xnext = xstar(knot) + 2 * delta * ustar(knot);
    xstar(knot + 1) = std::max(K(0, knot + 1), std::min(K(1, knot + 1), xnext));
  }

  return std::make_pair(xstar, ustar);
}

std::optional<PiecewisePolynomial<double>> Toppra::SolvePathParameterization() {
  const double s_dot_0 = 0;
  const double s_dot_N = 0;

  // Clp appears to outperform Mosek for these optimizations.
  // TODO(mpetersen94) Add Gurobi in the correct ordering
  auto solver = solvers::MakeFirstAvailableSolver(
      {solvers::ClpSolver::id(), solvers::MosekSolver::id()});

  const std::optional<Eigen::Matrix2Xd> K =
      ComputeBackwardPass(s_dot_0, s_dot_N, *solver);
  if (!K) {  // Error ocurred in backpass, return nothing
    return std::nullopt;
  }
  const auto forward_results = ComputeForwardPass(s_dot_0, K.value(), *solver);
  if (!forward_results) {  // Error ocurred in forward pass, return nothing
    return std::nullopt;
  }
  const Eigen::VectorXd x_star = forward_results.value().first;
  const Eigen::VectorXd u_star = forward_results.value().second;
  const Eigen::VectorXd sd_knots = x_star.cwiseSqrt();
  if (sd_knots.hasNaN()) {
    drake::log()->error("Toppra hit numerical issues. Found NaN sdot.");
    return std::nullopt;
  }

  // The path parameterization returned is piecewise quadratic.
  const int N = gridpoints_.size() - 1;
  std::vector<double> t_knots(N + 1);
  std::vector<Polynomial<double>> polynomials(N);
  t_knots[0] = path_.start_time();
  for (size_t knot = 0; knot < t_knots.size() - 1; knot++) {
    const double delta = gridpoints_(knot + 1) - gridpoints_(knot);
    const double sd_avg = (sd_knots(knot + 1) + sd_knots(knot)) / 2.;
    const double delta_t = delta / sd_avg;
    t_knots[knot + 1] = t_knots[knot] + delta_t;

    const Eigen::Vector3d coeffs(gridpoints_(knot), sd_knots(knot),
                                 0.5 * u_star(knot));
    polynomials[knot] = Polynomial<double>(coeffs);
  }

  return PiecewisePolynomial<double>(polynomials, t_knots);
}

}  // namespace multibody
}  // namespace drake
