#include "drake/multibody/optimization/toppra.h"

#include <algorithm>
#include <forward_list>
#include <limits>

#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {

using trajectories::PiecewisePolynomial;

Eigen::VectorXd Toppra::CalcGridpts(const PiecewisePolynomial<double>& path,
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
      gridpoints_{gridpoints} {
  DRAKE_DEMAND(gridpoints(0) == path.start_time());
  DRAKE_DEMAND(gridpoints(gridpoints.size() - 1) == path.end_time());
  DRAKE_DEMAND(path.rows() == plant.num_positions());
  DRAKE_DEMAND(path.cols() == 1);
  DRAKE_DEMAND(path.has_derivative());
  unused(plant_);
  for (int ii = 0; ii < gridpoints.size() - 1; ii++) {
    if (gridpoints(ii + 1) <= gridpoints(ii)) {
      throw std::runtime_error("Gridpoints must be monotonically increasing.");
    }
  }
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

    if (discretization == ToppraDiscretization::kInterpolation) {
      const double delta = gridpoints_(knot + 1) - gridpoints_(knot);
      const Eigen::VectorXd qs_dot_next =
          path_.EvalDerivative(gridpoints_(knot + 1), 1);
      const Eigen::VectorXd qs_ddot_next =
          path_.EvalDerivative(gridpoints_(knot + 1), 2);

      con_A.block(n_dof, 2 * knot, n_dof, 1) << qs_ddot_next;
      con_A.block(n_dof, 2 * knot + 1, n_dof, 1)
          << qs_dot_next + 2 * delta * qs_ddot_next;
      con_lb.block(n_dof, knot, n_dof, 1) << lower_limit;
      con_ub.block(n_dof, knot, n_dof, 1) << upper_limit;
    }
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

std::optional<Eigen::Matrix2Xd> Toppra::ComputeBackwardPass(double s_dot_0,
                                                            double s_dot_N) {
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
      const auto result = solvers::Solve(*backward_prog_);
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
      const auto result = solvers::Solve(*backward_prog_);
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
    drake::log()->error(fmt::format(
        "ToppraSolver: Initial velocity not controllable. {} not in "
        "({}, {}).",
        x_start, K(0, 0), K(1, 0)));
    return std::nullopt;
  }

  return K;
}

std::optional<std::pair<Eigen::VectorXd, Eigen::VectorXd>>
Toppra::ComputeForwardPass(double s_dot_0,
                           const Eigen::Ref<const Eigen::Matrix2Xd>& K) {
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
    const auto result = solvers::Solve(*forward_prog_);
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

  const std::optional<Eigen::Matrix2Xd> K =
      ComputeBackwardPass(s_dot_0, s_dot_N);
  if (!K) {  // Error ocurred in backpass, return nothing
    return std::nullopt;
  }
  const auto forward_results = ComputeForwardPass(s_dot_0, K.value());
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
