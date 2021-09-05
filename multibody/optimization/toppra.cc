#include "drake/multibody/optimization/toppra.h"

#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {

using trajectories::PiecewisePolynomial;

Eigen::VectorXd CalcGridpts(const PiecewisePolynomial<double>& path,
                            double max_err, int max_iter, double max_seg_length,
                            int min_points) {
  std::vector<double> gridpts{path.start_time(), path.end_time()};
  for (int iter = 0; iter < max_iter; iter++) {
    bool add_points{false};
    for (size_t ii = 0; ii < gridpts.size() - 1; ii++) {
      double mid_pt{0.5 * (gridpts[ii] + gridpts[ii + 1])};
      double dist{gridpts[ii + 1] - gridpts[ii]};
      if (dist > max_seg_length) {
        add_points = true;
        gridpts.emplace(gridpts.begin() + ii + 1, mid_pt);
        ii++;
        continue;
      }

      double error{0.5 * std::pow(dist, 2) *
                   path.EvalDerivative(mid_pt, 2).cwiseAbs().maxCoeff()};
      if (error > max_err) {
        add_points = true;
        gridpts.emplace(gridpts.begin() + ii + 1, mid_pt);
        ii++;
        continue;
      }
    }
    if (!add_points) {
      break;
    }
  }
  while (gridpts.size() < static_cast<size_t>(min_points)) {
    for (size_t ii = 0; ii < gridpts.size() - 1; ii += 2) {
      double mid_pt{0.5 * (gridpts[ii] + gridpts[ii + 1])};
      gridpts.emplace(gridpts.begin() + ii + 1, mid_pt);
    }
  }
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(gridpts.data(),
                                                       gridpts.size());
}

Toppra::Toppra(const PiecewisePolynomial<double>& path,
               const MultibodyPlant<double>& plant,
               const Eigen::VectorXd& gridpoints)
    : backward_prog_{new solvers::MathematicalProgram()},
      // Variables for the backward pass are [x, u]
      backward_vars_{backward_prog_->NewContinuousVariables(2)},
      backward_cost_{backward_prog_->AddLinearCost(Eigen::VectorXd::Zero(2), 0,
                                                   backward_vars_)},
      backward_continuity_con_{backward_prog_->AddLinearConstraint(
          Eigen::MatrixXd::Zero(1, 2), Eigen::VectorXd::Zero(1),
          Eigen::VectorXd::Zero(1), backward_vars_)},
      forward_prog_{new solvers::MathematicalProgram()},
      // Variables for the forward pass are u
      forward_vars_{forward_prog_->NewContinuousVariables(1, "u")},
      forward_cost_{forward_prog_->AddLinearCost(-Eigen::VectorXd::Ones(1), 0,
                                                 forward_vars_)},
      forward_continuity_con_{forward_prog_->AddLinearConstraint(
          Eigen::MatrixXd::Zero(1, 1), Eigen::VectorXd::Zero(1),
          Eigen::VectorXd::Zero(1), forward_vars_)},
      path_{path},
      plant_{plant},
      gridpoints_{gridpoints} {
  DRAKE_DEMAND(gridpoints(0) == path.start_time());
  DRAKE_DEMAND(gridpoints(gridpoints.size() - 1) == path.end_time());
}

Toppra::ToppraBoundingBoxConstraint& Toppra::AddJointVelocityLimit(
    const Eigen::VectorXd& lower_limit, const Eigen::VectorXd upper_limit) {
  const int N = gridpoints_.size() - 1;
  const int n_dof = path_.rows();
  DRAKE_DEMAND(lower_limit.size() == n_dof);
  DRAKE_DEMAND(upper_limit.size() == n_dof);
  Eigen::VectorXd x_lower_bound(N);
  Eigen::VectorXd x_upper_bound(N);

  for (int knot = 0; knot < N; knot++) {
    Eigen::VectorXd qs_dot = path_.EvalDerivative(gridpoints_(knot), 1);

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
  bb_constraint_.push_back(
      backward_prog_->AddBoundingBoxConstraint(0, 1, backward_vars_(0)));
  bb_constraint_coeff_.emplace_back(x_lower_bound, x_upper_bound);
  return bb_constraint_coeff_.back();
}

Toppra::ToppraLinearConstraint& Toppra::AddJointAccelerationLimit(
    const Eigen::VectorXd& lower_limit, const Eigen::VectorXd upper_limit,
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
    Eigen::VectorXd qs_dot = path_.EvalDerivative(gridpoints_(knot), 1);
    Eigen::VectorXd qs_ddot = path_.EvalDerivative(gridpoints_(knot), 2);

    con_A.block(0, 2 * knot, n_dof, 1) << qs_ddot;
    con_A.block(0, 2 * knot + 1, n_dof, 1) << qs_dot;
    con_lb.block(0, knot, n_dof, 1) << lower_limit;
    con_ub.block(0, knot, n_dof, 1) << upper_limit;

    if (discretization == ToppraDiscretization::kInterpolation) {
      double delta = gridpoints_(knot + 1) - gridpoints_(knot);
      Eigen::VectorXd qs_dot_next =
          path_.EvalDerivative(gridpoints_(knot + 1), 1);
      Eigen::VectorXd qs_ddot_next =
          path_.EvalDerivative(gridpoints_(knot + 1), 2);

      con_A.block(n_dof, 2 * knot, n_dof, 1) << qs_ddot_next;
      con_A.block(n_dof, 2 * knot + 1, n_dof, 1)
          << qs_dot_next + 2 * delta * qs_ddot_next;
      con_lb.block(n_dof, knot, n_dof, 1) << lower_limit;
      con_ub.block(n_dof, knot, n_dof, 1) << upper_limit;
    }
  }
  backward_lin_constraint_.push_back(backward_prog_->AddLinearConstraint(
      Eigen::MatrixXd::Zero(n_con, 2), Eigen::VectorXd::Zero(n_con),
      Eigen::VectorXd::Zero(n_con), backward_vars_));
  forward_lin_constraint_.push_back(forward_prog_->AddLinearConstraint(
      Eigen::MatrixXd::Zero(n_con, 1), Eigen::VectorXd::Zero(n_con),
      Eigen::VectorXd::Zero(n_con), forward_vars_));
  lin_constraint_coeff_.emplace_back(con_A, con_lb, con_ub);
  return lin_constraint_coeff_.back();
}

// ToppraLinearConstraint Toppra::AddJointTorqueLimit(
//     const Eigen::VectorXd& lower_limit, const Eigen::VectorXd upper_limit);

// ToppraBoundingBoxConstraint Toppra::AddFrameVelocityLimit(
//     const Frame<double>& constraint_frame, const Eigen::VectorXd&
//     lower_limit, const Eigen::VectorXd upper_limit);

// ToppraLinearConstraint Toppra::AddFrameAccelerationLimit(
//     const Frame<double>& constraint_frame, const Eigen::VectorXd&
//     lower_limit, const Eigen::VectorXd upper_limit);

Eigen::MatrixXd Toppra::ComputeBackwardPass(double s_dot_0, double s_dot_N) {
  DRAKE_DEMAND(s_dot_0 >= 0);
  DRAKE_DEMAND(s_dot_N >= 0);
  Eigen::Vector2d min_cost_A(1, 1e-9);
  Eigen::Vector2d max_cost_A(-1, -1e-9);
  const int N = gridpoints_.size() - 1;

  // Compute controllable set.
  Eigen::MatrixXd K(N + 1, 2);
  K.row(N) << std::pow(s_dot_N, 2), std::pow(s_dot_N, 2);
  // Setup and solve sequence of one-step problems
  for (int knot = N - 1; knot > -1; knot--) {
    // Setup constraints for both max & min
    double delta = gridpoints_(knot + 1) - gridpoints_(knot);
    backward_continuity_con_.evaluator()->UpdateCoefficients(
        Eigen::Vector2d(1, 2 * delta).transpose(), K.block(knot + 1, 0, 1, 1),
        K.block(knot + 1, 1, 1, 1));
    for (size_t bb_con = 0; bb_con < bb_constraint_coeff_.size(); bb_con++) {
      Eigen::VectorXd lb, ub;
      std::tie(lb, ub) = bb_constraint_coeff_[bb_con];
      bb_constraint_[bb_con].evaluator()->set_bounds(lb.segment(knot, 1),
                                                     ub.segment(knot, 1));
    }
    for (size_t lin_con = 0; lin_con < lin_constraint_coeff_.size();
         lin_con++) {
      Eigen::MatrixXd A, lb, ub;
      std::tie(A, lb, ub) = lin_constraint_coeff_[lin_con];
      backward_lin_constraint_[lin_con].evaluator()->UpdateCoefficients(
          A.block(0, 2 * knot, A.rows(), 2), lb.col(knot), ub.col(knot));
    }
    // Solve minimum
    {
      backward_cost_.evaluator()->UpdateCoefficients(min_cost_A);
      auto result = solvers::Solve(*backward_prog_);
      K(knot, 0) = result.GetSolution()(0);
    }
    // Solve maximum
    {
      backward_cost_.evaluator()->UpdateCoefficients(max_cost_A);
      auto result = solvers::Solve(*backward_prog_);
      K(knot, 1) = result.GetSolution()(0);
    }

    if (K.row(knot).hasNaN()) {
      throw std::runtime_error(
          fmt::format("Toppra hit numerical issues. Controllable set "
                      "at knot {}/{} "
                      "can't be computed.",
                      knot, N));
    }
    if (K(knot, 0) < 0) {
      K(knot, 0) = 0;
    }
  }

  // Check problem is controllable
  double x_start = std::pow(s_dot_0, 2);
  double eps = 1e-8;
  if (x_start < K(0, 0) - eps || x_start > K(0, 1) + eps) {
    throw std::runtime_error(fmt::format(
        "ToppraSolver: Initial velocity not controllable. {} not in "
        "({}, {}).",
        x_start, K(0, 0), K(0, 1)));
  }

  return K;
}

Eigen::VectorXd Toppra::ComputeForwardPass(double s_dot_0, Eigen::MatrixXd& K) {
  const int N = gridpoints_.size() - 1;
  DRAKE_DEMAND(s_dot_0 >= 0);
  DRAKE_DEMAND(K.rows() == N + 1);
  DRAKE_DEMAND(K.cols() == 2);

  // Compute controls greedily.
  Eigen::VectorXd xstar(N + 1);
  Eigen::VectorXd ustar(N);
  xstar(0) = std::pow(s_dot_0, 2);
  for (int knot = 0; knot < N; knot++) {
    double delta = gridpoints_(knot + 1) - gridpoints_(knot);
    backward_continuity_con_.evaluator()->UpdateCoefficients(
        Vector1d(2 * delta), Vector1d(K(knot + 1, 0) - xstar(knot)),
        Vector1d(K(knot + 1, 1) - xstar(knot)));
    for (size_t lin_con = 0; lin_con < lin_constraint_coeff_.size();
         lin_con++) {
      Eigen::MatrixXd A, lb, ub;
      std::tie(A, lb, ub) = lin_constraint_coeff_[lin_con];
      forward_lin_constraint_[lin_con].evaluator()->UpdateCoefficients(
          A.col(2 * knot + 1), lb.col(knot) - xstar(knot) * A.col(2 * knot),
          ub.col(knot) - xstar(knot) * A.col(2 * knot));
    }
    auto result = solvers::Solve(*forward_prog_);
    ustar(knot) = result.GetSolution()(0);
    double xnext = xstar(knot) + 2 * delta * ustar(knot);
    xnext *= 0.99;
    xstar(knot + 1) = std::max(K(knot + 1, 0), std::min(K(knot + 1, 1), xnext));
  }

  return xstar;
}

PiecewisePolynomial<double> Toppra::Solve() {
  double s_dot_0 = 0;
  double s_dot_N = 0;

  Eigen::MatrixXd K = ComputeBackwardPass(s_dot_0, s_dot_N);
  Eigen::VectorXd x_star = ComputeForwardPass(s_dot_0, K);
  Eigen::VectorXd sd_knots = x_star.cwiseSqrt();
  if (sd_knots.hasNaN()) {
    throw std::runtime_error("Toppra hit numerical issues. Found NaN sdot.");
  }

  Eigen::VectorXd t_knots(gridpoints_.size());
  Eigen::MatrixXd q_knots(path_.rows(), gridpoints_.size());
  t_knots(0) = 0;
  q_knots.col(0) = path_.value(gridpoints_(0));
  for (int knot = 1; knot < t_knots.size(); knot++) {
    double delta = gridpoints_(knot) - gridpoints_(knot - 1);
    double sd_avg = (sd_knots(knot) + sd_knots(knot - 1)) / 2.;
    double delta_t = delta / sd_avg;
    t_knots(knot) = t_knots(knot - 1) + delta_t;
    q_knots.col(knot) = path_.value(gridpoints_(knot));
  }

  const Eigen::VectorXd v_start = Eigen::VectorXd::Zero(path_.rows());
  const Eigen::VectorXd v_end = Eigen::VectorXd::Zero(path_.rows());
  return PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
      t_knots, q_knots, v_start, v_end);
}

}  // namespace multibody
}  // namespace drake
