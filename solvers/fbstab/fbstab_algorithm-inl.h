#pragma once

namespace drake {
namespace solvers {
namespace fbstab {

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                Feasibility>::FBstabAlgorithm(Variable* x1, Variable* x2,
                                              Variable* x3, Variable* x4,
                                              Residual* r1, Residual* r2,
                                              LinearSolver* lin_sol,
                                              Feasibility* fcheck) {
  if (x1 == nullptr || x2 == nullptr || x3 == nullptr || x4 == nullptr) {
    throw std::runtime_error("A Variable supplied to FBstabAlgorithm is null.");
  }
  if (r1 == nullptr || r2 == nullptr) {
    throw std::runtime_error("A Residual supplied to FBstabAlgorithm is null");
  }
  if (lin_sol == nullptr) {
    throw std::runtime_error(
        "The LinearSolver supplied to FBstabAlgorithm is null.");
  }
  if (fcheck == nullptr) {
    throw std::runtime_error(
        "The Feasibility object supplied to FBstabAlgorithm is null.");
  }

  xk_ = x1;
  xi_ = x2;
  xp_ = x3;
  dx_ = x4;

  rk_ = r1;
  ri_ = r2;

  linear_solver_ = lin_sol;
  feasibility_ = fcheck;
}

// TODO(dliaomcp@umich.edu): Enable printing to a log file rather than just
// stdout
template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
SolverOut FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                          Feasibility>::Solve(const Data* qp_data,
                                              Variable* x0) {
  // Make sure the linear solver and residuals objects are using the same value
  // for the alpha parameter.
  rk_->SetAlpha(alpha_);
  ri_->SetAlpha(alpha_);
  linear_solver_->SetAlpha(alpha_);

  struct SolverOut output = {
      MAXITERATIONS,  // exit flag
      0.0,            // residual
      0,              // prox iters
      0               // newton iters
  };

  // Supply a pointer to the data object.
  xk_->LinkData(qp_data);
  xi_->LinkData(qp_data);
  dx_->LinkData(qp_data);
  xp_->LinkData(qp_data);
  x0->LinkData(qp_data);

  // Initialization phase.
  double sigma_ = sigma0_;

  x0->InitializeConstraintMargin();
  xk_->Copy(*x0);
  xi_->Copy(*x0);
  dx_->Fill(1.0);

  rk_->NaturalResidual(*xk_);
  ri_->Fill(0.0);
  double E0 = rk_->Norm();
  double Ek = E0;
  double inner_tol = saturate(E0, inner_tol_min_, inner_tol_max_);

  newton_iters_ = 0;
  prox_iters_ = 0;
  PrintIterHeader();
  // Main proximal loop.
  for (int k = 0; k < max_prox_iters_; k++) {
    // The solver stops if:
    // a) the desired precision is obtained
    // b) the iterations stall, ie., ||x(k) - x(k-1)|| <= tol
    rk_->PenalizedNaturalResidual(*xk_);
    Ek = rk_->Norm();

    if (Ek <= abs_tol_ + E0 * rel_tol_ || dx_->Norm() <= stall_tol_) {
      output.eflag = SUCCESS;
      output.residual = Ek;
      output.newton_iters = newton_iters_;
      output.prox_iters = prox_iters_;
      x0->Copy(*xk_);

      PrintIterLine(prox_iters_, newton_iters_, *rk_, *ri_, inner_tol);
      PrintFinal(prox_iters_, newton_iters_, output.eflag, *rk_);

      return output;
    } else {
      PrintDetailedHeader(prox_iters_, newton_iters_, *rk_);
      PrintIterLine(prox_iters_, newton_iters_, *rk_, *ri_, inner_tol);
    }

    // TODO(dliaomcp@umich.edu) Check if the residual is decreasing.
    // TODO(dliaomcp@umich.edu) Implement adaptive rule for decreasing sigma.

    // Update subproblem tolerance.
    inner_tol = saturate(inner_tol * inner_tol_multiplier_, inner_tol_min_, Ek);

    // Solve the proximal subproblem.
    xi_->Copy(*xk_);
    double Eo = SolveProximalSubproblem(xi_, xk_, inner_tol, sigma_, Ek);
    if (newton_iters_ >= max_newton_iters_) {
      output.eflag = MAXITERATIONS;
      if (Eo < Ek) {
        x0->Copy(*xi_);
        output.residual = Eo;
      } else {
        x0->Copy(*xk_);
        output.residual = Ek;
        rk_->PenalizedNaturalResidual(*xk_);
      }
      output.newton_iters = newton_iters_;
      output.prox_iters = prox_iters_;
      PrintFinal(prox_iters_, newton_iters_, output.eflag, *rk_);
      return output;
    }

    // Compute dx <- x(k+1) - x(k).
    dx_->Copy(*xi_);
    dx_->axpy(*xk_, -1.0);
    // Check for infeasibility.
    if (check_feasibility_) {
      InfeasibilityStatus status = CheckInfeasibility(*dx_);
      if (status != FEASIBLE) {
        if (status == PRIMAL) {
          output.eflag = PRIMAL_INFEASIBLE;
        } else if (status == DUAL) {
          output.eflag = DUAL_INFEASIBLE;
        } else if (status == BOTH) {
          output.eflag = PRIMAL_DUAL_INFEASIBLE;
        }
        output.residual = Ek;
        output.newton_iters = newton_iters_;
        output.prox_iters = prox_iters_;
        x0->Copy(*dx_);
        PrintFinal(prox_iters_, newton_iters_, output.eflag, *rk_);
        return output;
      }
    }
    // x(k+1) = x(i)
    xk_->Copy(*xi_);
    prox_iters_++;
  }  // end proximal loop

  // Timeout exit.
  output.eflag = MAXITERATIONS;
  output.residual = Ek;
  output.newton_iters = newton_iters_;
  output.prox_iters = prox_iters_;
  x0->Copy(*xk_);
  PrintFinal(prox_iters_, newton_iters_, output.eflag, *rk_);

  return output;
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
double FBstabAlgorithm<Variable, Residual, Data, LinearSolver, Feasibility>::
    SolveProximalSubproblem(Variable* x, Variable* xbar, double tol,
                            double sigma, double current_outer_residual) {
  ClearMeritBuffer();

  double Eo = 0;   // KKT residual.
  double t = 1.0;  // Linesearch parameter.
  for (int i = 0; i < max_inner_iters_; i++) {
    // Compute subproblem residual.
    ri_->InnerResidual(*x, *xbar, sigma);
    double Ei = ri_->Norm();
    // Compute KKT residual.
    rk_->PenalizedNaturalResidual(*x);
    Eo = rk_->Norm();

    // The inner loop stops if:
    // a) The subproblem is solved to the prescribed
    // tolerance and the outer residual has been reduced.
    // b) The outer residual cannot be decreased
    // (this can happen if the problem is infeasible).
    if ((Ei <= tol && Eo < current_outer_residual) || (Ei <= inner_tol_min_)) {
      PrintDetailedLine(i, t, *ri_);
      PrintDetailedFooter(tol, *ri_);
      break;
    } else {
      PrintDetailedLine(i, t, *ri_);
    }
    if (newton_iters_ >= max_newton_iters_) {
      break;
    }

    // Solve for the Newton step.
    linear_solver_->Factor(*x, *xbar, sigma);
    ri_->Negate();
    linear_solver_->Solve(*ri_, dx_);
    newton_iters_++;

    // Perform a non-monotone linesearch.
    const double current_merit = ri_->Merit();
    InsertMerit(current_merit);
    const double m0 = MaxMerit();
    t = 1.0;
    for (int j = 0; j < max_linesearch_iters_; j++) {
      // Compute a trial point xp = x + t*dx
      // and evaluate the merit function at xp.
      xp_->Copy(*x);
      xp_->axpy(*dx_, t);
      ri_->InnerResidual(*xp_, *xbar, sigma);
      const double mp = ri_->Merit();
      // Armijo descent check.
      if (mp <= m0 - 2.0 * t * eta_ * current_merit) {
        break;
      } else {
        t *= beta_;
      }
    }
    x->axpy(*dx_, t);  // x <- x + t*dx
  }
  // Make duals non-negative.
  x->ProjectDuals();

  return Eo;
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
typename FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                         Feasibility>::InfeasibilityStatus
FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                Feasibility>::CheckInfeasibility(const Variable& x) {
  feasibility_->ComputeFeasibility(x, infeasibility_tol_);

  InfeasibilityStatus status = FEASIBLE;
  if (!feasibility_->IsPrimalFeasible()) {
    status = PRIMAL;
  }
  if (!feasibility_->IsDualFeasible()) {
    status = DUAL;
  }
  if (!feasibility_->IsDualFeasible() && !feasibility_->IsPrimalFeasible()) {
    status = BOTH;
  }
  return status;
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::UpdateOption(const char* option,
                                                double value) {
  if (strcmp(option, "abs_tol")) {
    abs_tol_ = max(value, 1e-14);
  } else if (strcmp(option, "rel_tol")) {
    rel_tol_ = max(value, 0.0);
  } else if (strcmp(option, "stall_tol")) {
    stall_tol_ = max(value, 1e-14);
  } else if (strcmp(option, "infeas_tol")) {
    infeasibility_tol_ = max(value, 1e-14);
  } else if (strcmp(option, "sigma0")) {
    sigma0_ = max(value, 1e-14);
  } else if (strcmp(option, "alpha")) {
    alpha_ = max(value, 0.001);
    alpha_ = min(alpha_, 0.999);
  } else if (strcmp(option, "beta")) {
    beta_ = max(value, 0.1);
    beta_ = min(beta_, 0.99);
  } else if (strcmp(option, "eta")) {
    eta_ = max(value, 1e-12);
    eta_ = min(eta_, 0.499);
  } else if (strcmp(option, "inner_tol_multiplier")) {
    inner_tol_multiplier_ = max(value, 0.0001);
    inner_tol_multiplier_ = min(inner_tol_multiplier_, 0.99);
  } else if (strcmp(option, "inner_tol_max")) {
    inner_tol_max_ = max(value, 1e-8);
    inner_tol_max_ = min(inner_tol_max_, 100.0);
  } else if (strcmp(option, "inner_tol_min")) {
    inner_tol_min_ = max(value, 1e-14);
    inner_tol_min_ = min(inner_tol_min_, 1e-2);
  } else {
    printf("%s is not an option, no action taken\n", option);
  }
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::UpdateOption(const char* option, int value) {
  if (strcmp(option, "max_newton_iters")) {
    max_newton_iters_ = max(value, 1);
  } else if (strcmp(option, "max_prox_iters")) {
    max_prox_iters_ = max(value, 1);
  } else if (strcmp(option, "max_inner_iters")) {
    max_inner_iters_ = max(value, 1);
  } else if (strcmp(option, "max_linesearch_iters")) {
    max_linesearch_iters_ = max(value, 1);
  } else {
    printf("%s is not an option, no action taken\n", option);
  }
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::UpdateOption(const char* option,
                                                bool value) {
  if (strcmp(option, "check_feasibility")) {
    check_feasibility_ = value;
  } else {
    printf("%s is not an option, no action taken\n", option);
  }
}

// static functions *************************************
template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
bool FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::strcmp(const char* x, const char* y) {
  for (int i = 0; x[i] != '\0' || y[i] != '\0'; i++) {
    if (x[i] != y[i]) {
      return false;
    }
  }
  return true;
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::InsertMerit(double x) {
  for (int i = merit_buffer_.size() - 1; i > 0; i--) {
    merit_buffer_.at(i) = merit_buffer_.at(i - 1);
  }
  merit_buffer_.at(0) = x;
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
double FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                       Feasibility>::MaxMerit() {
  double current_max = merit_buffer_.at(0);
  for (unsigned long i = 1; i < merit_buffer_.size(); i++) {
    if (merit_buffer_.at(i) > current_max) {
      current_max = merit_buffer_.at(i);
    }
  }
  return current_max;
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::ClearMeritBuffer() {
  for (unsigned long i = 0; i < merit_buffer_.size(); i++) {
    merit_buffer_.at(i) = 0.0;
  }
}

// printing *************************************
template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::PrintFinal(int prox_iters, int newton_iters,
                                              ExitFlag eflag,
                                              const Residual& r) {
  if (display_level_ >= FINAL) {
    printf("Optimization completed!  Exit code:");
    switch (eflag) {
      case SUCCESS:
        printf(" Success\n");
        break;
      case DIVERGENCE:
        printf(" Divergence\n");
        break;
      case MAXITERATIONS:
        printf(" Iteration limit exceeded\n");
        break;
      case PRIMAL_INFEASIBLE:
        printf(" Primal Infeasibility\n");
        break;
      case DUAL_INFEASIBLE:
        printf(" Dual Infeasibility\n");
        break;
      default:
        printf(" ???\n");
    }
    printf("Proximal iterations: %d out of %d\n", prox_iters, max_prox_iters_);
    printf("Newton iterations: %d out of %d\n", newton_iters,
           max_newton_iters_);
    printf("%10s  %10s  %10s  %10s\n", "|rz|", "|rl|", "|rv|", "Tolerance");
    printf("%10.4e  %10.4e  %10.4e  %10.4e\n", r.z_norm(), r.l_norm(),
           r.v_norm(), abs_tol_);
    printf("\n");
  }
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::PrintIterHeader() {
  if (display_level_ == ITER) {
    printf("%12s  %12s  %12s  %12s  %12s  %12s  %12s\n", "prox iter",
           "newton iters", "|rz|", "|rl|", "|rv|", "Inner res", "Inner tol");
  }
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::PrintIterLine(int prox_iters,
                                                 int newton_iters,
                                                 const Residual& rk,
                                                 const Residual& ri,
                                                 double itol) {
  if (display_level_ == ITER) {
    printf("%12d  %12d  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e\n", prox_iters,
           newton_iters, rk.z_norm(), rk.l_norm(), rk.v_norm(), ri.Norm(),
           itol);
  }
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::PrintDetailedHeader(int prox_iters,
                                                       int newton_iters,
                                                       const Residual& r) {
  if (display_level_ == ITER_DETAILED) {
    double t = r.Norm();
    printf("Begin Prox Iter: %d, Total Newton Iters: %d, Residual: %6.4e\n",
           prox_iters, newton_iters, t);
    printf("%10s  %10s  %10s  %10s  %10s\n", "Iter", "Step Size", "|rz|",
           "|rl|", "|rv|");
  }
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::PrintDetailedLine(int iter,
                                                     double step_length,
                                                     const Residual& r) {
  if (display_level_ == ITER_DETAILED) {
    printf("%10d  %10e  %10e  %10e  %10e\n", iter, step_length, r.z_norm(),
           r.l_norm(), r.v_norm());
  }
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
void FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                     Feasibility>::PrintDetailedFooter(double tol,
                                                       const Residual& r) {
  if (display_level_ == ITER_DETAILED) {
    printf(
        "Exiting inner loop. Inner residual: %6.4e, Inner tolerance: %6.4e\n",
        r.Norm(), tol);
  }
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
