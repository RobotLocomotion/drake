#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {
namespace fbstab {

// Return codes for the solver.
enum class ExitFlag {
  SUCCESS = 0,
  DIVERGENCE = 1,
  MAXITERATIONS = 2,
  PRIMAL_INFEASIBLE = 3,
  DUAL_INFEASIBLE = 4,
  PRIMAL_DUAL_INFEASIBLE = 5
};

/**
 * Packages the exit flag, overall residual, solve time,
 * and iteration counts.
 *
 * A negative valuve for solve_time indicates that no timing data is available.
 */
struct SolverOut {
  ExitFlag eflag = ExitFlag::MAXITERATIONS;
  double residual = 0.0;
  int newton_iters = 0;
  int prox_iters = 0;
  double solve_time = 0.0;  /// CPU time in s.
};

using clock = std::chrono::high_resolution_clock;
/**
 * This class implements the FBstab solver for
 * convex quadratic programs, see
 * https://arxiv.org/pdf/1901.04046.pdf for more details.
 *
 * FBstab tries to solve instances of the following convex QP:
 *
 *     min.  1/2 z'*H*z + f'*z
 *
 *     s.t.  Gz =  h
 *           Az <= b
 *
 * The algorithm is implemented using to abstract objects
 * representing variables, residuals etc.
 * These are template parameters for the class and
 * should be written so as to be efficient for specific classes
 * of QPs, e.g., model predictive control QPs or sparse QPs.
 *
 * The algorithm exits when: ||π(x)|| <= abs_tol + ||π(x0)|| rel_tol
 * where π is the natural residual function,
 * (17) in https://arxiv.org/pdf/1901.04046.pdf.
 *
 * @tparam Variable:      storage and methods for primal-dual variables
 * @tparam Residual:      storage and methods for QP residuals
 * @tparam Data:          QP specific data storage and operations
 * @tparam LinearSolver:  solves Newton step systems
 * @tparam Feasibility:   checks for primal-dual infeasibility
 */
template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
class FBstabAlgorithm {
 public:
  /** Display settings */
  enum class Display {
    OFF = 0,           ///< no display
    FINAL = 1,         ///< prints message upon completion
    ITER = 2,          ///< basic information at each outer loop iteration
    ITER_DETAILED = 3  ///< print detailed inner loop information
  };

  /**
   * Saves the components objects needed by the solver.
   *
   * @param[in] x1,x2,x3,x4 Variable objects used by the solver
   * @param[in] r1,r2 Residual objects used by the solver
   * @param[in] lin_sol Linear solver used by the solver
   * @param[in] fcheck Feasibility checker used by the solver
   */
  FBstabAlgorithm(Variable* x1, Variable* x2, Variable* x3, Variable* x4,
                  Residual* r1, Residual* r2, LinearSolver* lin_sol,
                  Feasibility* fcheck) {
    if (x1 == nullptr || x2 == nullptr || x3 == nullptr || x4 == nullptr) {
      throw std::runtime_error(
          "A Variable supplied to FBstabAlgorithm is null.");
    }
    if (r1 == nullptr || r2 == nullptr) {
      throw std::runtime_error(
          "A Residual supplied to FBstabAlgorithm is null");
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

  /**
   * Attempts to solve the QP for the given
   * data starting from the supplied initial guess.
   *
   * @param[in] qp_data problem data
   * @param[in,out] x0    initial primal-dual guess, overwritten with the
   * solution
   *
   * @return Details on the solver output
   */
  SolverOut Solve(const Data* qp_data, Variable* x0);

  /**
   * Allows setting of algorithm options.
   * @param[in] option option name
   * @param[in] value  new value
   *
   * Possible options and default parameters are:
   * - sigma0{1e-8}: Initial stabilization parameter
   * - alpha{0.95}:  Penalized FB function parameter
   * - beta{0.7}:    Backtracking linesearch parameter
   * - eta{1e-8}:    Sufficient decrease parameter
   * - inner_tol_multiplier{0.2}: Reduction factor for subproblem tolerance
   *
   * - abs_tol{1e-6}: Absolute tolerance
   * - rel_tol{1e-12}: Relative tolerance
   * - stall_tol{1e-10}: Tolerance on ||dx||
   * - infeas_tol{1e-8}: Relative tolerance used in feasibility checking
   *
   * - inner_tol_max{1.0}: Maximum value for the subproblem tolerance
   * - inner_tol_min{1e-12}: Minimum value for the subproblem tolerance
   *
   * - max_newton_iters{200}: Maximum number of Newton iterations before timeout
   * - max_prox_iters{30}: Maximum number of proximal iterations before timeout
   * - max_inner_iters{50}: Maximum number of iterations that can be applied
   * to a single subproblem
   * - max_linesearch_iters{20}: Maximum number of backtracking linesearch steps
   *
   * - check_feasibility{true}: Enables or disables the feasibility checker,
   * if the problem is known to be feasible then it can be disabled for speed.
   */
  void UpdateOption(const char* option, double value) {
    if (std::strcmp(option, "abs_tol") == 0) {
      abs_tol_ = std::max(value, 1e-14);
    } else if (std::strcmp(option, "rel_tol") == 0) {
      rel_tol_ = std::max(value, 0.0);
    } else if (std::strcmp(option, "stall_tol") == 0) {
      stall_tol_ = std::max(value, 1e-14);
    } else if (std::strcmp(option, "infeas_tol") == 0) {
      infeasibility_tol_ = std::max(value, 1e-14);
    } else if (std::strcmp(option, "sigma0") == 0) {
      sigma0_ = std::max(value, 1e-14);
    } else if (std::strcmp(option, "alpha") == 0) {
      alpha_ = saturate(value, 0.001, 0.999);
    } else if (std::strcmp(option, "beta") == 0) {
      beta_ = saturate(value, 0.1, 0.99);
    } else if (std::strcmp(option, "eta") == 0) {
      eta_ = saturate(value, 1e-12, 0.499);
    } else if (std::strcmp(option, "inner_tol_multiplier") == 0) {
      inner_tol_multiplier_ = saturate(value, 0.0001, 0.99);
    } else if (std::strcmp(option, "inner_tol_max") == 0) {
      inner_tol_max_ = saturate(value, 1e-8, 100.0);
    } else if (std::strcmp(option, "inner_tol_min") == 0) {
      inner_tol_min_ = saturate(value, 1e-14, 1e-2);
    } else {
      printf("%s is not an option, no action taken\n", option);
    }
  }

  void UpdateOption(const char* option, int value) {
    if (std::strcmp(option, "max_newton_iters") == 0) {
      max_newton_iters_ = std::max(value, 1);
    } else if (std::strcmp(option, "max_prox_iters") == 0) {
      max_prox_iters_ = std::max(value, 1);
    } else if (std::strcmp(option, "max_inner_iters") == 0) {
      max_inner_iters_ = std::max(value, 1);
    } else if (std::strcmp(option, "max_linesearch_iters") == 0) {
      max_linesearch_iters_ = std::max(value, 1);
    } else {
      printf("%s is not an option, no action taken\n", option);
    }
  }
  void UpdateOption(const char* option, bool value) {
    if (std::strcmp(option, "check_feasibility") == 0) {
      check_feasibility_ = value;
    } else if (std::strcmp(option, "record_solve_time") == 0) {
      record_solve_time_ = value;
    } else {
      printf("%s is not an option, no action taken\n", option);
    }
  }

  /** Getter for display_level_ */
  Display get_display_level() const { return display_level_; }
  /** Setter for display_level_ */
  void set_display_level(Display v) { display_level_ = v; }

 private:
  using time_point = clock::time_point;
  // Codes for infeasibility detection.
  enum class InfeasibilityStatus {
    FEASIBLE = 0,
    PRIMAL = 1,
    DUAL = 2,
    BOTH = 3
  };

  bool record_solve_time_ = true;

  // Iteration counters.
  int newton_iters_ = 0;
  int prox_iters_ = 0;

  // Variable objects
  Variable* xk_ = nullptr;  // outer loop variable
  Variable* xi_ = nullptr;  // inner loop variable
  Variable* xp_ = nullptr;  // workspace
  Variable* dx_ = nullptr;  // workspace

  // Residual objects
  Residual* rk_ = nullptr;  // outer loop residual
  Residual* ri_ = nullptr;  // inner loop residual

  // Linear system solver object
  LinearSolver* linear_solver_ = nullptr;
  // Feasibility checker object
  Feasibility* feasibility_ = nullptr;

  // Algorithm parameters,
  // see https://arxiv.org/pdf/1901.04046.pdf for details.
  double sigma0_ = 1e-8;  // initial regularization parameter
  double alpha_ = 0.95;   // see (19) in https://arxiv.org/pdf/1901.04046.pdf
  double beta_ = 0.7;     // backtracking lineseach parameter
  double eta_ = 1e-8;     // sufficient decrease parameter
  double inner_tol_multiplier_ = 1.0 / 5;  // tolerance reduction factor

  double abs_tol_ = 1e-6;            // absolute tolerance
  double rel_tol_ = 1e-12;           // relative tolerance
  double stall_tol_ = 1e-10;         // change tolerance
  double infeasibility_tol_ = 1e-8;  // infeasibility tolerance

  // Tolerance guards.
  double inner_tol_max_ = 1e-1;
  double inner_tol_min_ = 1e-12;

  // Iteration guards.
  int max_newton_iters_ = 500;
  int max_prox_iters_ = 100;
  int max_inner_iters_ = 100;
  int max_linesearch_iters_ = 20;

  bool check_feasibility_ = true;

  // Default display settings.
  FBstabAlgorithm::Display display_level_ = Display::FINAL;

  // TODO(dliaomcp@umich.edu) Switch to circular buffer implementation to avoid
  // copy overhead.
  static constexpr int kNonMonotoneLineSearch = 5;
  static_assert(kNonMonotoneLineSearch > 0,
                "kNonMonotoneLineSearch must be positive");
  std::array<double, kNonMonotoneLineSearch> merit_buffer_ = {
      {0.0, 0.0, 0.0, 0.0, 0.0}};

  /*
   * Attempts to solve a proximal subproblem x = P(xbar,sigma) using
   * the semismooth Newton's method. See (11) in
   * https://arxiv.org/pdf/1901.04046.pdf.
   *
   * @param[both]  x      Initial guess, overwritten with the solution.
   * @param[in]    xbar   Current proximal (outer) iterate
   * @param[in]    tol    Desired tolerance for the inner residual
   * @param[in]    sigma  Regularization strength
   * @param[in]    Eouter Current overall problem residual
   * @return Residual for the outer problem evaluated at x
   *
   * This method uses the member variables rk_, ri_, dx_, and xp_ as workspaces.
   */
  double SolveProximalSubproblem(Variable* x, Variable* xbar, double tol,
                                 double sigma, double current_outer_residual);

  /*
   * Prepares a suitable output structure.
   *
   * @param[in] e exit flag
   * @param[in] prox_iters
   * @param[in] newton_iters
   * @param[in] r
   * @param[in] start time instant when the solve call started
   */
  SolverOut PrepareOutput(ExitFlag e, int prox_iters, int newton_iters,
                          const Residual& r, time_point start) const {
    struct SolverOut output = {
        ExitFlag::MAXITERATIONS,  // exit flag
        0.0,                      // residual
        0,                        // prox iters
        0,                        // newton iters
        -1.0 / 1000.0,            // solve time
                                  // (-ve value indicates no timing data
                                  // available)
    };

    if (record_solve_time_) {
      time_point now = clock::now();
      std::chrono::duration<double> elapsed = now - start;
      output.solve_time = elapsed.count();
    }
    output.eflag = e;
    output.residual = r.Norm();
    output.newton_iters = newton_iters;
    output.prox_iters = prox_iters;

    // Printing is in ms.
    PrintFinal(prox_iters, newton_iters, e, r, 1000.0 * output.solve_time);
    return output;
  }

  /**
   * Checks if x certifies primal or dual infeasibility.
   * @param[in]  x
   * @return feasibility status
   */
  InfeasibilityStatus CheckInfeasibility(const Variable& x) {
    feasibility_->ComputeFeasibility(x, infeasibility_tol_);

    InfeasibilityStatus status = InfeasibilityStatus::FEASIBLE;
    if (!feasibility_->IsPrimalFeasible()) {
      status = InfeasibilityStatus::PRIMAL;
    }
    if (!feasibility_->IsDualFeasible()) {
      status = InfeasibilityStatus::DUAL;
    }
    if (!feasibility_->IsDualFeasible() && !feasibility_->IsPrimalFeasible()) {
      status = InfeasibilityStatus::BOTH;
    }
    return status;
  }

  /**
   * Shifts all elements in merit_buffer_ up one spot then inserts at [0].
   * @param[in] x value to be inserted at merit_buffer_[0]
   */
  void InsertMerit(double x) {
    for (int i = static_cast<int>(merit_buffer_.size()) - 1; i > 0; i--) {
      merit_buffer_.at(i) = merit_buffer_.at(i - 1);
    }
    merit_buffer_.at(0) = x;
  }

  // @return maximum value in merit_buffer_
  double MaxMerit() const {
    return *std::max_element(merit_buffer_.begin(), merit_buffer_.end());
  }

  // Projects x onto [a,b].
  template <class T>
  static T saturate(const T& x, const T& a, const T& b) {
    if (a > b) {
      throw std::runtime_error(
          "In FBstabAlgorithm::saturate: upper bound must be larger than the "
          "lower bound");
    }
    const T temp = std::min(x, b);
    return std::max(temp, a);
  }

  // Prints a header line to stdout depending on display settings.
  void PrintIterHeader() const {
    if (display_level_ == Display::ITER) {
      printf("%12s  %12s  %12s  %12s  %12s  %12s  %12s\n", "prox iter",
             "newton iters", "|rz|", "|rl|", "|rv|", "Inner res", "Inner tol");
    }
  }

  // Prints an iteration progress line to stdout depending on display settings.
  void PrintIterLine(int prox_iters, int newton_iters, const Residual& rk,
                     const Residual& ri, double itol) const {
    if (display_level_ == Display::ITER) {
      printf("%12d  %12d  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e\n", prox_iters,
             newton_iters, rk.z_norm(), rk.l_norm(), rk.v_norm(), ri.Norm(),
             itol);
    }
  }

  // Prints a detailed header line to stdout depending on display settings.
  void PrintDetailedHeader(int prox_iters, int newton_iters,
                           const Residual& r) const {
    if (display_level_ == Display::ITER_DETAILED) {
      double t = r.Norm();
      printf("Begin Prox Iter: %d, Total Newton Iters: %d, Residual: %6.4e\n",
             prox_iters, newton_iters, t);
      printf("%10s  %10s  %10s  %10s  %10s\n", "Iter", "Step Size", "|rz|",
             "|rl|", "|rv|");
    }
  }

  // Prints inner loop iterations details to stdout depending on display
  // settings.
  void PrintDetailedLine(int iter, double step_length,
                         const Residual& r) const {
    if (display_level_ == Display::ITER_DETAILED) {
      printf("%10d  %10e  %10e  %10e  %10e\n", iter, step_length, r.z_norm(),
             r.l_norm(), r.v_norm());
    }
  }

  // Prints a footer to stdout depending on display settings.
  void PrintDetailedFooter(double tol, const Residual& r) const {
    if (display_level_ == Display::ITER_DETAILED) {
      printf(
          "Exiting inner loop. Inner residual: %6.4e, Inner tolerance: "
          "%6.4e\n",
          r.Norm(), tol);
    }
  }
  // Prints a summary to stdout depending on display settings.
  void PrintFinal(int prox_iters, int newton_iters, ExitFlag eflag,
                  const Residual& r, double t) const {
    if (display_level_ >= Display::FINAL) {
      printf("\nOptimization completed!  Exit code:");
      switch (eflag) {
        case ExitFlag::SUCCESS:
          printf(" Success\n");
          break;
        case ExitFlag::DIVERGENCE:
          printf(" Divergence\n");
          break;
        case ExitFlag::MAXITERATIONS:
          printf(" Iteration limit exceeded\n");
          break;
        case ExitFlag::PRIMAL_INFEASIBLE:
          printf(" Primal Infeasibility\n");
          break;
        case ExitFlag::DUAL_INFEASIBLE:
          printf(" Dual Infeasibility\n");
          break;
        case ExitFlag::PRIMAL_DUAL_INFEASIBLE:
          printf(" Primal-Dual Infeasibility\n");
          break;
        default:
          DRAKE_UNREACHABLE();
      }
      printf("Time elapsed: %f ms (-1.0 indicates timing disabled)\n", t);
      printf("Proximal iterations: %d out of %d\n", prox_iters,
             max_prox_iters_);
      printf("Newton iterations: %d out of %d\n", newton_iters,
             max_newton_iters_);
      printf("%10s  %10s  %10s  %10s\n", "|rz|", "|rl|", "|rv|", "Tolerance");
      printf("%10.4e  %10.4e  %10.4e  %10.4e\n", r.z_norm(), r.l_norm(),
             r.v_norm(), abs_tol_);
      printf("\n");
    }
  }
};

// TODO(dliaomcp@umich.edu): Enable printing to a log file rather than just
// stdout
template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
SolverOut FBstabAlgorithm<Variable, Residual, Data, LinearSolver,
                          Feasibility>::Solve(const Data* qp_data,
                                              Variable* x0) {
  const time_point start_time{record_solve_time_ ? clock::now()
                                                 : time_point{} /* dummy */};

  // Make sure the linear solver and residuals objects are using the same value
  // for the alpha parameter.
  rk_->SetAlpha(alpha_);
  ri_->SetAlpha(alpha_);
  linear_solver_->SetAlpha(alpha_);

  // Supply a pointer to the data object.
  xk_->LinkData(qp_data);
  xi_->LinkData(qp_data);
  dx_->LinkData(qp_data);
  xp_->LinkData(qp_data);
  x0->LinkData(qp_data);

  // Initialization phase.
  const double sigma = sigma0_;

  x0->InitializeConstraintMargin();
  xk_->Copy(*x0);
  xi_->Copy(*x0);
  dx_->Fill(1.0);

  rk_->NaturalResidual(*xk_);
  ri_->Fill(0.0);
  double E0 = rk_->Norm();
  double Ek = E0;
  double inner_tol = saturate(E0, inner_tol_min_, inner_tol_max_);

  // Reset iteration count.
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
      PrintIterLine(prox_iters_, newton_iters_, *rk_, *ri_, inner_tol);
      SolverOut output = PrepareOutput(ExitFlag::SUCCESS, prox_iters_,
                                       newton_iters_, *rk_, start_time);
      x0->Copy(*xk_);
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
    const double Eo = SolveProximalSubproblem(xi_, xk_, inner_tol, sigma, Ek);
    // Iteration timeout check.
    if (newton_iters_ >= max_newton_iters_) {
      if (Eo < Ek) {
        x0->Copy(*xi_);
        SolverOut output = PrepareOutput(ExitFlag::MAXITERATIONS, prox_iters_,
                                         newton_iters_, *rk_, start_time);
        return output;
      } else {
        x0->Copy(*xk_);
        rk_->PenalizedNaturalResidual(*xk_);
        SolverOut output = PrepareOutput(ExitFlag::MAXITERATIONS, prox_iters_,
                                         newton_iters_, *rk_, start_time);
        return output;
      }
    }

    // Compute dx <- x(k+1) - x(k).
    dx_->Copy(*xi_);
    dx_->axpy(-1.0, *xk_);
    // Check for infeasibility.
    if (check_feasibility_) {
      InfeasibilityStatus status = CheckInfeasibility(*dx_);
      if (status != InfeasibilityStatus::FEASIBLE) {
        if (status == InfeasibilityStatus::PRIMAL) {
          SolverOut output =
              PrepareOutput(ExitFlag::PRIMAL_INFEASIBLE, prox_iters_,
                            newton_iters_, *rk_, start_time);
          x0->Copy(*dx_);
          return output;
        } else if (status == InfeasibilityStatus::DUAL) {
          SolverOut output =
              PrepareOutput(ExitFlag::DUAL_INFEASIBLE, prox_iters_,
                            newton_iters_, *rk_, start_time);
          x0->Copy(*dx_);
          return output;
        } else {
          SolverOut output =
              PrepareOutput(ExitFlag::PRIMAL_DUAL_INFEASIBLE, prox_iters_,
                            newton_iters_, *rk_, start_time);
          x0->Copy(*dx_);
          return output;
        }
      }
    }
    // x(k+1) = x(i)
    xk_->Copy(*xi_);
    prox_iters_++;
  }  // end proximal loop

  // Timeout exit.
  SolverOut output = PrepareOutput(ExitFlag::MAXITERATIONS, prox_iters_,
                                   newton_iters_, *rk_, start_time);
  x0->Copy(*xk_);
  return output;
}

template <class Variable, class Residual, class Data, class LinearSolver,
          class Feasibility>
double FBstabAlgorithm<Variable, Residual, Data, LinearSolver, Feasibility>::
    SolveProximalSubproblem(Variable* x, Variable* xbar, double tol,
                            double sigma, double current_outer_residual) {
  merit_buffer_.fill(0.0);  // Clear the buffer of past merit function values.

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
    const bool initialize_flag = linear_solver_->Initialize(*x, *xbar, sigma);
    if (initialize_flag != true) {
      throw std::runtime_error(
          "In FBstabAlgorithm::Solve: LinearSolver::Initialize failed.");
    }
    ri_->Negate();

    const bool solve_flag = linear_solver_->Solve(*ri_, dx_);
    if (solve_flag != true) {
      throw std::runtime_error(
          "In FBstabAlgorithm::Solve: LinearSolver::Solve failed.");
    }
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
      xp_->axpy(t, *dx_);
      ri_->InnerResidual(*xp_, *xbar, sigma);
      const double mp = ri_->Merit();
      // Armijo descent check.
      if (mp <= m0 - 2.0 * t * eta_ * current_merit) {
        break;
      } else {
        t *= beta_;
      }
    }
    x->axpy(t, *dx_);  // x <- x + t*dx
  }
  // Make duals non-negative.
  x->ProjectDuals();

  return Eo;
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
