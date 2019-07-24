#pragma once

#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>

namespace drake {
namespace solvers {
namespace fbstab {

// Return codes for the solver.
enum ExitFlag {
  SUCCESS = 0,
  DIVERGENCE = 1,
  MAXITERATIONS = 2,
  PRIMAL_INFEASIBLE = 3,
  DUAL_INFEASIBLE = 4,
  PRIMAL_DUAL_INFEASIBLE = 5
};

/**
 * Packages the exit flag, overall residual,
 * and iteration counts.
 */
struct SolverOut {
  ExitFlag eflag;
  double residual;
  int newton_iters;
  int prox_iters;
};

/**
 * This class implements the FBstab solver for
 * convex quadratic programs, see
 * https://arxiv.org/pdf/1901.04046.pdf for more details.
 *
 * FBstab tries to solve instances of the following convex QP:
 *
 * min.  1/2 z'*H*z + f'*z
 *
 * s.t.  Gz =  h
 *       Az <= b
 *
 * The algorithm is implemented using to abstract objects
 * representing variables, residuals etc.
 * These are template parameters for the class and
 * should be written so as to be efficient for specific classes
 * of QPs, e.g., model predictive control QPs or sparse QPs.
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
  /**
   * Display settings
   */
  enum Display {
    OFF = 0,           // no display
    FINAL = 1,         // prints message upon completion
    ITER = 2,          // basic information at each outer loop iteration
    ITER_DETAILED = 3  // print detailed inner loop information
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
                  Feasibility* fcheck);

  /**
   * Attempts to solve the QP for the given
   * data starting from the supplied initial guess.
   *
   * @param[in] qp_data problem data
   * @param[both] x0    initial primal-dual guess, overwritten with the solution
   *
   * @return Details on the solver output
   */
  SolverOut Solve(const Data* qp_data, Variable* x0);

  /**
   * Allows setting of algorithm options.
   * @param[in] option option name
   * @param[in] value  new value

   * Possible options and default parameters are:
   * sigma0{1e-8}: Initial stabilization parameter
   * alpha{0.95}:  Penalized FB function parameter
   * beta{0.7}:    Backtracking linesearch parameter
   * eta{1e-8}:    Sufficient decrease parameter
   * inner_tol_multiplier{0.2}: Reduction factor for subproblem tolerance
   *
   * The algorithm exists when: ||\pi(x)|| <= abs_tol + ||\pi(x0)|| rel_tol
   * where \pi is the natural residual function,
   * (17) in https://arxiv.org/pdf/1901.04046.pdf.
   *
   * abs_tol{1e-6}: Absolute tolerance
   * rel_tol{1e-12}: Relative tolerance
   * stall_tol{1e-10}: If the residual doesn't decrease by at least this failure
   * is declared
   * infeas_tol{1e-8}: Relative tolerance used in feasibility checking
   *
   * inner_tol_max{1.0}: Maximum value for the subproblem tolerance
   * inner_tol_min{1e-12}: Minimum value for the subproblem tolerance
   *
   * max_newton_iters{200}: Maximum number of Newton iterations before timeout
   * max_prox_iters{30}: Maximum number of proximal iterations before timeout
   * max_inner_iters{50}: Maximum number of iterations that can be applied
   * to a single subproblem
   * max_linesearch_iters{20}: Maximum number of backtracking linesearch steps
   *
   * check_feasibility{true}: Enables or disables the feasibility checker,
   * if the problem is known to be feasible then it can be disabled for speed.
   */
  void UpdateOption(const char* option, double value);
  void UpdateOption(const char* option, int value);
  void UpdateOption(const char* option, bool value);

  /** Acessor for display_level_ */
  Display& display_level() { return display_level_; }

 private:
  // Codes for infeasibility detection.
  enum InfeasibilityStatus { FEASIBLE = 0, PRIMAL = 1, DUAL = 2, BOTH = 3 };

  /**
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
   * Note: Uses
   * rk_, ri_, dx_, and xp_ as workspaces.
   */
  double SolveProximalSubproblem(Variable* x, Variable* xbar, double tol,
                                 double sigma, double current_outer_residual);

  /**
   * Checks if x certifies primal or dual infeasibility.
   * @param[in]  x
   * @return feasibility status
   */
  InfeasibilityStatus CheckInfeasibility(const Variable& x);

  static constexpr int knonmonotone_linesearch = 5;
  std::array<double, knonmonotone_linesearch> merit_buffer_ = {0.0};

  /**
   * Shifts all elements in merit_buffer_ up one spot then inserts at [0].
   * @param[in] x value to be inserted at merit_buffer_[0]
   */
  void InsertMerit(double x);

  // Sets all elements in merit_buffer_ to 0.
  void ClearMeritBuffer();

  // @return maximum value in merit_buffer_
  double MaxMerit();

  // Element wise max.
  template <class T>
  static T max(T a, T b) {
    return (a > b) ? a : b;
  }

  // Element wise min.
  template <class T>
  static T min(T a, T b) {
    return (a > b) ? b : a;
  }

  // Projects x onto [a,b].
  template <class T>
  static T saturate(T x, T a, T b) {
    const T temp = min(x, b);
    return max(temp, a);
  }

  /**
   * Compares two C style strings.
   * @param[in]  x
   * @param[in]  y
   * @return     true if x and y are equal
   */
  static bool strcmp(const char* x, const char* y);

  FBstabAlgorithm::Display display_level_ = FINAL;  // Default display settings.

  // Prints a header line to stdout depending on display settings.
  void PrintIterHeader();

  // Prints an iteration progress line to stdout depending on display settings.
  void PrintIterLine(int prox_iters, int newton_iters, const Residual& rk,
                     const Residual& ri, double itol);

  // Prints a detailed header line to stdout depending on display settings.
  void PrintDetailedHeader(int prox_iters, int newton_iters, const Residual& r);

  // Prints inner loop iterations details to stdout depending on display
  // settings.
  void PrintDetailedLine(int iter, double step_length, const Residual& r);

  // Prints a footer to stdout depending on display settings.
  void PrintDetailedFooter(double tol, const Residual& r);

  // Prints a summary to stdout depending on display settings.
  void PrintFinal(int prox_iters, int newton_iters, ExitFlag eflag,
                  const Residual& r);

  // Iteration counters.
  int newton_iters_ = 0;
  int prox_iters_ = 0;

  // Cariable objects
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
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake

#include "drake/solvers/fbstab/fbstab_algorithm-inl.h"
