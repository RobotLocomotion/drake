#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/components/dense_data.h"
#include "drake/solvers/fbstab/components/dense_feasibility.h"
#include "drake/solvers/fbstab/components/dense_linear_solver.h"
#include "drake/solvers/fbstab/components/dense_residual.h"
#include "drake/solvers/fbstab/components/dense_variable.h"
#include "drake/solvers/fbstab/fbstab_algorithm.h"

namespace drake {
namespace solvers {
namespace fbstab {

/** Convenience type for the templated dense version of the algorithm. */
using FBstabAlgoDense = FBstabAlgorithm<DenseVariable, DenseResidual, DenseData,
                                        DenseLinearSolver, DenseFeasibility>;

/**
 * FBstabDense implements the Proximally Stabilized Semismooth Algorithm
 * for solving convex quadratic programs of the following form (1):
 *
 *     min.    1/2  z'Hz + f'z
 *     s.t.         Az <= b
 *
 * where H is symmetric and positive semidefinite and its dual
 *
 *     min.   1/2  z'Hz + b'v
 *     s.t.   Hz + f + A'v = 0
 *            v >= 0.
 *
 * Or equivalently for solving its KKT system
 *
 *     Hz + f + A' v = 0
 *     Az <= b, v >= 0
 *     (b - Az)' v = 0
 *
 * where v is a dual variable.
 *
 * The algorithm is described in https://arxiv.org/pdf/1901.04046.pdf.
 * Aside from convexity there are no assumptions made about the problem.
 * This method can detect unboundedness/infeasibility and accepts
 * arbitrary initial guesses.
 *
 * The problem is of size (nz,nv) where:
 * - nz > 0 is the number of decision variables
 * - nv > 0 is the number of inequality constraints
 *
 * Usage example:
 * @code
 * MatrixXd H(2,2);
 * MatrixXd A(1,2);
 * VectorXd f(2);
 * VectorXd b(1);
 *
 * H << 1,0,1,0;
 * A << 1,0;
 * f << 1,-1;
 * b << 0;
 *
 * FBstabDense::QPData data = {&H, &A, &f, &b};
 *
 * VectorXd x0 = VectorXd::Zero(2);
 * VectorXd v0 = VectorXd::Zero(1);
 * VectorXd y0 = VectorXd::Zero(1);
 *
 * FBstabDense::QPVariable x = {&x0, &v0, &y0};
 *
 * FBstabDense solver(2,1);
 * solver.Solve(data,x); // x is used as an initial guess then overwritten
 * @endcode
 */
class FBstabDense {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FBstabDense);
  /** Structure to hold the problem data. */
  struct QPData {
    /// nz x nz real positive semidefinite Hessian matrix.
    const Eigen::MatrixXd* H = nullptr;
    /// nv x nz real constraint Jacobian.
    const Eigen::MatrixXd* A = nullptr;
    /// nz real linear cost.
    const Eigen::VectorXd* f = nullptr;
    /// nv real constraint rhs.
    const Eigen::VectorXd* b = nullptr;
  };

  /**
   * Structure to hold the initial guess.
   * The vectors pointed to by z, v, and y WILL BE OVERWRITTEN
   * with the solution.
   */
  struct QPVariable {
    /// Decision variables in \reals^nz.
    Eigen::VectorXd* z = nullptr;
    /// Inequality duals in \reals^nv.
    Eigen::VectorXd* v = nullptr;
    /// Constraint margin, i.e., y = b-Az, in \reals^nv.
    Eigen::VectorXd* y = nullptr;
  };
  /**
   * Allocates needed workspace given the dimensions of the QPs to
   * be solved. Throws a runtime_error if any inputs are non-positive.
   *
   * @param[in] num_variables
   * @param[in] num_constraints
   */
  FBstabDense(int num_variables, int num_constraints);

  /**
   * Solves an instance of (1)
   *
   * @param[in]   qp  problem data
   *
   * @param[in,out] x   initial guess, overwritten with the solution
   *
   * @param[in] use_initial_guess if false the solver is initialized at the
   * origin.
   *
   * @return Summary of the optimizer output, see fbstab_algorithm.h.
   */
  SolverOut Solve(const QPData& qp, const QPVariable* x,
                  bool use_initial_guess = true);

  /**
   * Allows for setting of solver options. See fbstab_algorithm.h for
   * a list of adjustable options.
   * @param[in] option Option name
   * @param[in] value  New value
   */
  void UpdateOption(const char* option, double value);
  void UpdateOption(const char* option, int value);
  void UpdateOption(const char* option, bool value);

  /**
   * Controls the verbosity of the algorithm.
   * See fbstab_algorithm.h for details.
   * @param[in] level new display level
   */
  void SetDisplayLevel(FBstabAlgoDense::Display level);

 private:
  int nz_ = 0;
  int nv_ = 0;

  std::unique_ptr<FBstabAlgoDense> algorithm_;
  std::unique_ptr<DenseVariable> x1_;
  std::unique_ptr<DenseVariable> x2_;
  std::unique_ptr<DenseVariable> x3_;
  std::unique_ptr<DenseVariable> x4_;
  std::unique_ptr<DenseResidual> r1_;
  std::unique_ptr<DenseResidual> r2_;
  std::unique_ptr<DenseLinearSolver> linear_solver_;
  std::unique_ptr<DenseFeasibility> feasibility_checker_;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
