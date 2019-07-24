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

/**
 * @ file FBstabDense implements the Proximally Stabilized Semismooth Algorithm
 * for solving convex quadratic programs of the following form (1):
 *
 * min.    1/2  z'Hz + f'z
 * s.t.         Az <= b
 *
 * where H is symmetric and positive semidefinite and its dual
 *
 * min.   1/2  z'Hz + b'v
 * s.t.   Hz + f + A'v = 0
 *        v >= 0.
 *
 * Or equivalently for solving its KKT system
 *
 * Hz + f + A' v = 0
 * Az <= b, v >= 0
 * (b - Az)' v = 0
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
 * DenseQPData data = {&H, &A, &f, &b};
 *
 * VectorXd x0 = VectorXd::Zero(2);
 * VectorXd v0 = VectorXd::Zero(1);
 * VectorXd y0 = VectorXd::Zero(1);
 *
 * DenseQPVariable x = {&x0, &v0, &y0};
 *
 * FBstabDense solver(2,1);
 * solver.Solve(data,x); // x is used as an initial guess then overwritten
 * @endcode
 */

/**
 * Structure to hold the problem data.
 * Fields:
 * - H \in \reals^{nx x nx} is the Hessian
 * - A \in \reals^{nv x nx} is the constraint matrix
 * - f \in \reals^nx is the linear term
 * - b \in \reals^nv is the constraint vector
 */
struct DenseQPData {
  const Eigen::MatrixXd* H = nullptr;
  const Eigen::MatrixXd* A = nullptr;
  const Eigen::VectorXd* f = nullptr;
  const Eigen::VectorXd* b = nullptr;
};

/**
 * Structure to hold the initial guess.
 * The vectors pointed to by z, v, and y WILL BE OVERWRITTEN
 * with the solution.
 *
 * Fields:
 * - z \in \reals^nz are the decision variables
 * - v \in \reals^nv are the inequality duals
 * - y \in \reals^nv are the constraint margins, i.e., y = b - Az
 */
struct DenseQPVariable {
  Eigen::VectorXd* z = nullptr;
  Eigen::VectorXd* v = nullptr;
  Eigen::VectorXd* y = nullptr;
};

// Convenience type for the templated dense version of the algorithm.
using FBstabAlgoDense = FBstabAlgorithm<DenseVariable, DenseResidual, DenseData,
                                        DenseLinearSolver, DenseFeasibility>;

class FBstabDense {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FBstabDense);
  /**
   * Allocates needed workspace given the dimensions of the QPs to
   * be solved.
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
   * @param[both] x   initial guess, overwritten with the solution
   *
   * @param[in] use_initial_guess if false the solver is initialized at the
   * origin.
   *
   * @return 	summary of the optimizer output. Has the following fields:
   *
   * - eflag: ExitFlag enum (see fbstab_algorithm.h) indicating success or
   * failure
   * - residual: Norm of the KKT residual
   * - newton_iters: Number of Newton steps
   * - prox_iters: Number of proximal iterations
   */
  SolverOut Solve(const DenseQPData& qp, const DenseQPVariable& x,
                  bool use_initial_guess = true);

  /**
   * Allows for setting of solver options. See fbstab_algorithm.h for
   * a list of adjustable options
   * @param[in] option Option name
   * @param[in] value  New value
   */
  void UpdateOption(const char* option, double value);
  void UpdateOption(const char* option, int value);
  void UpdateOption(const char* option, bool value);

  /**
   * Controls the verbosity of the algorithm.
   * @param[in] level new display level
   *
   * Possible values are:
   * - OFF: Silent operation
   * - FINAL: Prints a summary at the end
   * - ITER: Major iterations details
   * - ITER_DETAILED: Major and minor iteration details
   *
   * The default value is FINAL.
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
