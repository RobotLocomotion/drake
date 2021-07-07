#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/components/dense_data.h"
#include "drake/solvers/fbstab/components/dense_residual.h"
#include "drake/solvers/fbstab/components/dense_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

// Forward declaration of testing class to enable a friend declaration.
namespace test {
class DenseComponentUnitTests;
}  // namespace test

/**
 * A class for computing the search directions used by the FBstab QP Solver.
 * It solves systems of linear equations of the form
 *
 *      [Hs   A'] dz = rz  <==>  V*dx = r
 *      [-CA  D ] dv   rv
 *
 * using a Schur complement approach as described in (28) and (29) of
 * https://arxiv.org/pdf/1901.04046.pdf. Note that this code doesn't have
 * equality constraints so is a simplification of (28) and (29).
 *
 * This class allocates its own workspace memory and splits step computation
 * into solve and factor steps to allow for solving with multiple
 * right hand sides.
 *
 * This class has mutable fields and is thus not thread safe.
 *
 * Usage:
 * @code
 * DenseLinearSolver solver(2,2);
 * solver.Factor(x,xbar,sigma);
 * solver.Solve(r,&dx,sigma);
 * @endcode
 */
class DenseLinearSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DenseLinearSolver)
  /**
   * Allocates workspace memory.
   * @param [nz] Number of decision variables.
   * @param [nv] Number of inequality constraints.
   */
  DenseLinearSolver(int nz, int nv);

  /**
   * Factors the matrix V(x,xbar,sigma) using a Schur complement approach
   * followed by a Cholesky factorization and stores the factorization
   * internally.
   *
   * The matrix V is computed as described in
   * Algorithm 4 of https://arxiv.org/pdf/1901.04046.pdf.
   *
   * @param[in]  x       Inner loop iterate
   * @param[in]  xbar    Outer loop iterate
   * @param[in]  sigma   Regularization strength
   * @return             true if factorization succeeds false otherwise.
   *
   * Throws an exception if x and xbar aren't the correct size,
   * sigma is negative or the problem data isn't linked.
   */
  bool Initialize(const DenseVariable& x, const DenseVariable& xbar,
                  double sigma);

  /**
   * Solves the system V*x = r and stores the result in x.
   * This method assumes that the Factor routine was run to
   * compute then factor the matrix V.
   *
   * @param[in]   r   The right hand side vector
   * @param[out]  x   Overwritten with the solution
   * @return true if successful, false otherwise
   *
   * Throws an exception if x and r aren't the correct sizes,
   * if x is null or if the problem data isn't linked.
   */
  bool Solve(const DenseResidual& r, DenseVariable* x) const;

  /**
   * Sets the alpha parameter defined in (19)
   * of https://arxiv.org/pdf/1901.04046.pdf.
   */
  void SetAlpha(double alpha);

 private:
  friend class test::DenseComponentUnitTests;
  int nz_ = 0;  // number of decision variables
  int nv_ = 0;  // number of inequality constraints

  double alpha_ = 0.95;  // See (19) in https://arxiv.org/pdf/1901.04046.pdf.
  const double zero_tolerance_ = 1e-13;

  // workspace variables
  Eigen::MatrixXd K_;
  mutable Eigen::VectorXd r1_;
  mutable Eigen::VectorXd r2_;
  Eigen::VectorXd Gamma_;
  Eigen::VectorXd mus_;
  Eigen::VectorXd gamma_;
  Eigen::MatrixXd B_;

  // Computes the gradient of the penalized fischer-burmeister (PFB)
  // function, (19) in https://arxiv.org/pdf/1901.04046.pdf.
  // See section 3.3.
  Eigen::Vector2d PFBGradient(double a, double b) const;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
