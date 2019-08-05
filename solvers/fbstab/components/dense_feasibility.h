#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/components/dense_data.h"
#include "drake/solvers/fbstab/components/dense_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

/**
 * This class detects infeasibility in quadratic programs, see
 * dense_data.h for a description of the QPs.
 * It contains methods for determining if a primal-dual variable
 * is a certificate of primal and/or dual infeasibility.
 * It implements Algorithm 3 of https://arxiv.org/pdf/1901.04046.pdf.
 */
class DenseFeasibility {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DenseFeasibility)
  /**
   * Allocates workspace memory.
   * @param[in] nz number of decision variables
   * @param[in] nv number of inequality constraints
   *
   * Throws a runtime_error is any inputs are non-positive.
   */
  DenseFeasibility(int nz, int nv);

  /**
   * Checks if the primal-dual variable x
   * is a certificate of infeasibility and
   * stores the results internally.
   * It uses the results from Proposition 4 of
   * https://arxiv.org/pdf/1901.04046.pdf.
   *
   * @param[in] x   Variable to check
   * @param[in] tol Numerical tolerance
   *
   * Throws a runtime_error if tol isn't positive.
   */
  void ComputeFeasibility(const DenseVariable& x, double tol);

  /**
   * Returns the results of ComputeFeasibility
   * @return false if the last point checked certifies that
   *               the dual QP is infeasible, true otherwise
   */
  bool IsDualFeasible() const { return dual_feasible_; }

  /**
   * Returns the results of ComputeFeasibility
   * @return false if the last point checked certifies that
   *               the QP is infeasible, true otherwise
   */
  bool IsPrimalFeasible() const { return primal_feasible_; }

 private:
  int nz_ = 0;  // number of decision variables
  int nv_ = 0;  // number of inequality constraints

  // workspace vectors
  Eigen::VectorXd z1_;
  Eigen::VectorXd v1_;

  bool primal_feasible_ = true;
  bool dual_feasible_ = true;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
