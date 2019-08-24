#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace solvers {
namespace fbstab {

/**
 * Represents data for quadratic programing problems of the following type (1):
 *
 * min.    1/2  z'Hz + f'z
 * s.t.         Az <= b
 *
 * where H is symmetric and positive semidefinite.
 */
class DenseData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DenseData)
  /**
   * Stores the problem data and performs input validation.
   * This class assumes that the pointers to the data remain valid.
   *
   * @param[in] H Hessian matrix
   * @param[in] f Linear term
   * @param[in] A Constraint matrix
   * @param[in] b Constraint vector
   *
   * Throws a runtime exception if any of the inputs are null or if
   * the sizes of the inputs are inconsistent.
   */
  DenseData(const Eigen::MatrixXd* H, const Eigen::VectorXd* f,
            const Eigen::MatrixXd* A, const Eigen::VectorXd* b);

  /** Read only accessor for the H matrix. */
  const Eigen::MatrixXd& H() const { return *H_; }

  /** Read only accessor for the f vector. */
  const Eigen::VectorXd& f() const { return *f_; }

  /** Read only accessor for the A matrix. */
  const Eigen::MatrixXd& A() const { return *A_; }

  /** Read only accessor for the b vector. */
  const Eigen::VectorXd& b() const { return *b_; }

  /**
   * @return number of decision variables (i.e., dimension of z)
   */
  int num_variables() const { return nz_; }
  /**
   * @return number of inequality constraints
   */
  int num_constraints() const { return nv_; }

 private:
  int nz_ = 0;  // Number of decision variables.
  int nv_ = 0;  // Number of constraints.

  const Eigen::MatrixXd* const H_{nullptr};
  const Eigen::VectorXd* const f_{nullptr};
  const Eigen::MatrixXd* const A_{nullptr};
  const Eigen::VectorXd* const b_{nullptr};

  friend class DenseVariable;
  friend class DenseResidual;
  friend class DenseLinearSolver;
  friend class DenseFeasibility;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
