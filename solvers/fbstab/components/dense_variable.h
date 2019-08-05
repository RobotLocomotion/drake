#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/components/dense_data.h"

namespace drake {
namespace solvers {
namespace fbstab {

/**
 * Implements primal-dual variables for inequality constrained QPs,
 * see dense_data.h for a mathematical description.
 * This class stores variables and defines methods implementing useful
 * operations.
 *
 * Primal-dual variables have 3 components:
 * - z: Decision variables
 * - v: Inequality duals
 * - y: Inequality margins
 *
 * where
 * length(z) = nz
 * length(v) = nv
 * length(y) = nv
 */
class DenseVariable {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DenseVariable)

  /**
   * Allocates memory for a primal-dual variables.
   *
   * @param[in] nz Number of decision variables > 0
   * @param[in] nv Number of inequality constraints > 0
   */
  DenseVariable(int nz, int nv);

  /**
   * Creates a primal-dual variable using preallocated memory.
   * @param[in] z    A vector to store the decision variables.
   * @param[in] v    A vector to store the dual variables.
   * @param[in] y    A vector to store the inequality margin.
   *
   * Throws an exception if any inputs are null or have mismatched or zero size.
   */
  DenseVariable(Eigen::VectorXd* z, Eigen::VectorXd* v, Eigen::VectorXd* y);

  /**
   * Links to problem data needed to perform calculations,
   * Calculations cannot be performed until a data object is provided.
   * @param[in] data Pointer to the problem data
   */
  void LinkData(const DenseData* data) { data_ = data; }

  /**
   * Fills the variable with one value,
   * i.e., x <- a * ones.
   * @param[in] a
   *
   * Throws an exception if problem data has not been linked.
   */
  void Fill(double a);

  /**
   * Sets the field x.y = b - A* x.z.
   * Throws an exception if problem data has not been linked.
   */
  void InitializeConstraintMargin();

  /**
   * Performs the operation *this <- a*x + *this
   * (where u is this object).
   * This is a level 1 BLAS operation for this object;
   * see http://www.netlib.org/blas/blasqr.pdf.
   *
   * @param[in] a scalar
   * @param[in] x vector
   *
   * Note that this handles the constraint margin correctly, i.e., after the
   * operation u.y = b - A*(u.z + a*x.z).
   * Throws an exception if problem data has not been linked.
   */
  void axpy(double a, const DenseVariable& x);

  /**
   * Performs a deep copy operation.
   * @param[in] x variable to be copied
   *
   * Throws an exception if sizes are mismatched.
   */
  void Copy(const DenseVariable& x);

  /**
   * Projects the inequality duals onto the non-negative orthant,
   * i.e., v <- max(0,v).
   */
  void ProjectDuals();

  /**
   * Computes the Euclidean norm.
   * @return sqrt(|z|^2 + |v|^2)
   */
  double Norm() const;

  /** Accessor for the primal variable. */
  Eigen::VectorXd& z() { return *z_; }

  /** Accessor for the dual variable. */
  Eigen::VectorXd& v() { return *v_; }

  /** Accessor for the constraint margin. */
  Eigen::VectorXd& y() { return *y_; }

  /** Accessor for the primal variable. */
  const Eigen::VectorXd& z() const { return *z_; }

  /** Accessor for the dual variable. */
  const Eigen::VectorXd& v() const { return *v_; }

  /** Accessor for the constraint margin. */
  const Eigen::VectorXd& y() const { return *y_; }

  int num_constraints() const { return nv_; }
  int num_variables() const { return nz_; }

 private:
  int nz_ = 0;  // Number of decision variable
  int nv_ = 0;  // Number of inequality constraints
  const DenseData* data_ = nullptr;
  const DenseData* data() const;
  Eigen::VectorXd* z_ = nullptr;  // primal variable
  Eigen::VectorXd* v_ = nullptr;  // dual variable
  Eigen::VectorXd* y_ = nullptr;  // inequality margin
  std::unique_ptr<Eigen::VectorXd> z_storage_;
  std::unique_ptr<Eigen::VectorXd> v_storage_;
  std::unique_ptr<Eigen::VectorXd> y_storage_;

  friend class DenseResidual;
  friend class DenseLinearSolver;
  friend class DenseFeasibility;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
