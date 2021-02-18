#pragma once

#include "drake/multibody/contact_solvers/linear_operator.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace internal {
/* A generic interface to solve a linear system of equations of the form
 Ax = b, where A ∈ ℝⁿˣⁿ and b ∈ ℝⁿ are given and x ∈ ℝⁿ is the unknown to be
 solved for. The unknown x and the right hand side b are dense. The linear
 operator A is represented by contact_solvers::internal::LinearOperator and may
 be dense or sparse.
 @tparam_nonsymbolic_scalar T */
template <typename T>
class LinearSystemSolver {
 public:
  virtual ~LinearSystemSolver() {}

  /* Performs the linear solve to get x = A⁻¹b. */
  virtual void Solve(const Eigen::Ref<const VectorX<T>>& b,
                     EigenPtr<VectorX<T>> x) const = 0;

  /* Returns the size of the linear system, which is equal to the number of rows
   and columns of the matrix A. */
  int size() const {
    DRAKE_THROW_UNLESS(A_ != nullptr);
    return A_->rows();
  }

  /* Sets up the left-hand side of the linear system. This class keeps a
   reference to input linear operator `A` and therefore it is
   required that `A` outlives this object. */
  void ResetOperator(const contact_solvers::internal::LinearOperator<T>* A) {
    DRAKE_THROW_UNLESS(A != nullptr);
    DRAKE_THROW_UNLESS(A->rows() == A->cols());
    A_ = A;
      DoResetOperator(A);
  }

  /* Returns the underlying linear operator. Useful for debugging purposes.
   */
  const contact_solvers::internal::LinearOperator<T>& A() {
    DRAKE_THROW_UNLESS(A_ != nullptr);
    return *A_;
  }

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemSolver);

  LinearSystemSolver() = default;

  /* Derived classes must implement this method to set up the left hand side of
   the system. */
  virtual void DoResetOperator(
      const contact_solvers::internal::LinearOperator<T>* A) = 0;

 private:
  const contact_solvers::internal::LinearOperator<T>* A_{nullptr};
};
}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
