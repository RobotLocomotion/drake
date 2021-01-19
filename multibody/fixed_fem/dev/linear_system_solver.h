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
    DRAKE_ASSERT(A_ != nullptr);
    return A_->rows();
  }

  /* Returns the underlying linear operator. Useful for debugging purposes. */
  const contact_solvers::internal::LinearOperator<T>& A() {
    DRAKE_ASSERT(A_ != nullptr);
    return *A_;
  }

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemSolver);

  /* Constructs a new linear solver with the prescribed linear operator A.
   This class keeps a reference to input linear operator `A` and therefore it is
   required that `A` outlives this object.
   @param[in] A    The linear operator that describes the linear system. The
   number of rows and columns must be the same for A.
   @throw std::exception if A.rows() != A.cols(). */
  explicit LinearSystemSolver(
      const contact_solvers::internal::LinearOperator<T>& A)
      : A_(&A) {
    DRAKE_THROW_UNLESS(A.rows() == A.cols());
  }

 private:
  const contact_solvers::internal::LinearOperator<T>* A_;
};
}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
