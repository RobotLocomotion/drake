#pragma once

#include "drake/multibody/contact_solvers/linear_operator.h"

namespace drake {
namespace multibody {
namespace fem {
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
  virtual ~LinearSystemSolver() = default;

  /* Performs the linear solve to get x = A⁻¹b. */
  virtual void Solve(const Eigen::Ref<const VectorX<T>>& b,
                     EigenPtr<VectorX<T>> x) const = 0;

  /* Returns the size of the linear system, which is equal to the number of rows
   and columns of the matrix A. */
  int size() const {
    DRAKE_THROW_UNLESS(A_ != nullptr);
    return A_->rows();
  }

  /* Perform precomputes on A such as factorize the matrix. This method only
   needs to be called every time A changes. A common workflow looks like:
   ```
   // Precompute (e.g. factorize) A.
   linear_solver.Compute();
   // Solve for Ax₁ = b₁.
   linear_solver.Solve(b1, &x1);
   // Solve for Ax₂ = b₂ without factorizing A again.
   linear_solver.Solve(b2, &x2);
   ...
   ``` */
  void Compute() {
    DRAKE_THROW_UNLESS(A_ != nullptr);
    DRAKE_THROW_UNLESS(A_->rows() == A_->cols());
    DoCompute();
  }

  /* Returns the underlying linear operator. Useful for debugging purposes.
   */
  const contact_solvers::internal::LinearOperator<T>& A() {
    DRAKE_THROW_UNLESS(A_ != nullptr);
    return *A_;
  }

  /* Sets the tolerance for iterative solvers. No-op for direct solvers. */
  virtual void set_tolerance(const T&) {}

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemSolver);
  /* Constructs a new linear solver with the prescribed linear operator A.
   This class keeps a reference to input linear operator `A` and therefore it
   is required that `A` outlives this object.
   @param[in] A    The linear operator that describes the linear system. */
  explicit LinearSystemSolver(
      const contact_solvers::internal::LinearOperator<T>* A)
      : A_(A) {
    DRAKE_DEMAND(A_ != nullptr);
  }

  /* Derived classes should override this method to perform percomputes (e.g.
   factorize) on A, the left hand side of the system, if necessary. */
  virtual void DoCompute() {}

 private:
  const contact_solvers::internal::LinearOperator<T>* A_{nullptr};
};
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
