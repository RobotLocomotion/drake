#pragma once

#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/linear_operator.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* A LinearOperator that provides the inverse operator for a symmetric positive
 definite (SPD) matrix.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class InverseSpdOperator final : public LinearOperator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseSpdOperator)

  /* Constructs an operator with given `name` implementing the LinearOperator
   interface for the inverse of the given SPD matrix `A`.
   @pre `A` is SPD.
   @warn The constructor does not check that the given matrix `A` is SPD. If the
   matrix provided at construction is not SPD, this class does not guarantee to
   provide correct results. */
  InverseSpdOperator(const std::string& name, const MatrixX<T>& A);

  ~InverseSpdOperator() = default;

  int rows() const final { return size_; }
  int cols() const final { return size_; }

 private:
  void DoMultiply(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                  Eigen::SparseVector<T>* y) const final;

  void DoMultiply(const Eigen::Ref<const VectorX<T>>& x,
                  VectorX<T>* y) const final;

  void DoMultiplyByTranspose(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                             Eigen::SparseVector<T>* y) const final {
    DoMultiply(x, y);
  }

  void DoMultiplyByTranspose(const Eigen::Ref<const VectorX<T>>& x,
                             VectorX<T>* y) const final {
    DoMultiply(x, y);
  }

  int size_;
  Eigen::LLT<MatrixX<T>> A_llt_;
};
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::InverseSpdOperator)
