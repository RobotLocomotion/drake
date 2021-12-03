#pragma once

#include <string>

#include <Eigen/SparseCore>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/linear_operator.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// A LinearOperator that wraps an existing BlockSparseMatrix.
//
// @tparam_nonsymbolic_scalar
template <typename T>
class BlockSparseLinearOperator final : public LinearOperator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BlockSparseLinearOperator)

  // Constructs an operator with given `name` implementing the LinearOperator
  // interface for matrix `A`.
  // This class keeps a reference to input matrix `A` and therefore it is
  // required that it outlives this object.
  BlockSparseLinearOperator(const std::string& name,
                            const BlockSparseMatrix<T>* A);

  ~BlockSparseLinearOperator() final = default;

  int rows() const final { return A_->rows(); }
  int cols() const final { return A_->cols(); }

 private:
  void DoMultiply(const Eigen::Ref<const VectorX<T>>& x,
                  VectorX<T>* y) const final;
  void DoMultiply(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                  Eigen::SparseVector<T>* y) const final;
  void DoMultiplyByTranspose(const Eigen::Ref<const VectorX<T>>& x,
                             VectorX<T>* y) const final;
  void DoMultiplyByTranspose(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                             Eigen::SparseVector<T>* y) const final;
  void DoAssembleMatrix(Eigen::SparseMatrix<T>* A) const final;
  void DoAssembleMatrix(BlockSparseMatrix<T>* A) const final;

  const BlockSparseMatrix<T>* const A_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        BlockSparseLinearOperator)
