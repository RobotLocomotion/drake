#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/petsc_symmetric_block_sparse_matrix.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Wrapper around PETSc symmetric block sparse matrix.
 @tparam_nonsymbolic_scalar */
class PetscSchurComplement {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PetscSchurComplement);

  ~PetscSchurComplement();

  /* Constructs the Schur complement for an empty matrix, i.e., A, B, and D are
   all empty. */
  PetscSchurComplement();

  PetscSchurComplement(
      const PetscSymmetricBlockSparseMatrix& symmetric_block_sparse_matrix,
      const std::vector<int>& eliminated, const std::vector<int>& rest);

  std::unique_ptr<PetscSchurComplement> Clone() const;

  void CalcSchurComplement(
      const PetscSymmetricBlockSparseMatrix& symmetric_block_sparse_matrix,
      const std::vector<int>& eliminated, const std::vector<int>& rest);

  const MatrixX<double>& get_D_complement() const;

  VectorX<double> SolveForY(const Eigen::Ref<const VectorX<double>>& x) const;

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
