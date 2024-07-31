#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/contact_solvers/supernodal_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Supernodal Cholesky solver implemented by the SuperNodalKKTSolver in Conex.
class ConexSuperNodalSolver final : public SuperNodalSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConexSuperNodalSolver);

  // @param num_jacobian_row_blocks
  //   Number of row blocks in the matrix J.
  // @param jacobian_blocks
  //   A vector of triplets (p, t, Jₚₜ) specifying the non-zero blocks of the
  //   Jacobian matrix.  The number of block columns nₜ is inferred
  //   from the largest column index t in the vector of triplets (p, t, Jₚₜ).
  //   An exception is thrown if any of the following conditions fail:
  //     1) There is at least one triplet  (p, t, Jₚₜ) with column index t for
  //     each t ∈ {1,nₜ}.
  //     2) There is at most two triplets (p, t, Jₚₜ) with the same row
  //     index p.
  // @param mass_matrices
  //   Specifies a block-diagonal mass matrix M of size nᵥ x nᵥ.  The block
  //   columns of the mass matrix and the block columns of the Jacobian J both
  //   induce a partition of the set {0, 1, ..., nᵥ}, where nᵥ denotes the
  //   number of scalar decision variables. The partition induced by M must
  //   refine the partition induced by J, otherwise an exception is thrown. (See
  //   https://en.wikipedia.org/wiki/Partition_of_a_set for definition of
  //   refinement.)  For instance, if J has block structure
  //     J = |J₁  0|
  //         |J₂ J₃|
  //   Then we require existence of n such that
  //     num_cols(J₁) =  ∑num_cols(Mₜ), t = 1…n
  //     num_cols(J₃) =  ∑num_cols(Mₜ), t = n+1…nᵥ
  //   If this condition fails, an exception is thrown.
  ConexSuperNodalSolver(int num_jacobian_row_blocks,
                        const std::vector<BlockTriplet>& jacobian_blocks,
                        const std::vector<Eigen::MatrixXd>& mass_matrices);

  ~ConexSuperNodalSolver();

 private:
  // This class is responsible for filling a dense matrix of the form
  // sub_matrix(M) +  Jᵀₚ Gₚ Jₚ where Jₚ is a block row of the Jacobian. Each
  // row of the Jacobian can have at most two blocks, i.e. Jₚ = [Jₚ,ₜ₁ Jₚ,ₜ₂],
  // where blocks t₁ and t₂ correspond to block-diagonal entries in M. That is,
  // sub_matrix(M) = diag(Mₜ₁ Mₜ₂).
  class CliqueAssembler;

  /* NVI implementations. */
  bool DoSetWeightMatrix(
      const std::vector<Eigen::MatrixXd>& block_diagonal_G) final;
  Eigen::MatrixXd DoMakeFullMatrix() const final;
  bool DoFactor() final;
  void DoSolveInPlace(Eigen::VectorXd* b) const final;
  int DoGetSize() const final { return size_; }

  void Initialize(const std::vector<std::vector<int>>& cliques,
                  int num_jacobian_row_blocks,
                  const std::vector<BlockTriplet>& jacobian_blocks,
                  const std::vector<Eigen::MatrixXd>& mass_matrices);

  // We use 'void' here to avoid the inclusion of conex's headers within a
  // Drake header; the actual type of the object is a SupernodalKKTSolver.
  // We use a shared_ptr to have a type-erased deleter; the object is not
  // actually ever shared anywhere.
  std::shared_ptr<void> solver_;

  // N.B. This array stores pointers to clique assemblers owned by
  // owned_clique_assemblers_.
  std::vector<CliqueAssembler*> clique_assemblers_ptrs_;
  std::vector<std::unique_ptr<CliqueAssembler>> owned_clique_assemblers_;
  int size_{};
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
