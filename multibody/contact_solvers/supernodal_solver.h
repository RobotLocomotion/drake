#pragma once

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"

// Forward declaration to avoid the inclusion of conex's headers within a Drake
// header.
namespace conex {
class SupernodalKKTSolver;
}

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using BlockMatrixTriplet = std::tuple<int, int, Eigen::MatrixXd>;

// A supernodal Cholesky solver for solving the symmetric positive definite
// system
//   H⋅x = b
// where H = M + Jᵀ G J. The matrices M and J are set at construction and the
// weight matrix G is set with SetWeightMatrix(), which can be called multiple
// times on a constructed object.
//
// Example use case:
//
//  SuperNodalSolver solver( ... );
//  solver.SetWeightMatrix( ... );
//  solver.Factor();
//
//  // Solve H⋅x1 = b1.
//  x1 = solver.Solve(b1);
//  // Solve H⋅x2 = b2. This reuses the factorization (important for speed!).
//  x2 = solver.Solve(b2);
//
//  // Update weight matrix and refactor.
//  solver.SetWeightMatrix( ... );
//  solver.Factor();
//  // Solve H⋅x = b using updated factorization.
//  x = solver.Solve(b);
class SuperNodalSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SuperNodalSolver)

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
  SuperNodalSolver(int num_jacobian_row_blocks,
                   const std::vector<BlockMatrixTriplet>& jacobian_blocks,
                   const std::vector<Eigen::MatrixXd>& mass_matrices);

  ~SuperNodalSolver();

  // Sets the block-diagonal weight matrix G.  The block rows of J and G both
  // partition the set {1, 2, ..., num_rows(J)}. Similar to the mass_matrix,
  // the partition induced by G must refine the partition induced by J,
  // otherwise an exception is thrown.
  void SetWeightMatrix(const std::vector<Eigen::MatrixXd>& block_diagonal_G);

  // Returns the M + J^T G J as a dense matrix (for debugging).
  // Throws if Factor() has been called (since factorization
  // is done in place. Throws if SetWeightMatrix has not been called.
  Eigen::MatrixXd MakeFullMatrix() const;

  // Computes the supernodal LLT factorization. Returns true if factorization
  // succeeds, otherwise returns false.  Failure is triggered by an internal
  // failure of Eigen::LLT.  This can fail if, for instance, the input matrix M
  // + J^T G J is not positive definite. If failure is encountered, the user
  // should verify that the specified matrix M + J^T G J is positive definite
  // and not poorly conditioned.  Throws if SetWeightMatrix() has not been
  // called.
  bool Factor();

  // Solves the system H⋅x = b and returns x.
  // Throws if Factor() has not been called.
  Eigen::VectorXd Solve(const Eigen::VectorXd& b) const;

  // Solves the system H⋅x = b and writes the result in b.
  // Throws if Factor() has not been called.
  void SolveInPlace(Eigen::VectorXd* b) const;

 private:
  // This class is responsible for filling a dense matrix of the form
  // sub_matrix(M) +  Jᵀₚ Gₚ Jₚ where Jₚ is a block row of the Jacobian. Each
  // row of the Jacobian can have at most two blocks, i.e. Jₚ = [Jₚ,ₜ₁ Jₚ,ₜ₂],
  // where blocks t₁ and t₂ correspond to block-diagonal entries in M. That is,
  // sub_matrix(M) = diag(Mₜ₁ Mₜ₂).
  class CliqueAssembler;

  void Initialize(const std::vector<std::vector<int>>& cliques,
                  int num_jacobian_row_blocks,
                  const std::vector<BlockMatrixTriplet>& jacobian_blocks,
                  const std::vector<Eigen::MatrixXd>& mass_matrices);

  bool factorization_ready_ = false;
  bool matrix_ready_ = false;

  std::unique_ptr<::conex::SupernodalKKTSolver> solver_;
  // N.B. This array stores pointers to clique assemblers owned by
  // owned_clique_assemblers_.
  std::vector<CliqueAssembler*> clique_assemblers_ptrs_;
  std::vector<std::unique_ptr<CliqueAssembler>> owned_clique_assemblers_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
