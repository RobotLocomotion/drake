#pragma once

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"
#include "drake/multibody/contact_solvers/matrix_block.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using BlockMatrixTriplet = std::tuple<int, int, MatrixBlock<double>>;

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
  //   number of scalar decision variables. These two partitions must be the
  //   same, otherwise an exception is thrown.
  SuperNodalSolver(int num_jacobian_row_blocks,
                   const std::vector<BlockMatrixTriplet>& jacobian_blocks,
                   const std::vector<Eigen::MatrixXd>& mass_matrices);

  ~SuperNodalSolver();

  // Sets the block-diagonal weight matrix G.  The block rows of J and G both
  // partition the set {1, 2, ..., num_rows(J)}.
  // The partition induced by G must refine the partition induced by J,
  // otherwise an exception is thrown.  (See
  // https://en.wikipedia.org/wiki/Partition_of_a_set for definition of
  // refinement.)
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
  bool factorization_ready_ = false;
  bool matrix_ready_ = false;
  std::unique_ptr<BlockSparseSymmetricMatrix> A_;
  // The indices into `jacobian_blocks_` on row i organized by column.
  std::vector<std::vector<int>> row_to_triplet_list_;
  std::vector<Eigen::MatrixXd> mass_matrices_;
  std::vector<BlockMatrixTriplet> jacobian_blocks_;
  std::unique_ptr<BlockSparseCholeskySolver<Eigen::MatrixXd>> solver_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
