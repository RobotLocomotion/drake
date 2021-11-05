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
class Solver;
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
// Matrix M layout:
//   M is symmetric positive definite with a block diagonal structure. The t-th
//   diagonal entry Mₜ has size nₜ×nₜ. M has size nᵥ×nᵥ, with nᵥ = ∑nₜ.
//
// Matrix J layout:
//   J will in general have a block sparse structure. Block Jₚₜ of the J has
//   size nₚ×nₜ. The number of columns in the Jacobian is nᵥ. The number of rows
//   is r = ∑nₚ.
//
// Weight matrix G layout:
//   G is a block diagonal matrix where the k-th block has size nₖ×nₖ. The size
//   of the matrix is r×r, with r = ∑nₖ.
//   Note: the block structure of J and G both partition the set {1, 2, ...,
//   num_rows(J) }.  We require that the partition induced by G refines the
//   partition induced by J. See
//   https://en.wikipedia.org/wiki/Partition_of_a_set for definition of
//   refinement.
//
// Example use case:
//
//  SolverNodalSolver solver( ... );
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
  //   Blocks Jₚₜ provided as triplets (p, t, Jₚₜ).
  //   Per each block row p, there can only be at most two blocks,
  //   otherwise an exception is thrown.
  // @param mass_matrices
  //   Block diagonal matrix M provided as a vector of block diagonal entries.
  SuperNodalSolver(int num_jacobian_row_blocks,
                   const std::vector<BlockMatrixTriplet>& jacobian_blocks,
                   const std::vector<Eigen::MatrixXd>& mass_matrices);

  ~SuperNodalSolver();

  // Sets the block-diagonal matrix G. The block sizes of G must refine
  // the partition of the matrix J that was specified by the input
  // to the constructor. For instance, if J is partitioned like
  //
  //   J = |J1  0|
  //       |J2 J3|
  //
  //  Then we require existence of n and m such that
  //
  //    num_rows(J1) = \sum^n_{i=1} num_rows (G_i),
  //    num_rows(J2) = \sum^{m}_{i=n+1} num_rows (G_i)
  //
  //  If this condition fails, an exception is thrown unless NDEBUG is defined.
  void SetWeightMatrix(const std::vector<Eigen::MatrixXd>& block_diagonal_G);

  // Returns the M + J^T G J as a dense matrix (for debugging).
  // Throws if Factor() has been called (since factorization
  // is done in place. Throws if SetWeightMatrix has not been called.
  Eigen::MatrixXd MakeFullMatrix();

  // Computes the supernodal LLT factorization. Returns true
  // if factorization succeeds, otherwise returns false.
  // Failure is triggered by an internal failure of Eigen::LLT.
  // This can fail if, for instance, the input matrix is not
  // positive definite. If failure is encountered, the user
  // should verify that the specified matrix M + J^T G H
  // is positive definite and not poorly conditioned.
  // Throws if SetWeightMatrix() has not been called.
  bool Factor();

  // Solves the system H⋅x = b and returns x.
  // Throws if Factor() has not been called.
  Eigen::VectorXd Solve(const Eigen::VectorXd& b);

  // Solves the system H⋅x = b and writes the result in b.
  // Throws if Factor() has not been called.
  void SolveInPlace(Eigen::VectorXd* b);

 private:
  // This class is responsible for filling a dense matrix of the form
  // sub_matrix(M) + J^T_i G_i J_i where J_i is a block row of the Jacobian and
  // sub_matrix(M) is specified by AssignMassMatrix.
  class CliqueAssembler;

  // Helper struct for assembling input into the
  // conex supernodal solver. Stores the cliques,
  // the partition of the cliques into supernodes
  // and seperators, and the order used for
  // elimination.
  struct SolverData {
    std::vector<std::vector<int>> cliques;
    int num_vars;
    std::vector<int> order;
    std::vector<std::vector<int>> supernodes;
    std::vector<std::vector<int>> separators;
  };

  struct SparsityData {
    SolverData data;
    std::vector<std::vector<int>> cliques_assembler;
  };

 private:
  using MatrixBlock = std::pair<Eigen::MatrixXd, std::vector<int>>;

  void Initialize(const std::vector<std::vector<int>>& cliques,
                  const std::vector<std::vector<Eigen::MatrixXd>>& row_data);
  SparsityData GetEliminationOrdering(
      int num_jacobian_row_blocks,
      const std::vector<BlockMatrixTriplet>& jacobian_blocks);

  bool factorization_ready_ = false;
  bool matrix_ready_ = false;
  const std::vector<MatrixBlock> mass_matrices_;
  std::vector<Eigen::MatrixXd> weight_matrices_;
  std::vector<std::vector<int>> cliques_;
  SparsityData clique_data_;
  std::unique_ptr<::conex::Solver> solver_;
  // N.B. This array stores pointers to clique assemblers owned by
  // owned_clique_assemblers_.
  std::vector<CliqueAssembler*> clique_assemblers_ptrs_;
  std::vector<std::unique_ptr<CliqueAssembler>> owned_clique_assemblers_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
