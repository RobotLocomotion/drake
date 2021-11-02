#pragma once

#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "conex/kkt_solver.h"
#include <Eigen/Dense>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {


// MatrixBlock::first contains the data of a submatrix and MatrixBlock::second
// contains the columns numbers. The rows are inferred.
//
//using MatrixBlockData = std::pair<Eigen::MatrixXd, int>;
//
//using MatrixBlocks = std::vector<MatrixBlock>;
//
//using JacobianRowData = std::vector<MatrixBlockData>;

// A supernodal solver for the solving the symmetric positive definite system
//   H⋅x = b where H = M + J^T G J.  The matrices M and J are set by the
//   constructor and the weight matrix G is set by SetWeightMatrix(), which can
//   be called multiple times on a constructed object.
//
// Mass matrix M layout: M is a block diagonal matrix where the t-th diagonal
//   entry Mₜ has size nₜ×nₜ. M has size nᵥ×nᵥ, with nᵥ = ∑nₜ.
//
// Weight matrix G layout: G is a block diagonal matrix where the k-th block has
//   size nₖ×nₖ. The size of the matrix is r×r, with r = ∑nₖ.
//
// Jacobian J layout: Block Jₚₜ of the Jacobian has size nₚ×nₜ. The number of
//   columns in the Jacobian is nᵥ. The number of rows is r = ∑nₚ.
//
// Example use case:
//
//  SolverNodalSolver solver( ... ); solver.SetWeightMatrix( ... );
//  solver.Factor();
//
//  // Solve Tx = b1. x1 = solver.Solve(b1); // Resolve Tx = b2. x2 =
//  solver.Solve(b2);
//
//  // Update weight matrix and refactor solver.SetWeightMatrix( ... );
//  solver.Factor(); // Solve Tx = b1 with different weight matrix G. x1 =
//  solver.Solve(b1);
using BlockMatrixTriplet = std::tuple<int, int, Eigen::MatrixXd>;


class SuperNodalSolver {
 public:

  // @param num_jacobian_row_blocks Number of row blocks in the matrix J.
  // @param jacobian_blocks Blocks Bij provided as triplets (i, j, Bij).
  // @param mass_matrices Block diagonal matrix M provided as a vector of block
  // diagonal entries.
  //
  // If M_{ij} is nonzero in the matrix H = M + J^T G J, then [J^T G J]_{ij} 
  // must also be nonzero for generic G.
  //  If this condition fails, an exception is thrown. 
  SuperNodalSolver(int num_jacobian_row_blocks,
                   const std::vector<BlockMatrixTriplet>& jacobian_blocks,
                   const std::vector<Eigen::MatrixXd>& mass_matrices);

  // Sets the block-diagonal matrix G. The block sizes of G must refine
  // the partition of the matrix J that was specified by the input
  // to the constructor. For instance, if J is partitioned like
  //
  //  J1 
  //  J2 J3
  //
  //  Then we require existence of n and m such that
  //
  //  num_rows(J1) = \sum^n_{i=1} num_rows (G_i),  
  //  num_rows(J2) = \sum^{m}_{i=n+1} num_rows (G_i)
  //
  //  If this condition fails, an exception is thrown unless NDEBUG is defined. 
  void SetWeightMatrix(const std::vector<Eigen::MatrixXd>& block_diagonal_G);

  // Returns the M + J^T G J as a dense matrix (for debugging).
  Eigen::MatrixXd FullMatrix() {
    if (!matrix_ready_) {
      throw std::runtime_error(
          std::string("Call to FullMatrix() failed: weight matrix not set or "
                      "matrix already factored."));
    }
    return solver_.KKTMatrix();
  }

  // Computes the supernodal LLT factorization.
  void Factor();

  // Solves the system H⋅x = b and returns x.
  Eigen::MatrixXd Solve(const Eigen::VectorXd& b);

  // Solves the system H⋅x = b and writes the result in b.
  void SolveInPlace(Eigen::VectorXd* b);

 private:
  class CliqueAssembler final : public ::conex::LinearKKTAssemblerBase {
   public:
    void SetDenseData() override;

    void SetWeightMatrixIndex(int start, int end) {
      weight_start_ = start;
      weight_end_ = end;
    }

    void SetWeightMatrixPointer(
        const std::vector<Eigen::MatrixXd>* weight_matrix) {
      weight_matrix_ = weight_matrix;
    }

    void AssignMassMatrix(int i, const Eigen::MatrixXd& A) {
      mass_matrix_position_.push_back(i);
      mass_matrix_.push_back(A);
    }
    int NumRows() { return row_data_.at(0).rows(); }

    void SetMatrixBlocks(const std::vector<Eigen::MatrixXd>& r) {
      row_data_ = r;
      temporaries_.resize(r.size());
      int num_vars = 0;
      for (size_t j = 0; j < row_data_.size(); j++) {
        num_vars += r.at(j).cols();
        temporaries_.at(j).resize(r.at(j).rows(), r.at(j).cols());
      }

      LinearKKTAssemblerBase::SetNumberOfVariables(num_vars);
      const int size = SizeOf(LinearKKTAssemblerBase::schur_complement_data);
      workspace_memory_.resize(size);
      LinearKKTAssemblerBase::schur_complement_data.InitializeWorkspace(
          workspace_memory_.data());
    }

   private:
    void BuildSubmatrixFromWeightedRow(const Eigen::MatrixXd& A);
    std::vector<Eigen::MatrixXd> row_data_;
    std::vector<int> mass_matrix_position_;
    std::vector<Eigen::MatrixXd> mass_matrix_;
    std::vector<Eigen::MatrixXd> temporaries_;
    Eigen::VectorXd workspace_memory_;
    const std::vector<Eigen::MatrixXd>* weight_matrix_;
    int weight_start_ = 0;
    int weight_end_ = 0;
  };

 private:

  using MatrixBlock = std::pair<Eigen::MatrixXd, std::vector<int>>;
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

  void Initialize(const std::vector<std::vector<int>>& cliques,
                  const std::vector<std::vector<Eigen::MatrixXd>>& row_data);
  SparsityData GetEliminationOrdering(int num_jacobian_row_blocks, 
                            const std::vector<BlockMatrixTriplet>& jacobian_blocks);


  bool factorization_ready_ = false;
  bool matrix_ready_ = false;
  bool weight_matrix_ready_ = false;
  const std::vector<MatrixBlock> mass_matrices_;
  std::vector<Eigen::MatrixXd> weight_matrices_;
  std::vector<std::vector<int>> cliques_;
  SparsityData clique_data_;
  ::conex::Solver solver_;
  std::vector<CliqueAssembler*> clique_assemblers_;
  std::vector<CliqueAssembler> jacobian_assemblers_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
