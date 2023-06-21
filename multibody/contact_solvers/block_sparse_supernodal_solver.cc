#include "drake/multibody/contact_solvers/block_sparse_supernodal_solver.h"

#include <utility>

using Eigen::MatrixXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

/* Input verification helper for the constructor of BlockSparseSuperNodalSolver.
 Returns true iff the given Jacobian matrix has the same partition as the
 given mass matrix.
 @param[in] jacobian_column_block_size The j-th entry contains the number of
 scalar columns in the j-th block column in the sparse Jacobian matrix.
 @param[in] mass_matrices  The i-th entry contains the i-th diagonal block of
 the block diagonal mass matrix. */
bool MassMatrixPartitionEqualsJacobianPartition(
    const std::vector<int>& jacobian_column_block_size,
    const std::vector<MatrixXd>& mass_matrices) {
  if (jacobian_column_block_size.size() != mass_matrices.size()) {
    return false;
  }
  for (int i = 0; i < ssize(mass_matrices); ++i) {
    if (jacobian_column_block_size[i] != mass_matrices[i].rows()) {
      return false;
    }
  }
  return true;
}

}  // namespace

BlockSparseSuperNodalSolver::BlockSparseSuperNodalSolver(
    int num_jacobian_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks,
    const std::vector<Eigen::MatrixXd>& mass_matrices)
    : mass_matrices_(mass_matrices),
      jacobian_blocks_(jacobian_blocks),
      solver_(std::make_unique<BlockSparseCholeskySolver<Eigen::MatrixXd>>()) {
  const std::vector<int> jacobian_column_block_size =
      GetJacobianBlockSizesVerifyTriplets(jacobian_blocks);
  /* Throw an exception if verification fails. */
  if (!MassMatrixPartitionEqualsJacobianPartition(jacobian_column_block_size,
                                                  mass_matrices)) {
    throw std::runtime_error(
        "Mass matrices and constraint jacobians are incompatible.");
  }
  row_to_triplet_index_ =
      GetRowToTripletMapping(num_jacobian_row_blocks, jacobian_blocks);

  const int num_nodes = mass_matrices.size();
  std::vector<int> block_sizes(num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    block_sizes[i] = mass_matrices[i].rows();
  }
  /* Build diagonal entry in sparsity pattern. */
  std::vector<std::vector<int>> sparsity(num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    sparsity[i].emplace_back(i);
  }
  /* Build off-diagonal entry in sparsity pattern. */
  const int num_constraints = row_to_triplet_index_.size();
  for (int r = 0; r < num_constraints; ++r) {
    const std::vector<int>& triplets = row_to_triplet_index_[r];
    DRAKE_DEMAND(triplets.size() <= 2);
    if (triplets.size() == 2) {
      const int j = std::get<1>(jacobian_blocks[triplets[0]]);
      const int i = std::get<1>(jacobian_blocks[triplets[1]]);
      DRAKE_DEMAND(j < i);
      sparsity[j].emplace_back(i);
    }
  }
  BlockSparsityPattern block_sparsity_pattern(std::move(block_sizes),
                                              std::move(sparsity));
  H_ = std::make_unique<BlockSparseSymmetricMatrix>(
      std::move(block_sparsity_pattern));
  /* The solver analyzes the sparsity pattern of the H_ (currently a zero
   matrix) so that subsequent updates to the matrix can use UpdateMatrix()
   that doesn't perform symbolic factorization and allocation. */
  solver_->SetMatrix(*H_);
}

BlockSparseSuperNodalSolver::~BlockSparseSuperNodalSolver() = default;

bool BlockSparseSuperNodalSolver::DoSetWeightMatrix(
    const std::vector<Eigen::MatrixXd>& weight_matrix) {
  H_->SetZero();
  /* Add mass matrices. */
  const int block_cols = mass_matrices_.size();
  for (int i = 0; i < block_cols; ++i) {
    H_->SetBlock(i, i, mass_matrices_[i]);
  }
  /* Add in JᵀGJ terms. */
  const int num_constraints = row_to_triplet_index_.size();
  DRAKE_THROW_UNLESS(ssize(weight_matrix) >= num_constraints);
  int weight_start = 0;
  int weight_end = 0;
  for (int k = 0; k < num_constraints; ++k) {
    const std::vector<int>& triplets = row_to_triplet_index_[k];
    const int num_constraint_equations =
        std::get<2>(jacobian_blocks_[triplets[0]]).rows();
    int G_rows = 0;
    while (G_rows < num_constraint_equations) {
      G_rows += weight_matrix[weight_end++].rows();
    }
    if (G_rows != num_constraint_equations) {
      return false;
    }

    if (triplets.size() == 1) {
      const MatrixBlock<double>& J = std::get<2>(jacobian_blocks_[triplets[0]]);
      const MatrixBlock<double> GJ = J.LeftMultiplyByBlockDiagonal(
          weight_matrix, weight_start, weight_end - 1);
      MatrixXd JTGJ = MatrixXd::Zero(J.cols(), J.cols());
      // TODO(xuchenhan-tri): Consider adding a more specialized routine for
      // computing JᵢᵀGJⱼ to further exploit sparsity. */
      J.TransposeAndMultiplyAndAddTo(GJ, &JTGJ);
      const int c = std::get<1>(jacobian_blocks_[triplets[0]]);
      H_->AddToBlock(c, c, std::move(JTGJ));
    } else {
      DRAKE_DEMAND(triplets.size() == 2);
      const int j = std::get<1>(jacobian_blocks_[triplets[0]]);
      const int i = std::get<1>(jacobian_blocks_[triplets[1]]);
      DRAKE_DEMAND(j < i);
      const MatrixBlock<double>& Jj =
          std::get<2>(jacobian_blocks_[triplets[0]]);
      const MatrixBlock<double>& Ji =
          std::get<2>(jacobian_blocks_[triplets[1]]);

      // TODO(xuchenhan-tri): Consider adding a more specialized routine for
      // computing JᵀGJ to further exploit sparsity. */
      const MatrixBlock<double> GJj = Jj.LeftMultiplyByBlockDiagonal(
          weight_matrix, weight_start, weight_end - 1);
      const MatrixBlock<double> GJi = Ji.LeftMultiplyByBlockDiagonal(
          weight_matrix, weight_start, weight_end - 1);

      MatrixXd JiTGJi = MatrixXd::Zero(Ji.cols(), Ji.cols());
      MatrixXd JiTGJj = MatrixXd::Zero(Ji.cols(), Jj.cols());
      MatrixXd JjTGJj = MatrixXd::Zero(Jj.cols(), Jj.cols());

      Ji.TransposeAndMultiplyAndAddTo(GJi, &JiTGJi);
      Ji.TransposeAndMultiplyAndAddTo(GJj, &JiTGJj);
      Jj.TransposeAndMultiplyAndAddTo(GJj, &JjTGJj);

      H_->AddToBlock(i, i, JiTGJi);
      H_->AddToBlock(i, j, JiTGJj);
      H_->AddToBlock(j, j, JjTGJj);
    }
    weight_start = weight_end;
  }
  solver_->UpdateMatrix(*H_);
  return true;
}

bool BlockSparseSuperNodalSolver::DoFactor() {
  return solver_->Factor();
}

void BlockSparseSuperNodalSolver::DoSolveInPlace(Eigen::VectorXd* b) const {
  solver_->SolveInPlace(b);
}

Eigen::MatrixXd BlockSparseSuperNodalSolver::DoMakeFullMatrix() const {
  return H_->MakeDenseMatrix();
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
