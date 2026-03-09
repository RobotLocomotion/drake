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
    const std::vector<MatrixX<double>>& A, const BlockSparseMatrix<double>& J)
    : BlockSparseSuperNodalSolver(J.block_rows(), J.get_blocks(), A) {}

BlockSparseSuperNodalSolver::BlockSparseSuperNodalSolver(
    int num_jacobian_row_blocks, std::vector<BlockTriplet> jacobian_blocks,
    std::vector<Eigen::MatrixXd> mass_matrices)
    : jacobian_blocks_(std::move(jacobian_blocks)),
      mass_matrices_(std::move(mass_matrices)) {
  const std::vector<int> jacobian_column_block_size =
      GetJacobianBlockSizesVerifyTriplets(jacobian_blocks_);
  /* Throw an exception if verification fails. */
  if (!MassMatrixPartitionEqualsJacobianPartition(jacobian_column_block_size,
                                                  mass_matrices_)) {
    throw std::runtime_error(
        "Mass matrices and constraint Jacobians are incompatible.");
  }
  row_to_triplet_index_ =
      GetRowToTripletMapping(num_jacobian_row_blocks, jacobian_blocks_);

  const int num_nodes = mass_matrices_.size();
  std::vector<int> block_sizes(num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    block_sizes[i] = mass_matrices_[i].rows();
  }
  /* Build diagonal entry in sparsity pattern. */
  std::vector<std::vector<int>> sparsity(num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    sparsity[i].emplace_back(i);
  }
  /* Build off-diagonal entry in sparsity pattern. */
  const int num_constraints = row_to_triplet_index_.size();
  for (int r = 0; r < num_constraints; ++r) {
    const std::vector<int>& triplet_indices = row_to_triplet_index_[r];
    DRAKE_DEMAND(triplet_indices.size() <= 2);
    if (triplet_indices.size() == 2) {
      const int j = jacobian_blocks_[triplet_indices[0]].col;
      const int i = jacobian_blocks_[triplet_indices[1]].col;
      DRAKE_DEMAND(j < i);
      sparsity[j].emplace_back(i);
    }
  }
  H_ = std::make_unique<BlockSparseSymmetricMatrixXd>(
      BlockSparsityPattern(std::move(block_sizes), std::move(sparsity)));
  /* The solver analyzes the sparsity pattern of the H_ (currently a zero
   matrix) so that subsequent updates to the matrix can use UpdateMatrix()
   that doesn't perform symbolic factorization and allocation. */
  solver_.SetMatrix(*H_);
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
  // TODO(xuchenhan-tri): Getting the starting indices of G blocks as well as
  // checking partitions of G refines partitions of block rows of J should
  // happen in the base class.
  /* Recall that the partition of the weight matrix G is a refinement on the
   partition of the block rows of J. Here we use `weight_start` and
   `weight_end` to track the indices into `weight_matrix` that corresponds to
   the k-th block row of J. */
  int weight_start = 0;
  int weight_end = 0;
  for (int k = 0; k < num_constraints; ++k) {
    const std::vector<int>& triplet_indices = row_to_triplet_index_[k];
    const int num_constraint_equations =
        jacobian_blocks_[triplet_indices[0]].value.rows();
    int G_rows = 0;
    while (G_rows < num_constraint_equations &&
           weight_end < ssize(weight_matrix)) {
      G_rows += weight_matrix[weight_end++].rows();
    }
    if (G_rows != num_constraint_equations) {
      return false;
    }

    if (triplet_indices.size() == 1) {
      const MatrixBlock<double>& J = jacobian_blocks_[triplet_indices[0]].value;
      const MatrixBlock<double> GJ = J.LeftMultiplyByBlockDiagonal(
          weight_matrix, weight_start, weight_end - 1);
      MatrixXd JTGJ = MatrixXd::Zero(J.cols(), J.cols());
      // TODO(xuchenhan-tri): Consider adding a more specialized routine for
      // computing JᵢᵀGJⱼ to further exploit sparsity. */
      J.TransposeAndMultiplyAndAddTo(GJ, &JTGJ);
      const int c = jacobian_blocks_[triplet_indices[0]].col;
      H_->AddToBlock(c, c, std::move(JTGJ));
    } else {
      DRAKE_DEMAND(triplet_indices.size() == 2);
      const int j = jacobian_blocks_[triplet_indices[0]].col;
      const int i = jacobian_blocks_[triplet_indices[1]].col;
      DRAKE_DEMAND(j < i);
      const MatrixBlock<double>& Jj =
          jacobian_blocks_[triplet_indices[0]].value;
      const MatrixBlock<double>& Ji =
          jacobian_blocks_[triplet_indices[1]].value;

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
  solver_.UpdateMatrix(*H_);
  return true;
}

bool BlockSparseSuperNodalSolver::DoFactor() {
  return solver_.Factor();
}

void BlockSparseSuperNodalSolver::DoSolveInPlace(Eigen::VectorXd* b) const {
  solver_.SolveInPlace(b);
}

Eigen::MatrixXd BlockSparseSuperNodalSolver::DoMakeFullMatrix() const {
  return H_->MakeDenseMatrix();
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
