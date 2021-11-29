#include "drake/multibody/contact_solvers/supernodal_solver.h"

#include <algorithm>
#include <utility>

#include "conex/clique_ordering.h"
#include "conex/kkt_solver.h"

using Eigen::MatrixXd;
using std::vector;
using MatrixBlock = std::pair<Eigen::MatrixXd, std::vector<int>>;
using MatrixBlocks = std::vector<MatrixBlock>;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

// For each i in [s, e], computes y(i, :) = g(i) * x(i, :),
void LeftMultiplyByBlockDiagonal(const std::vector<MatrixXd>& g, int s, int e,
                                 const MatrixXd& x, MatrixXd* y) {
  int start = 0;
  for (int index = s; index <= e; ++index) {
    const int num_rows = g[s].rows();
    y->middleRows(start, num_rows).noalias() =
        g[index] * x.middleRows(start, num_rows);
    start += num_rows;
  }
}

template <typename MatrixType>
void Compute_Ji_transpose_Gi_Ji(const vector<MatrixXd>& jacobian_row_data,
                                const vector<MatrixXd>& weight_matrix,
                                int w_start, int w_end, MatrixType* yptr,
                                vector<MatrixXd>* temp) {
  MatrixType& y = *yptr;
  y.setZero();

  int r_offset = 0;
  for (size_t i = 0; i < jacobian_row_data.size(); ++i) {
    int c_offset = 0;
    const MatrixXd& r = jacobian_row_data[i];
    for (size_t j = 0; j < jacobian_row_data.size(); ++j) {
      const MatrixXd& c = jacobian_row_data[j];
      LeftMultiplyByBlockDiagonal(weight_matrix, w_start, w_end, c,
                                  &(*temp)[j]);
      y.block(r_offset, c_offset, r.cols(), c.cols()).noalias() +=
          r.transpose() * (*temp)[j];
      c_offset += c.cols();
    }
    r_offset += r.cols();
  }
}

// Returns a vector<vector<int>> y where y[i] contains the indices of the block
// Jacobians on row i organized by column.
vector<std::vector<int>> GetRowToTripletMapping(
    int num_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks) {
  vector<vector<int>> y(num_row_blocks);
  vector<vector<int>> column(num_row_blocks);

  // Sorts row data by column using our assumption that at most two columns
  // are non-zero per row.
  auto sort_row_data_by_column = [](vector<int>* col_index,
                                    vector<int>* triplet_position) {
    if ((*col_index)[0] > (*col_index)[1]) {
      std::swap((*col_index)[0], (*col_index)[1]);
      std::swap((*triplet_position)[0], (*triplet_position)[1]);
    }
  };

  int cnt = 0;
  for (const auto& j : jacobian_blocks) {
    int index = std::get<0>(j);
    y[index].emplace_back(cnt);
    column[index].emplace_back(std::get<1>(j));
    if (column[index].size() == 2) {
      sort_row_data_by_column(&column[index], &y[index]);
    }
    if (column[index].size() > 2) {
      throw std::runtime_error(
          "Jacobian can only be nonzero on at most two column blocks.");
    }
    ++cnt;
  }
  return y;
}

void VerifyMassMatrixPartitionRefinesJacobianPartition(
    const std::vector<int>& jacobian_column_block_size,
    const vector<MatrixXd>& mass_matrices) {
  int size = 0;
  size_t block_column = 0;
  for (size_t i = 0; i < mass_matrices.size(); i++) {
    // Check if too many mass matrices
    if (block_column >= jacobian_column_block_size.size()) {
      throw std::runtime_error(
          "The dimensions of the mass matrix and Jacobian are incompatible. "
          "The mass matrix has more scalar columns than the Jacobian.");
    }

    size += mass_matrices[i].cols();
    // Check if mass matrix overlaps two jacobian blocks.
    if (size > jacobian_column_block_size[block_column]) {
      throw std::runtime_error(
          "Column partition induced by mass matrix must refine the partition "
          "induced by the Jacobian.");
    }

    // Advance to next block if refinement of current block found.
    if (size == jacobian_column_block_size[block_column]) {
      ++block_column;
      size = 0;
    }
  }

  // Verify all jacobian blocks are refined.
  if (block_column < jacobian_column_block_size.size()) {
    throw std::runtime_error(
        "The dimensions of the mass matrix and Jacobian are incompatible. The "
        "Jacobian has more scalar columns than the mass matrix.");
  }
}

std::vector<int> GetJacobianBlockSizesVerifyTriplets(
    const std::vector<BlockMatrixTriplet>& jacobian_blocks) {
  int max_index = -1;
  for (const auto& j : jacobian_blocks) {
    int col_index = std::get<1>(j);
    if (col_index > max_index) {
      max_index = col_index;
    }
  }

  std::vector<int> block_column_size(max_index + 1, -1);

  for (const auto& j : jacobian_blocks) {
    int col_index = std::get<1>(j);

    if ((std::get<2>(j).cols() == 0) || (std::get<2>(j).rows() == 0)) {
      throw std::runtime_error(
          "Invalid Jacobian triplets: a triplet contains an empty matrix");
    }

    if (block_column_size[col_index] == -1) {
      block_column_size[col_index] = std::get<2>(j).cols();
    } else {
      if (block_column_size[col_index] != std::get<2>(j).cols()) {
        throw std::runtime_error(
            "Invalid Jacobian triplets: conflicting block sizes specified for "
            "column " +
            std::to_string(col_index) + ".");
      }
    }
  }

  for (int i = 0; i < max_index; i++) {
    // Verify that initial value of -1 has been overwritten. (A block column
    // size of zero will be caught by the empty matrix check.)
    if (block_column_size[i] < 0) {
      throw std::runtime_error(
          "Invalid Jacobian triplets: no triplet provided for column " +
          std::to_string(i) + ".");
    }
  }

  return block_column_size;
}

std::vector<int> GetMassMatrixStartingColumn(
    const std::vector<Eigen::MatrixXd>& mass_matrices) {
  vector<int> y;
  int col_start = 0;
  for (size_t i = 0; i < mass_matrices.size(); ++i) {
    int col_size = mass_matrices[i].cols();
    y.emplace_back(col_start);
    col_start += col_size;
  }
  return y;
}

}  // namespace

class SuperNodalSolver::CliqueAssembler final
    : public ::conex::SupernodalAssemblerBase {
 public:
  // Helper functions for specifying G_i
  void SetWeightMatrixIndex(int start, int end) {
    weight_start_ = start;
    weight_end_ = end;
  }
  void SetWeightMatrixPointer(
      const std::vector<Eigen::MatrixXd>* weight_matrix) {
    weight_matrix_ = weight_matrix;
  }

  // Updates a vector of mass matrices m_i satisfying
  // sub_matrix(M) = blkdiag(m_1, m_2, ..., m_n).
  void AssignMassMatrix(int i, const Eigen::MatrixXd& A) {
    mass_matrix_position_.push_back(i);
    mass_matrix_.push_back(A);
  }

  int NumRows() { return jacobian_row_data_[0].rows(); }

  // Copies in J_i and allocates memory for temporaries.
  void Initialize(std::vector<Eigen::MatrixXd>&& jacobian_row);

 private:
  void SetDenseData() override;

 private:
  std::vector<Eigen::MatrixXd> jacobian_row_data_;
  std::vector<int> mass_matrix_position_;
  std::vector<Eigen::MatrixXd> mass_matrix_;
  std::vector<Eigen::MatrixXd> G_times_J_;
  // TODO(FrankPermenter): remove this and rely on Conex to allocate
  // its own memory (requires a Conex update).
  Eigen::VectorXd workspace_memory_;
  const std::vector<Eigen::MatrixXd>* weight_matrix_ = nullptr;
  int weight_start_ = 0;
  int weight_end_ = 0;
};

void SuperNodalSolver::CliqueAssembler::SetDenseData() {
  if (!weight_matrix_) {
    throw std::runtime_error("Weight matrix not set.");
  }
  if (mass_matrix_position_.size() != mass_matrix_.size()) {
    throw std::runtime_error("Failed to add mass matrix.");
  }

  Compute_Ji_transpose_Gi_Ji(jacobian_row_data_, *weight_matrix_, weight_start_,
                             weight_end_, &submatrix_data_.G, &G_times_J_);
  int i = 0;
  for (const auto& pos : mass_matrix_position_) {
    submatrix_data_.G.block(pos, pos, mass_matrix_[i].rows(),
                            mass_matrix_[i].cols()) += mass_matrix_[i];
    ++i;
  }
}

std::pair<int, int> FindPositionInClique(int element,
                                         const vector<vector<int>>& clique) {
  int i = 0;
  for (const auto& ci : clique) {
    int j = 0;
    for (int cj : ci) {
      if (cj == element) {
        return std::pair<int, int>(i, j);
      }
      ++j;
    }
    ++i;
  }
  throw std::runtime_error("Failed to find mass matrix indices.");
}

vector<int> CumulativeSum(const vector<int>& x, int N) {
  vector<int> y(N + 1);
  y[0] = 0;
  std::partial_sum(x.begin(), x.begin() + N, y.begin() + 1);
  return y;
}

void SortTheCliques(std::vector<std::vector<int>>* path) {
  for (size_t i = 0; i < path->size(); ++i) {
    std::sort((*path)[i].begin(), (*path)[i].end());
  }
}

// The destructor is defined in the source so we can use an incomplete
// definition when storing unique pointers of CliqueAssembler within a
// std::vector.
SuperNodalSolver::~SuperNodalSolver() {}

// Helper struct for assembling the input to the conex supernodal solver. The
// variable "cliques" contains the nodes of a "supernodal elimination tree,"
// also called a "clique tree" or "junction tree". The variables "supernodes"
// and "separators" provide a partition of each node and satisfy
//
//   cliques[i] = Union(separators[i], supernodes[i]).
//
// The variable "order" and "supernodes" together define
// the elimination ordering:
//
//   elimination_order = { supernodes[order[0]], supernodes[order[1]],
//   ...., supernodes[order.back()] };
//
// The variable "order" can also be interpreted as a concatenation of paths in
// the supernodal elimination tree.
//
// See page 290 of Chordal Graphs and Semidefinite Optimization by Vandenberghe
// and Andersen.
struct SolverData {
  std::vector<std::vector<int>> cliques;
  int num_vars;
  std::vector<int> order;
  std::vector<std::vector<int>> supernodes;
  std::vector<std::vector<int>> separators;
};

struct SparsityData {
  SolverData data;
  // Expands the "block" cliques from SolverData into actual cliques of scalar
  // variables.
  std::vector<std::vector<int>> variable_cliques;
};

SparsityData GetEliminationOrdering(
    int num_jacobian_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks) {
  SparsityData clique_data;
  SolverData& data = clique_data.data;

  size_t n = num_jacobian_row_blocks;
  vector<int> order(n);
  vector<vector<int>> supernodes(n);
  vector<vector<int>> separators(n);
  vector<vector<int>> cliques(n);
  clique_data.variable_cliques.resize(n);

  vector<int> column_block_sizes(jacobian_blocks.size());

  int num_column_blocks = 0;
  for (const auto& b : jacobian_blocks) {
    int i = std::get<0>(b);
    int j = std::get<1>(b);
    column_block_sizes[j] = std::get<2>(b).cols();
    cliques[i].push_back(j);
    if (j >= num_column_blocks) {
      num_column_blocks = j + 1;
    }
  }
  SortTheCliques(&cliques);

  int largest_clique = 0;
  for (size_t i = 1; i < cliques.size(); ++i) {
    if (cliques[i].size() > cliques[largest_clique].size()) {
      largest_clique = i;
    }
  }

  // Computes new set of cliques and clique order such that the "Running
  // Intersection Property" holds. The new cliques are the input cliques with
  // added variables that correspond to fill-in in the Cholesky factorization.
  // The new cliques are stored implicitly as the union of separators and
  // supernodes:
  //
  //   new_clique = Union(supernodes[i], separators[i])
  //
  // They satisfy the running intersection property:
  //
  //   new_clique[order[i]] ∩ new_clique[order[j]]
  //            ⊇
  //   new_clique[order[i]] ∩ new_clique[order[k]]
  //
  // for all i < j < k.
  //
  // Reference: page 268 of Chordal Graphs and Semidefinite Optimization
  // by Vandenberghe and Andersen.
  conex::PickCliqueOrder(cliques, largest_clique, &order, &supernodes,
                         &separators);

  // The cliques correspond to block columns of the Jacobian. We now expand them
  // into scalar variables for use by the supernodal solver (which is unaware of
  // the block-structure).
  const vector<int> offsets =
      CumulativeSum(column_block_sizes, num_column_blocks);
  vector<vector<int>> supernodes_full(order.size());
  vector<vector<int>> separators_full(order.size());
  vector<vector<int>> cliques_full_with_fill_in(order.size());
  for (size_t i = 0; i < order.size(); ++i) {
    for (int s : supernodes[i]) {
      for (int cnt = 0; cnt < column_block_sizes[s]; ++cnt) {
        supernodes_full[i].push_back(offsets[s] + cnt);
        cliques_full_with_fill_in[i].push_back(offsets[s] + cnt);
      }
    }
    for (int s : separators[i]) {
      for (int cnt = 0; cnt < column_block_sizes[s]; ++cnt) {
        separators_full[i].push_back(offsets[s] + cnt);
        cliques_full_with_fill_in[i].push_back(offsets[s] + cnt);
      }
    }
    // Expand the original cliques (without fill-in) into scalar variables.
    for (int s : cliques[i]) {
      for (int cnt = 0; cnt < column_block_sizes[s]; ++cnt) {
        clique_data.variable_cliques[i].push_back(offsets[s] + cnt);
      }
    }
  }

  data.cliques = cliques_full_with_fill_in;
  SortTheCliques(&data.cliques);

  data.num_vars = offsets[num_column_blocks];
  data.supernodes = supernodes_full;
  data.order = order;
  data.separators = separators_full;
  return clique_data;
}

void SuperNodalSolver::Initialize(
    const vector<vector<int>>& cliques, int num_jacobian_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks,
    const std::vector<Eigen::MatrixXd>& mass_matrices) {
  std::vector<int> jacobian_column_block_size =
      GetJacobianBlockSizesVerifyTriplets(jacobian_blocks);
  // Will throw an exception if verification fails.
  VerifyMassMatrixPartitionRefinesJacobianPartition(jacobian_column_block_size,
                                                    mass_matrices);

  clique_assemblers_ptrs_.resize(cliques.size());

  vector<vector<int>> row_to_triplet_list =
      GetRowToTripletMapping(num_jacobian_row_blocks, jacobian_blocks);
  for (size_t i = 0; i < cliques.size(); ++i) {
    owned_clique_assemblers_[i] = std::make_unique<CliqueAssembler>();
    std::vector<MatrixXd> jacobian_blocks_of_row;
    jacobian_blocks_of_row.reserve(row_to_triplet_list[i].size());
    for (const auto& j : row_to_triplet_list[i]) {
      jacobian_blocks_of_row.push_back(std::get<2>(jacobian_blocks[j]));
    }
    owned_clique_assemblers_[i]->Initialize(std::move(jacobian_blocks_of_row));
    clique_assemblers_ptrs_[i] = owned_clique_assemblers_[i].get();
  }

  const std::vector<int> mass_matrix_starting_columns =
      GetMassMatrixStartingColumn(mass_matrices);
  int cnt = 0;
  for (const auto& c : mass_matrix_starting_columns) {
    const std::pair<int, int> position = FindPositionInClique(c, cliques);
    owned_clique_assemblers_[position.first]->AssignMassMatrix(
        position.second, mass_matrices[cnt]);
    ++cnt;
  }

  // Make connections between clique_assemblers and solver->Assemble().
  solver_->Bind(clique_assemblers_ptrs_);
}

SuperNodalSolver::SuperNodalSolver(
    int num_jacobian_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks,
    const std::vector<Eigen::MatrixXd>& mass_matrices)
    : owned_clique_assemblers_(num_jacobian_row_blocks) {
  SparsityData clique_data =
      GetEliminationOrdering(num_jacobian_row_blocks, jacobian_blocks);

  solver_ = std::make_unique<::conex::SupernodalKKTSolver>(
      clique_data.variable_cliques, clique_data.data.num_vars,
      clique_data.data.order, clique_data.data.supernodes,
      clique_data.data.separators);

  Initialize(clique_data.variable_cliques, num_jacobian_row_blocks,
             jacobian_blocks, mass_matrices);
}

void SuperNodalSolver::SetWeightMatrix(
    const std::vector<Eigen::MatrixXd>& weight_matrix) {
  // We copy these pointers so that SetDenseData (a virtual function override)
  // can access the weight matrices when solver_->Assemble() is called
  // below. For safety, we replace these pointers with nullptr when
  // solver_->Assemble() returns.
  for (auto& c : owned_clique_assemblers_) {
    c->SetWeightMatrixPointer(&weight_matrix);
  }

  int e_last = -1;
  bool weight_matrix_incompatible = false;
  for (auto& c : owned_clique_assemblers_) {
    int num_rows = c->NumRows();
    int s = e_last + 1;
    int e = s;
    int num_rows_found = weight_matrix[e].rows();
    while (num_rows_found < num_rows) {
      ++e;
      num_rows_found += weight_matrix[e].rows();
    }
    if (num_rows_found != num_rows) {
      weight_matrix_incompatible = true;
    }
    e_last = e;
    c->SetWeightMatrixIndex(s, e);
  }

  if (!weight_matrix_incompatible) {
    solver_->Assemble();
  }

  // Destroy references to argument weight_matrix.
  for (auto& c : owned_clique_assemblers_) {
    c->SetWeightMatrixPointer(nullptr);
  }

  if (weight_matrix_incompatible) {
    throw std::runtime_error("Weight matrix incompatible with Jacobian.");
  }

  factorization_ready_ = false;
  matrix_ready_ = true;
}

bool SuperNodalSolver::Factor() {
  if (!matrix_ready_) {
    throw std::runtime_error("Call to Factor() failed: weight matrix not set.");
  }
  bool success = solver_->Factor();
  factorization_ready_ = success;
  matrix_ready_ = false;
  return success;
}

Eigen::VectorXd SuperNodalSolver::Solve(const Eigen::VectorXd& b) const {
  if (!factorization_ready_) {
    throw std::runtime_error(
        "Call to Solve() failed: factorization not ready.");
  }
  Eigen::VectorXd y = b;
  // The supernodal solver uses a mapped MatrixXd as input, so we create this
  // map.
  Eigen::Map<MatrixXd, Eigen::Aligned> ymap(y.data(), b.rows(), 1);
  solver_->SolveInPlace(&ymap);
  return y;
}

void SuperNodalSolver::SolveInPlace(Eigen::VectorXd* b) const {
  if (!factorization_ready_) {
    throw std::runtime_error(
        "Call to Solve() failed: factorization not ready.");
  }
  Eigen::Map<MatrixXd, Eigen::Aligned> ymap(b->data(), b->rows(), 1);
  solver_->SolveInPlace(&ymap);
}

Eigen::MatrixXd SuperNodalSolver::MakeFullMatrix() const {
  if (!matrix_ready_) {
    throw std::runtime_error(
        "Call to MakeFullMatrix() failed: weight matrix not set or matrix has "
        "been factored in place.");
  }
  return solver_->KKTMatrix();
}

void SuperNodalSolver::CliqueAssembler::Initialize(
    std::vector<Eigen::MatrixXd>&& r) {
  G_times_J_.resize(r.size());
  int num_vars = 0;
  for (size_t j = 0; j < r.size(); ++j) {
    num_vars += r[j].cols();
    G_times_J_[j].resize(r[j].rows(), r[j].cols());
  }
  jacobian_row_data_ = std::move(r);

  SupernodalAssemblerBase::SetNumberOfVariables(num_vars);
  const int size = SizeOf(SupernodalAssemblerBase::submatrix_data_);
  workspace_memory_.resize(size);
  SupernodalAssemblerBase::submatrix_data_.InitializeWorkspace(
      workspace_memory_.data());
}
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
