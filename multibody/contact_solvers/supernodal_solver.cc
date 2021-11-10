#include "drake/multibody/contact_solvers/supernodal_solver.h"

#include <algorithm>
#include <utility>

#include "conex/clique_ordering.h"
#include "conex/kkt_solver.h"
#include "conex/supernodal_solver.h"

using Eigen::MatrixXd;
using std::vector;
using MatrixBlock = std::pair<Eigen::MatrixXd, std::vector<int>>;
using MatrixBlocks = std::vector<MatrixBlock>;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

// For each i \in [s, e], computes y(i, :) = g(i) * x(i, :),
void LeftMultiplyByBlockDiagonal(const std::vector<MatrixXd>& w, int s, int e,
                                 const MatrixXd& x, MatrixXd* y) {
  int start = 0;
  for (int index = s; index <= e; ++index) {
    const int num_rows = w.at(s).rows();
    y->middleRows(start, num_rows).noalias() =
        w.at(index) * x.middleRows(start, num_rows);
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
    const MatrixXd& r = jacobian_row_data.at(i);
    for (size_t j = 0; j < jacobian_row_data.size(); ++j) {
      const MatrixXd& c = jacobian_row_data.at(j);
      LeftMultiplyByBlockDiagonal(weight_matrix, w_start, w_end, c,
                                  &temp->at(j));
      y.block(r_offset, c_offset, r.cols(), c.cols()).noalias() +=
          r.transpose() * temp->at(j);
      c_offset += c.cols();
    }
    r_offset += r.cols();
  }
}

// Returns a vector<vector<int>> y where
// y.at(i) contains the indices of the block Jacobians
// on row i organized by column.
vector<std::vector<int>> GetRowToTripletMapping(
    int num_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks) {
  vector<vector<int>> y(num_row_blocks);
  vector<vector<int>> column(num_row_blocks);

  // Sorts row data by column using our assumption that at most two columns
  // are non-zero per row.
  auto sort_row_data_by_column = [](vector<int>* col_index,
                                    vector<int>* triplet_position) {
    if (col_index->at(0) > col_index->at(1)) {
      std::swap(col_index->at(0), col_index->at(1));
      std::swap(triplet_position->at(0), triplet_position->at(1));
    }
  };

  int cnt = 0;
  for (const auto& j : jacobian_blocks) {
    int index = std::get<0>(j);
    y.at(index).emplace_back(cnt);
    column.at(index).emplace_back(std::get<1>(j));
    if (column.at(index).size() == 2) {
      sort_row_data_by_column(&column.at(index), &y.at(index));
    }
    if (column.at(index).size() > 2) {
      throw std::runtime_error(
          "Jacobian can only be nonzero on at most two column blocks.");
    }
    ++cnt;
  }
  return y;
}

std::vector<int> GetMassMatrixStartingColumn(
    const std::vector<Eigen::MatrixXd>& mass_matrices) {
  vector<int> y;
  int col_start = 0;
  for (size_t i = 0; i < mass_matrices.size(); ++i) {
    int col_size = mass_matrices.at(i).cols();
    y.emplace_back(col_start);
    col_start += col_size;
  }
  return y;
}

}  // namespace

class SuperNodalSolver::CliqueAssembler final
    : public ::conex::LinearKKTAssemblerBase {
 public:
  // Fills the matrix sub_matrix(M) + J^T_i G_i J_i.

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

  int NumRows() { return jacobian_row_data_.at(0).rows(); }

  // Copies in J_i and allocates memory for temporaries.
  void Initialize(const std::vector<Eigen::MatrixXd>&& jacobian_row);

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
  const std::vector<Eigen::MatrixXd>* weight_matrix_;
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
                             weight_end_, &schur_complement_data.G,
                             &G_times_J_);
  int i = 0;
  for (const auto& pos : mass_matrix_position_) {
    schur_complement_data.G.block(pos, pos, mass_matrix_.at(i).rows(),
                                  mass_matrix_.at(i).cols()) +=
        mass_matrix_.at(i);
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
  y.at(0) = 0;
  std::partial_sum(x.begin(), x.begin() + N, y.begin() + 1);
  return y;
}

void SortTheCliques(std::vector<std::vector<int>>* path) {
  for (size_t i = 0; i < path->size(); ++i) {
    std::sort(path->at(i).begin(), path->at(i).end());
  }
}

// The constructor is defined in the source so we can use an incomplete
// definition when storing unique pointers of CliqueAssembler within a
// std::vector.
SuperNodalSolver::~SuperNodalSolver() {}

// Helper struct for assembling the input to the
// conex supernodal solver. The variable "cliques"
// contains the nodes of a "supernodal elimination tree,"
// also called a "clique tree" or "junction tree". The variables "supernodes"
// and "separators" provide a partition of each node and satisfy
// cliques.at(i) = Union(separators.at(i), supernodes.at(i)).
// The variable "order" and "supernodes" together define
// the elimination ordering:
//
//  elimination_order = [ supernodes.at(order(0)), supernodes.at(order(1)),
//  ...., supernodes.at(order.back()) ].
//
// The variable "order" can also be interpreted as a concatenation
// of paths in the supernodal elimination tree.
//
// See page 290 of Chordal Graphs and Semidefinite Optimization
// by Vandenberghe and Andersen.
struct SolverData {
  std::vector<std::vector<int>> cliques;
  int num_vars;
  std::vector<int> order;
  std::vector<std::vector<int>> supernodes;
  std::vector<std::vector<int>> separators;
};

struct SparsityData {
  SolverData data;
  // Expands the "block" cliques from SolverData into
  // actual cliques of scalar variables.
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
    column_block_sizes.at(j) = std::get<2>(b).cols();
    cliques.at(i).push_back(j);
    if (j >= num_column_blocks) {
      num_column_blocks = j + 1;
    }
  }
  SortTheCliques(&cliques);

  int largest_clique = 0;
  for (size_t i = 1; i < cliques.size(); ++i) {
    if (cliques.at(i).size() > cliques.at(largest_clique).size()) {
      largest_clique = i;
    }
  }

  // Computes new set of cliques and clique order such that the "Running
  // Intersection Property" holds. The new cliques are the input cliques with
  // added variables that correspond to fill-in in the Cholesky factorization.
  // The new cliques are stored stored implicitly as the union
  // of separators and supernodes:
  //
  //   new_clique = Union(supernodes.at(i), separators.at(i))
  //
  // They satisfy the running intersection property:
  //
  //   new_clique.at(order.at(i)) \cap  new_clique.at(order.at(j))
  //            \supseteq
  //   new_clique.at(order.at(i)) \cap  new_clique.at(order.at(k))
  //
  // for all i < j < k.
  //
  // Reference: page 268 of Chordal Graphs and Semidefinite Optimization
  // by Vandenberghe and Andersen.
  conex::PickCliqueOrder(cliques, largest_clique, &order, &supernodes,
                         &separators);

  // The cliques correspond to block columns of the Jacobian.
  // We now expand them into scalar variables for use by the
  // supernodal solver (which is unaware of the block-structure).
  const vector<int> offsets =
      CumulativeSum(column_block_sizes, num_column_blocks);
  vector<vector<int>> supernodes_full(order.size());
  vector<vector<int>> separators_full(order.size());
  vector<vector<int>> cliques_full_with_fill_in(order.size());
  for (size_t i = 0; i < order.size(); ++i) {
    for (int s : supernodes.at(i)) {
      for (int cnt = 0; cnt < column_block_sizes.at(s); ++cnt) {
        supernodes_full.at(i).push_back(offsets.at(s) + cnt);
        cliques_full_with_fill_in.at(i).push_back(offsets.at(s) + cnt);
      }
    }
    for (int s : separators.at(i)) {
      for (int cnt = 0; cnt < column_block_sizes.at(s); ++cnt) {
        separators_full.at(i).push_back(offsets.at(s) + cnt);
        cliques_full_with_fill_in.at(i).push_back(offsets.at(s) + cnt);
      }
    }
    // Expand the original cliques (without fill-in) into
    // scalar variables.
    for (int s : cliques.at(i)) {
      for (int cnt = 0; cnt < column_block_sizes.at(s); ++cnt) {
        clique_data.variable_cliques.at(i).push_back(offsets.at(s) + cnt);
      }
    }
  }

  data.cliques = cliques_full_with_fill_in;
  SortTheCliques(&data.cliques);

  data.num_vars = offsets.at(num_column_blocks);
  data.supernodes = supernodes_full;
  data.order = order;
  data.separators = separators_full;
  return clique_data;
}

void SuperNodalSolver::Initialize(
    const vector<vector<int>>& cliques, int num_jacobian_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks,
    const std::vector<Eigen::MatrixXd>& mass_matrices) {
  clique_assemblers_ptrs_.resize(cliques.size());

  vector<vector<int>> row_to_triplet_list =
      GetRowToTripletMapping(num_jacobian_row_blocks, jacobian_blocks);
  for (size_t i = 0; i < cliques.size(); ++i) {
    owned_clique_assemblers_.at(i) = std::make_unique<CliqueAssembler>();
    std::vector<MatrixXd> jacobian_blocks_of_row;
    jacobian_blocks_of_row.reserve(row_to_triplet_list.at(i).size());
    for (const auto& j : row_to_triplet_list.at(i)) {
      jacobian_blocks_of_row.push_back(std::get<2>(jacobian_blocks.at(j)));
    }
    owned_clique_assemblers_.at(i)->Initialize(
        std::move(jacobian_blocks_of_row));
    clique_assemblers_ptrs_.at(i) = owned_clique_assemblers_.at(i).get();
  }

  const std::vector<int> mass_matrix_starting_columns =
      GetMassMatrixStartingColumn(mass_matrices);
  int cnt = 0;
  for (const auto& c : mass_matrix_starting_columns) {
    const std::pair<int, int> position = FindPositionInClique(c, cliques);
    owned_clique_assemblers_.at(position.first)
        ->AssignMassMatrix(position.second, mass_matrices.at(cnt));
    ++cnt;
  }

  solver_->Bind(clique_assemblers_ptrs_);
}

SuperNodalSolver::SuperNodalSolver(
    int num_jacobian_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks,
    const std::vector<Eigen::MatrixXd>& mass_matrices)
    : owned_clique_assemblers_(num_jacobian_row_blocks) {
  SparsityData clique_data_ =
      GetEliminationOrdering(num_jacobian_row_blocks, jacobian_blocks);

  solver_ = std::make_unique<::conex::SupernodalKKTSolver>(
      clique_data_.variable_cliques, clique_data_.data.num_vars,
      clique_data_.data.order, clique_data_.data.supernodes,
      clique_data_.data.separators);

  Initialize(clique_data_.variable_cliques, num_jacobian_row_blocks,
             jacobian_blocks, mass_matrices);
}

void SuperNodalSolver::SetWeightMatrix(
    const std::vector<Eigen::MatrixXd>& weight_matrix) {
  // We copy these pointers so that SetDenseData (a virtual function override)
  // can access the weight matrices when solver_->AssembleFromCliques is called
  // below.  For safety, we replace these pointers with NULL when
  // solver_->AssembleFromCliques returns.
  for (auto& c : owned_clique_assemblers_) {
    c->SetWeightMatrixPointer(&weight_matrix);
  }

  int e_last = -1;
  bool weight_matrix_incompatible = false;
  for (auto& c : owned_clique_assemblers_) {
    int num_rows = c->NumRows();
    int s = e_last + 1;
    int e = s;
    int num_rows_found = weight_matrix.at(e).rows();
    while (num_rows_found < num_rows) {
      ++e;
      num_rows_found += weight_matrix.at(e).rows();
    }
    if (num_rows_found != num_rows) {
      weight_matrix_incompatible = true;
    }
    e_last = e;
    c->SetWeightMatrixIndex(s, e);
  }

  if (!weight_matrix_incompatible) {
    solver_->AssembleFromCliques(clique_assemblers_ptrs_);
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
    const std::vector<Eigen::MatrixXd>&& r) {
  jacobian_row_data_ = std::move(r);
  G_times_J_.resize(r.size());
  int num_vars = 0;
  for (size_t j = 0; j < jacobian_row_data_.size(); ++j) {
    num_vars += r.at(j).cols();
    G_times_J_.at(j).resize(r.at(j).rows(), r.at(j).cols());
  }

  LinearKKTAssemblerBase::SetNumberOfVariables(num_vars);
  const int size = SizeOf(LinearKKTAssemblerBase::schur_complement_data);
  workspace_memory_.resize(size);
  LinearKKTAssemblerBase::schur_complement_data.InitializeWorkspace(
      workspace_memory_.data());
}
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
