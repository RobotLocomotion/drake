#include "drake/multibody/contact_solvers/supernodal_solver.h"

#include <algorithm>
#include <string>
#include <utility>

#include "conex/clique_ordering.h"
#include "conex/supernodal_solver.h"

using Eigen::MatrixXd;
using std::vector;
using MatrixBlock = std::pair<Eigen::MatrixXd, std::vector<int>>;
using MatrixBlocks = std::vector<MatrixBlock>;
using MatrixBlockData = std::pair<Eigen::MatrixXd, int>;
using JacobianRowData = std::vector<MatrixBlockData>;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

namespace {

inline int NumberOfVars(const MatrixBlocks& row_data) {
  int y = 0;
  for (auto& r : row_data) {
    y += r.second.size();
  }
  return y;
}

void MultiplyByBlockDiagonal(const std::vector<MatrixXd>& w, int s, int e,
                             const MatrixXd& x, MatrixXd* y) {
  int start = 0;
  for (int index = s; index <= e; index++) {
    const int num_rows = w.at(s).rows();
    y->middleRows(start, num_rows).noalias() =
        w.at(index) * x.middleRows(start, num_rows);
    start += num_rows;
  }
}

template <typename T>
void SumOverWeightedMatrixBlocks(const vector<MatrixXd>& row_data,
                                 const vector<MatrixXd>& weight_matrix,
                                 int w_start, int w_end, T* yptr,
                                 vector<MatrixXd>* temp) {
  auto& y = *yptr;
  y.setZero();

  int r_offset = 0;
  for (size_t i = 0; i < row_data.size(); i++) {
    int c_offset = 0;
    const auto& r = row_data.at(i);
    for (size_t j = 0; j < row_data.size(); j++) {
      const auto& c = row_data.at(j);
      MultiplyByBlockDiagonal(weight_matrix, w_start, w_end, c, &temp->at(j));
      y.block(r_offset, c_offset, r.cols(), c.cols()).noalias() +=
          r.transpose() * temp->at(j);
      c_offset += c.cols();
    }
    r_offset += r.cols();
  }
}

class Container {
 public:
  template <typename T>
  explicit Container(const T& x) : obj(x), kkt(std::any_cast<T>(&obj)) {}
  std::any obj;
  ::conex::KKT_SystemAssembler kkt;
};


// TODO(FrankPermenter): Remove this.
inline std::vector<std::vector<MatrixXd>> GetRowData(
    const std::vector<MatrixBlocks>& jacobian_blocks) {
  vector<std::vector<MatrixXd>> y(jacobian_blocks.size());
  int cnt = 0;
  for (auto& j : jacobian_blocks) {
    for (auto& ji : j) {
      y.at(cnt).emplace_back(ji.first);
    }
    cnt++;
  }
  return y;
}

inline std::vector<int> ColumnToClique(int start, int clique_size) {
  int start_position = start;
  std::vector<int> y(clique_size);
  for (int i = 0; i < clique_size; i++) {
    y[i] = start_position + i;
  }
  return y;
}

void SortClique(std::vector<int>* c, std::vector<Eigen::MatrixXd>* m) {
  if (c->at(0) > c->at(1)) {
    auto temp = c->at(1);
    c->at(1) = c->at(0);
    c->at(0) = temp;

    auto temp2 = m->at(1);
    m->at(1) = m->at(0);
    m->at(0) = temp2;
  }
}

inline std::vector<std::vector<MatrixXd>> GetRowData(
    int num_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks) {
  vector<std::vector<MatrixXd>> y(num_row_blocks);
  vector<vector<int>> column(num_row_blocks);

  for (auto& j : jacobian_blocks) {
    int index = std::get<0>(j);
    y.at(index).emplace_back(std::get<2>(j));
    column.at(index).emplace_back(std::get<1>(j));
    if (column.at(index).size() == 2) {
      SortClique(&column.at(index), &y.at(index));
    }
    if (column.at(index).size() > 2) {
      throw std::runtime_error(
          "Jacobian can only be nonzero on at most two column blocks.");
    }
  }
  return y;
}



inline MatrixBlocks GetMassMatrix(
    const std::vector<Eigen::MatrixXd>& mass_matrices) {
  MatrixBlocks y;
  int clique_start = 0;
  for (size_t i = 0; i < mass_matrices.size(); i++) {
    int clique_size = mass_matrices.at(i).cols();
    y.emplace_back(mass_matrices.at(i),
                   ColumnToClique(clique_start, clique_size));
    clique_start += clique_size;
  }
  return y;
}

inline Eigen::MatrixXd BlockDiag(int s, int e,
                                 std::vector<Eigen::MatrixXd> Gi) {
  Eigen::MatrixXd G(100, 100);
  G.setZero();
  int row_start = 0;
  for (int i = s; i < e; i++) {
    G.block(row_start, row_start, Gi.at(i).rows(), Gi.at(i).cols()) = Gi.at(i);
    row_start += s - e;
  }
  return G.topLeftCorner(row_start, row_start);
}

inline std::vector<Eigen::MatrixXd> GetWeightMatrix(
    const std::vector<int>& patch_size, const std::vector<Eigen::MatrixXd>& G) {
  std::vector<Eigen::MatrixXd> y(patch_size.size());
  int s = 0;
  int e = 0;
  for (size_t i = 0; i < patch_size.size(); i++) {
    e = s + patch_size.at(i);
    y.at(i) = BlockDiag(s, e, G);
    s = e + 1;
  }
  return y;
}

}  // namespace

void SuperNodalSolver::CliqueAssembler::SetDenseData() {
#ifndef NDEBUG
  if (!weight_matrix_) {
    throw std::runtime_error("Weight matrix not set.");
  }
  if (mass_matrix_position_.size() != mass_matrix_.size()) {
    throw std::runtime_error("Failed to add mass matrix.");
  }
#endif

  SumOverWeightedMatrixBlocks(row_data_, *weight_matrix_, weight_start_,
                              weight_end_, &schur_complement_data.G,
                              &temporaries_);
  int i = 0;
  for (auto& pos : mass_matrix_position_) {
    schur_complement_data.G.block(pos, pos, mass_matrix_.at(i).rows(),
                                  mass_matrix_.at(i).cols()) +=
        mass_matrix_.at(i);
    i++;
  }
}

std::pair<int, int> FindPositionInClique(int element,
                                         const vector<vector<int>>& clique) {
  int i = 0;
  for (auto ci : clique) {
    int j = 0;
    for (auto cj : ci) {
      if (cj == element) {
        return std::pair<int, int>(i, j);
      }
      j++;
    }
    i++;
  }
  throw std::runtime_error("Failed to find mass matrix indices.");
}

vector<int> CumulativeSum(const vector<int>& x, int N) {
  vector<int> y(N + 1);
  y.at(0) = 0;
  for (int i = 1; i < N + 1; i++) {
    y.at(i) = y.at(i - 1) + x.at(i - 1);
  }
  return y;
}

void SortTheCliques(std::vector<std::vector<int>>* path) {
  for (size_t i = 0; i < path->size(); i++) {
    std::sort(path->at(i).begin(), path->at(i).end());
  }
}

SuperNodalSolver::SparsityData SuperNodalSolver::GetEliminationOrdering(
    int num_jacobian_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks) {
  using std::vector;
  SparsityData clique_data;
  auto& data = clique_data.data;

  size_t n = num_jacobian_row_blocks;
  vector<int> order(n);
  vector<vector<int>> supernodes(n);
  vector<vector<int>> separators(n);
  vector<vector<int>> cliques(n);
  clique_data.cliques_assembler.resize(n);

  vector<int> column_block_sizes(jacobian_blocks.size());
  ::conex::RootedTree tree(n);

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
  for (size_t i = 1; i < cliques.size(); i++) {
    if (cliques.at(i).size() > cliques.at(largest_clique).size()) {
      largest_clique = i;
    }
  }

  conex::PickCliqueOrder(cliques, largest_clique, &order, &supernodes, &separators);

  auto offsets = CumulativeSum(column_block_sizes, num_column_blocks);
  vector<vector<int>> supernodes_full(order.size());
  vector<vector<int>> separators_full(order.size());
  vector<vector<int>> cliques_full(order.size());
  for (size_t i = 0; i < order.size(); i++) {
    for (auto s : supernodes.at(i)) {
      for (int cnt = 0; cnt < column_block_sizes.at(s); cnt++) {
        supernodes_full.at(i).push_back(offsets.at(s) + cnt);
        cliques_full.at(i).push_back(offsets.at(s) + cnt);
      }
    }
    for (auto s : separators.at(i)) {
      for (int cnt = 0; cnt < column_block_sizes.at(s); cnt++) {
        separators_full.at(i).push_back(offsets.at(s) + cnt);
        cliques_full.at(i).push_back(offsets.at(s) + cnt);
      }
    }
    for (auto s : cliques.at(i)) {
      for (int cnt = 0; cnt < column_block_sizes.at(s); cnt++) {
        clique_data.cliques_assembler.at(i).push_back(offsets.at(s) + cnt);
      }
    }
  }

  data.cliques = cliques_full;
  SortTheCliques(&data.cliques);

  data.num_vars = offsets.at(num_column_blocks);
  data.supernodes = supernodes_full;
  data.order = order;
  data.separators = separators_full;
  return clique_data;
}

void SuperNodalSolver::Initialize(
    const vector<vector<int>>& cliques,
    const vector<vector<Eigen::MatrixXd>>& row_data) {
  int i = 0;
  clique_assemblers_.resize(row_data.size());
  for (auto& c : row_data) {
    jacobian_assemblers_.at(i).SetMatrixBlocks(c);
    clique_assemblers_.at(i) = &jacobian_assemblers_.at(i);
    i++;
  }

  for (auto& c : mass_matrices_) {
    auto position = FindPositionInClique(c.second.at(0), cliques);
    jacobian_assemblers_.at(position.first)
        .AssignMassMatrix(position.second, c.first);
  }

  solver_.Bind(clique_assemblers_);
}

std::vector<JacobianRowData> ConvertRow(
    const std::vector<MatrixBlocks>& row_data) {
  std::vector<JacobianRowData> y(row_data.size());
  int i = 0;
  for (auto& ri : row_data) {
    for (auto& rii : ri) {
      y.at(i).emplace_back(rii.first, rii.second.at(0));
    }
    i++;
  }
  return y;
}

SuperNodalSolver::SuperNodalSolver(
    int num_jacobian_row_blocks,
    const std::vector<BlockMatrixTriplet>& jacobian_blocks,
    const std::vector<Eigen::MatrixXd>& mass_matrices)
    : mass_matrices_(GetMassMatrix(mass_matrices)),
      clique_data_(
          GetEliminationOrdering(num_jacobian_row_blocks, jacobian_blocks)),
      solver_(clique_data_.cliques_assembler, clique_data_.data.num_vars,
              clique_data_.data.order, clique_data_.data.supernodes,
              clique_data_.data.separators),
      jacobian_assemblers_(num_jacobian_row_blocks) {
  Initialize(clique_data_.cliques_assembler,
             GetRowData(num_jacobian_row_blocks, jacobian_blocks));
}

void SuperNodalSolver::SetWeightMatrix(
    const std::vector<Eigen::MatrixXd>& weight_matrix) {
  for (auto& c : jacobian_assemblers_) {
    c.SetWeightMatrixPointer(&weight_matrix);
  }

  if (!weight_matrix_ready_) {
    int e_last = -1;
    for (auto& c : jacobian_assemblers_) {
      int num_rows = c.NumRows();
      int s = e_last + 1;
      int e = s;
      int num_rows_found = weight_matrix.at(e).rows();
      while (num_rows_found < num_rows) {
        e++;
        num_rows_found += weight_matrix.at(e).rows();
      }
      e_last = e;
      c.SetWeightMatrixIndex(s, e);
      if (num_rows_found != num_rows) {
        throw std::runtime_error("Weight matrix incompatible with Jacobian.");
      }
    }
    weight_matrix_ready_ = true;
  }

  solver_.AssembleFromCliques(clique_assemblers_);

#ifndef NDEBUG
  // Destroy references to argument weight_matrix.
  for (auto& c : jacobian_assemblers_) {
    c.SetWeightMatrixPointer(NULL);
  }
#endif

  factorization_ready_ = false;
  matrix_ready_ = true;
}

void SuperNodalSolver::Factor() {
  if (!matrix_ready_) {
    throw std::runtime_error(
        std::string("Call to Factor() failed: weight matrix not set."));
  }
  solver_.Factor();
  factorization_ready_ = true;
  matrix_ready_ = false;
}

Eigen::MatrixXd SuperNodalSolver::Solve(const Eigen::VectorXd& b) {
  if (!factorization_ready_) {
    throw std::runtime_error(
        std::string("Call to Solve() failed: factorization not ready."));
  }
  MatrixXd y = b;
  Eigen::Map<MatrixXd, Eigen::Aligned> ymap(y.data(), b.rows(), 1);
  solver_.SolveInPlace(&ymap);
  return y;
}

void SuperNodalSolver::SolveInPlace(Eigen::VectorXd* b) {
  if (!factorization_ready_) {
    throw std::runtime_error(
        std::string("Call to Solve() failed: factorization not ready."));
  }
  Eigen::Map<MatrixXd, Eigen::Aligned> ymap(b->data(), b->rows(), 1);
  solver_.SolveInPlace(&ymap);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
