#include "drake/multibody/contact_solvers/supernodal_solver.h"

using Eigen::MatrixXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

SuperNodalSolver::~SuperNodalSolver() = default;

void SuperNodalSolver::SetWeightMatrix(
    const std::vector<Eigen::MatrixXd>& weight_matrix) {
  bool weight_matrix_compatible = DoSetWeightMatrix(weight_matrix);
  if (!weight_matrix_compatible) {
    throw std::runtime_error("Weight matrix incompatible with Jacobian.");
  }

  factorization_ready_ = false;
  matrix_ready_ = true;
}

Eigen::MatrixXd SuperNodalSolver::MakeFullMatrix() const {
  if (!matrix_ready_) {
    throw std::runtime_error(
        "Call to MakeFullMatrix() failed: weight matrix not set or matrix has "
        "been factored in place.");
  }
  return DoMakeFullMatrix();
}

bool SuperNodalSolver::Factor() {
  if (!matrix_ready_) {
    throw std::runtime_error("Call to Factor() failed: weight matrix not set.");
  }
  bool success = DoFactor();
  factorization_ready_ = success;
  matrix_ready_ = false;
  return success;
}

Eigen::VectorXd SuperNodalSolver::Solve(const Eigen::VectorXd& b) const {
  DRAKE_THROW_UNLESS(b.size() == GetSize());
  if (!factorization_ready_) {
    throw std::runtime_error(
        "Call to Solve() failed: factorization not ready.");
  }
  Eigen::VectorXd y = b;
  DoSolveInPlace(&y);
  return y;
}

void SuperNodalSolver::SolveInPlace(Eigen::VectorXd* b) const {
  DRAKE_THROW_UNLESS(b != nullptr && b->size() == GetSize());
  if (!factorization_ready_) {
    throw std::runtime_error(
        "Call to SolveInPlace() failed: factorization not ready.");
  }
  DoSolveInPlace(b);
}

std::vector<std::vector<int>> GetRowToTripletMapping(
    int num_row_blocks, const std::vector<BlockTriplet>& jacobian_blocks) {
  DRAKE_THROW_UNLESS(num_row_blocks >= 0);
  std::vector<std::vector<int>> y(num_row_blocks);
  std::vector<std::vector<int>> column(num_row_blocks);

  // Sorts row data by column using our assumption that at most two columns
  // are non-zero per row.
  auto sort_row_data_by_column = [](std::vector<int>* col_index,
                                    std::vector<int>* triplet_position) {
    if ((*col_index)[0] > (*col_index)[1]) {
      std::swap((*col_index)[0], (*col_index)[1]);
      std::swap((*triplet_position)[0], (*triplet_position)[1]);
    }
  };

  int cnt = 0;
  for (const auto& j : jacobian_blocks) {
    int index = j.row;
    if (index >= num_row_blocks) {
      throw std::runtime_error(fmt::format(
          "Jacobian block with block row index {} is greater than or equal to "
          "the total number of block rows in the Jacobian, {}.",
          index, num_row_blocks));
    }
    y[index].emplace_back(cnt);
    column[index].emplace_back(j.col);
    if (column[index].size() == 2) {
      sort_row_data_by_column(&column[index], &y[index]);
    }
    if (column[index].size() > 2) {
      throw std::runtime_error(
          "Jacobian can only be nonzero on at most two column blocks.");
    }
    ++cnt;
  }
  for (int i = 0; i < num_row_blocks; ++i) {
    if (y[i].empty()) {
      throw std::runtime_error(
          fmt::format("The {}-th block row in the Jacobian is empty.", i));
    }
  }
  return y;
}

std::vector<int> GetJacobianBlockSizesVerifyTriplets(
    const std::vector<BlockTriplet>& jacobian_blocks) {
  int max_index = -1;
  for (const auto& j : jacobian_blocks) {
    int col_index = j.col;
    if (col_index > max_index) {
      max_index = col_index;
    }
  }

  std::vector<int> block_column_size(max_index + 1, -1);

  for (const auto& j : jacobian_blocks) {
    int col_index = j.col;

    if ((j.value.cols() == 0) || (j.value.rows() == 0)) {
      throw std::runtime_error(
          "Invalid Jacobian triplets: a triplet contains an empty matrix");
    }

    if (block_column_size[col_index] == -1) {
      block_column_size[col_index] = j.value.cols();
    } else {
      if (block_column_size[col_index] != j.value.cols()) {
        throw std::runtime_error(fmt::format(
            "Invalid Jacobian triplets: conflicting block sizes specified "
            "for column {}.",
            col_index));
      }
    }
  }

  for (int i = 0; i < max_index; ++i) {
    // Verify that initial value of -1 has been overwritten. (A block column
    // size of zero will be caught by the empty matrix check.)
    if (block_column_size[i] < 0) {
      throw std::runtime_error(fmt::format(
          "Invalid Jacobian triplets: no triplet provided for column {}.", i));
    }
  }

  return block_column_size;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
