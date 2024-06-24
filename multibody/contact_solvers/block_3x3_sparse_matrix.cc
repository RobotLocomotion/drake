#include "drake/multibody/contact_solvers/block_3x3_sparse_matrix.h"

#include <algorithm>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

template <typename T>
using Triplet = typename Block3x3SparseMatrix<T>::Triplet;
/* Given a vector of Triplets with the same block row index that is sorted
 in increasing block column index, merge entries that have the same block
 column index in place by summing them up. After this function, the triplets in
 `data` is still sorted in increasing block column index but does not contain
 duplicate block column indices, and the size of the vector is equal to the
 number of unique block column indices. */
template <typename T>
void MergeDuplicates(std::vector<Triplet<T>>* data) {
  DRAKE_DEMAND(data != nullptr);
  if (data->empty()) return;
  const int block_row = std::get<0>((*data)[0]);
  /* Traverse the vector with slow/fast pointer where the fast pointer is the
   source and the slow pointer is the destination. */
  int slow = 0;
  int fast = 0;
  int slow_col = std::get<1>((*data)[slow]);
  Matrix3<T> value = Matrix3<T>::Zero();
  while (fast < static_cast<int>(data->size())) {
    const Triplet<T>& t = (*data)[fast];
    const int fast_col = std::get<1>(t);
    if (fast_col == slow_col) {
      value += std::get<2>(t);
    } else {
      (*data)[slow++] = {block_row, slow_col, value};
      slow_col = fast_col;
      value = std::get<2>(t);
    }
    ++fast;
  }
  /* Don't forget to write down the last batch of triplets. */
  (*data)[slow++] = {block_row, slow_col, value};
  data->resize(slow);
}

}  // namespace

template <class T>
void Block3x3SparseMatrix<T>::SetFromTriplets(
    const std::vector<Triplet>& triplets) {
  /* Clear all existing data. */
  for (std::vector<Triplet>& row : row_data_) {
    row.clear();
  }
  for (std::vector<Index>& indices : col_to_indices_) {
    indices.clear();
  }

  /* Populate `row_data_` and `col_to_indices_`. */
  for (const Triplet& t : triplets) {
    const int block_row = std::get<0>(t);
    const int block_col = std::get<1>(t);
    DRAKE_DEMAND(0 <= block_row && block_row < block_rows_);
    DRAKE_DEMAND(0 <= block_col && block_col < block_cols_);
    row_data_[block_row].push_back(t);
  }

  /* Maintain the ordering invariance in `row_data_` and compute the total
   number of non-zero blocks. */
  num_blocks_ = 0;
  for (int r = 0; r < block_rows_; ++r) {
    std::sort(row_data_[r].begin(), row_data_[r].end(),
              [](const Triplet& t1, const Triplet& t2) {
                const int col_1 = std::get<1>(t1);
                const int col_2 = std::get<1>(t2);
                return col_1 < col_2;
              });
    MergeDuplicates<T>(&(row_data_[r]));
    num_blocks_ += row_data_[r].size();
  }

  for (int block_row = 0; block_row < static_cast<int>(row_data_.size());
       ++block_row) {
    for (int flat_index = 0;
         flat_index < static_cast<int>(row_data_[block_row].size());
         ++flat_index) {
      const int block_col = std::get<1>(row_data_[block_row][flat_index]);
      col_to_indices_[block_col].push_back({block_row, flat_index});
    }
  }
}

template <class T>
void Block3x3SparseMatrix<T>::MultiplyAndAddTo(
    const Eigen::Ref<const MatrixX<T>>& A, EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(A.rows() == cols());
  DRAKE_DEMAND(y->rows() == rows());
  for (const std::vector<Triplet>& row : row_data_) {
    for (const Triplet& triplet : row) {
      const int block_row = std::get<0>(triplet);
      const int block_col = std::get<1>(triplet);
      const Matrix3<T>& m = std::get<2>(triplet);
      y->template middleRows<3>(3 * block_row) +=
          m * A.template middleRows<3>(3 * block_col);
    }
  }
}

template <class T>
void Block3x3SparseMatrix<T>::LeftMultiplyAndAddTo(
    const Eigen::Ref<const MatrixX<T>>& A, EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(A.cols() == rows());
  DRAKE_DEMAND(y->rows() == A.rows());
  for (const std::vector<Triplet>& row : row_data_) {
    for (const Triplet& triplet : row) {
      const int block_row = std::get<0>(triplet);
      const int block_col = std::get<1>(triplet);
      const Matrix3<T>& m = std::get<2>(triplet);
      y->template middleCols<3>(3 * block_col) +=
          A.template middleCols<3>(3 * block_row) * m;
    }
  }
}

template <class T>
void Block3x3SparseMatrix<T>::TransposeAndMultiplyAndAddTo(
    const Eigen::Ref<const MatrixX<T>>& A, EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(rows() == A.rows());
  DRAKE_DEMAND(y->rows() == cols());
  for (const std::vector<Triplet>& row : row_data_) {
    for (const Triplet& triplet : row) {
      const int block_row = std::get<0>(triplet);
      const int block_col = std::get<1>(triplet);
      const Matrix3<T>& m = std::get<2>(triplet);
      y->template middleRows<3>(3 * block_col) +=
          m.transpose() * A.template middleRows<3>(3 * block_row);
    }
  }
}

template <class T>
void Block3x3SparseMatrix<T>::TransposeAndMultiplyAndAddTo(
    const Block3x3SparseMatrix<T>& A, EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(rows() == A.rows());
  DRAKE_DEMAND(y->rows() == this->cols());
  DRAKE_DEMAND(y->cols() == A.cols());

  if (A.row_data_.empty() || this->row_data_.empty()) {
    return;
  }

  /* We are performing yᵢⱼ += ∑ₖ Mₖᵢᵀ * Aₖⱼ. For each ij block in y, we need to
   sum over block row indices, k, where Mₖᵢ and Aₖⱼ both have non-zero blocks.
   We do this by looping over block row indices. For each block row k, we find
   all blocks of A and M that have k as block row index. Then we loop over their
   block column indices (i for M and j for A), perform the dense multiplication,
   and add to the corresponding block in y. */
  for (int k = 0; k < block_rows_; ++k) {
    for (const Triplet& m : row_data_[k]) {
      const int i = std::get<1>(m);  // block column index of M block
      const Matrix3<T>& M_ki = std::get<2>(m);
      for (const Triplet& a : A.row_data_[k]) {
        const int j = std::get<1>(a);  // block column index of A block
        const Matrix3<T>& A_kj = std::get<2>(a);
        y->template block<3, 3>(3 * i, 3 * j) += M_ki.transpose() * A_kj;
      }
    }
  }
}

template <class T>
void Block3x3SparseMatrix<T>::MultiplyWithScaledTransposeAndAddTo(
    const VectorX<T>& scale, EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(cols() == scale.size());
  DRAKE_DEMAND(rows() == y->rows());
  DRAKE_DEMAND(rows() == y->cols());
  /* Let y be the result, then we are computing
   yᵢⱼ += ∑ₖ Mᵢₖ * scaleₖ * Mⱼₖ. */
  for (int k = 0; k < block_cols_; ++k) {
    const std::vector<Index>& indices = col_to_indices_[k];
    const auto scale_block = scale.template segment<3>(3 * k);
    for (int a = 0; a < static_cast<int>(indices.size()); ++a) {
      const Triplet& t1 = row_data_[indices[a].row][indices[a].flat];
      const int i = std::get<0>(t1);
      const Matrix3<T>& M_ik = std::get<2>(t1);
      for (int b = a; b < static_cast<int>(indices.size()); ++b) {
        const Triplet& t2 = row_data_[indices[b].row][indices[b].flat];
        const int j = std::get<0>(t2);
        const Matrix3<T>& M_kj = std::get<2>(t2);
        const Matrix3<T> y_ij =
            M_ik * scale_block.asDiagonal() * M_kj.transpose();
        y->template block<3, 3>(3 * i, 3 * j) += y_ij;
        /* Exploit symmetry. */
        if (a != b) {
          y->template block<3, 3>(3 * j, 3 * i) += y_ij.transpose();
        }
      }
    }
  }
}

template <class T>
MatrixX<T> Block3x3SparseMatrix<T>::MakeDenseMatrix() const {
  MatrixX<T> result = MatrixX<T>::Zero(rows(), cols());
  for (const std::vector<Triplet>& row : row_data_) {
    for (const Triplet& triplet : row) {
      const int block_row = std::get<0>(triplet);
      const int block_col = std::get<1>(triplet);
      const Matrix3<T>& m = std::get<2>(triplet);
      result.template block<3, 3>(3 * block_row, 3 * block_col) = m;
    }
  }
  return result;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::contact_solvers::internal::Block3x3SparseMatrix);
