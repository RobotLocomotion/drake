#include "drake/multibody/contact_solvers/block_3x3_sparse_matrix.h"

#include <algorithm>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <class T>
void Block3x3SparseMatrix<T>::SetFromTriplets(
    const std::vector<Triplet>& triplets) {
  /* Clear all existing data. */
  for (RowData& row : data_) {
    row.clear();
  }
  for (std::vector<Index>& indices : col_to_indices_) {
    indices.clear();
  }

  /* Populate `data_` and `col_to_indices_`. */
  for (const Triplet& t : triplets) {
    const int block_row = std::get<0>(t);
    const int block_col = std::get<1>(t);
    col_to_indices_[block_col].push_back(
        {block_row, static_cast<int>(data_[block_row].size())});
    data_[block_row].push_back(t);
  }
  num_blocks_ = triplets.size();

  /* Maintain the ordering invariance in `data_`. */
  for (int r = 0; r < block_rows_; ++r) {
    std::sort(data_[r].begin(), data_[r].end(),
              [](const Triplet& t1, const Triplet& t2) {
                const int col_1 = std::get<1>(t1);
                const int col_2 = std::get<1>(t2);
                return col_1 < col_2;
              });
  }

  /* Maintain the ordering invariance in `col_to_indices_`. */
  for (int c = 0; c < block_cols_; ++c) {
    std::sort(col_to_indices_[c].begin(), col_to_indices_[c].end(),
              [](const Index& i1, const Index& i2) {
                return i1.row < i2.row;
              });
  }
}

template <class T>
void Block3x3SparseMatrix<T>::MultiplyAndAddTo(
    const Eigen::Ref<const MatrixX<T>>& A, EigenPtr<VectorX<T>> y) const {
  DRAKE_DEMAND(A.rows() == cols());
  for (const RowData& row : data_) {
    for (const Triplet& triplet : row) {
      const int block_row = std::get<0>(triplet);
      const int block_col = std::get<1>(triplet);
      const Matrix3<T>& m = std::get<2>(triplet);
      y->template segment<3>(3 * block_row) +=
          m * A.template middleRows<3>(3 * block_col);
    }
  }
}

template <class T>
void Block3x3SparseMatrix<T>::LeftMultiplyAndAddTo(
    const Eigen::Ref<const MatrixX<T>>& A, EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(A.cols() == rows());
  for (const RowData& row : data_) {
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
  DRAKE_DEMAND(rows() == A.rows());
  for (const RowData& row : data_) {
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
  DRAKE_DEMAND(rows() == A.rows());
  DRAKE_DEMAND(y->rows() == this->cols());
  DRAKE_DEMAND(y->cols() == A.cols());

  if (A.data_.empty() || this->data_.empty()) {
    return;
  }

  /* In Einstein notation, We are performing *yᵢⱼ += Mₖᵢ * Aₖⱼ. For each ij
   entry in y, we need to sum over row indices, k, where Mₖᵢ and Aₖⱼ both
   have non-zero blocks. We do this by looping over row indices. For each k,
   we find all blocks of A and M that have k as row index. Then we loop over
   their column indices (i for M and j for A), perform the dense
   multiplication, and add to the corresponding entry in y. */

  for (int k = 0; k < block_rows_; ++k) {
    for (const Triplet& m : data_[k]) {
      const int i = std::get<1>(m);
      const Matrix3<T>& M_ki = std::get<2>(m).transpose();
      for (const Triplet& a : A.data_[k]) {
        const int j = std::get<1>(a);
        const Matrix3<T>& A_kj = std::get<2>(a);
        y->template block<3, 3>(3 * i, 3 * j) += M_ki * A_kj;
      }
    }
  }
}

template <class T>
void Block3x3SparseMatrix<T>::MultiplyWithScaledTransposeAndAddTo(
    const VectorX<T>& scale, EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(y !=nullptr);
  DRAKE_DEMAND(cols() == scale.size());
  DRAKE_DEMAND(rows() == y->rows());
  DRAKE_DEMAND(rows() == y->cols());
  /* Let y be the result, then we are computing
   yᵢⱼ += ∑ₖ Mᵢₖ * scaleₖ * Mⱼₖ. */
  for (int k = 0; k < block_cols_; ++k) {
    const std::vector<Index>& indices = col_to_indices_[k];
    const auto scale_block = scale.template segment<3>(3 * k);
    for (int a = 0; a < static_cast<int>(indices.size()); ++a) {
      const Triplet& t1 = data_[indices[a].row][indices[a].flat];
      const int i = std::get<0>(t1);
      const Matrix3<T>& M_ik = std::get<2>(t1);
      for (int b = a; b < static_cast<int>(indices.size()); ++b) {
        const Triplet& t2 = data_[indices[b].row][indices[b].flat];
        const int j = std::get<0>(t2);
        const Matrix3<T>& M_jk = std::get<2>(t2).transpose();
        Matrix3<T> y_ij = M_ik * scale_block.asDiagonal() * M_jk;
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
  for (const RowData& row : data_) {
    for (const Triplet& triplet : row) {
      const int block_row = std::get<0>(triplet);
      const int block_col = std::get<1>(triplet);
      const Matrix3<T>& m = std::get<2>(triplet);
      result.template block<3, 3>(3 * block_row, 3 * block_col) += m;
    }
  }
  return result;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::contact_solvers::internal::Block3x3SparseMatrix)
