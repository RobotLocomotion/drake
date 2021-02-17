#pragma once

#include <array>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/math/gradient.h"

namespace drake {
namespace math {

template <std::size_t Size>
std::array<int, Size> intRange(int start) {
  std::array<int, Size> ret;
  for (unsigned int i = 0; i < Size; i++) {
    ret[i] = i + start;
  }
  return ret;
}

/*
 * Output type of matGradMultMat
 */
template <typename DerivedA, typename DerivedB, typename DerivedDA>
struct MatGradMultMat {
  typedef typename Eigen::Matrix<
      typename DerivedA::Scalar,
      (DerivedA::RowsAtCompileTime == Eigen::Dynamic ||
               DerivedB::ColsAtCompileTime == Eigen::Dynamic
           ? Eigen::Dynamic
           : static_cast<int>(DerivedA::RowsAtCompileTime) *
             static_cast<int>(DerivedB::ColsAtCompileTime)),
      DerivedDA::ColsAtCompileTime> type;
};

/*
 * Output type of matGradMult
 */
template <typename DerivedDA, typename DerivedB>
struct MatGradMult {
  typedef typename Eigen::Matrix<
      typename DerivedDA::Scalar,
      (DerivedDA::RowsAtCompileTime == Eigen::Dynamic ||
               DerivedB::SizeAtCompileTime == Eigen::Dynamic
           ? Eigen::Dynamic
           : static_cast<int>(DerivedDA::RowsAtCompileTime) /
             static_cast<int>(DerivedB::RowsAtCompileTime) *
             static_cast<int>(DerivedB::ColsAtCompileTime)),
      DerivedDA::ColsAtCompileTime> type;
};

/*
 * Output type and array types of getSubMatrixGradient for std::arrays
 * specifying rows and columns
 */
template <int QSubvectorSize, typename Derived, std::size_t NRows,
          std::size_t NCols>
struct GetSubMatrixGradientArray {
  typedef typename Eigen::Matrix<typename Derived::Scalar, (NRows * NCols),
                                 ((QSubvectorSize == Eigen::Dynamic)
                                      ? Derived::ColsAtCompileTime
                                      : QSubvectorSize)> type;
};

/*
 * Output type of getSubMatrixGradient for a single element of the input matrix
 */
template <int QSubvectorSize, typename Derived>
struct GetSubMatrixGradientSingleElement {
  typedef typename Eigen::Block<const Derived, 1,
                                ((QSubvectorSize == Eigen::Dynamic)
                                     ? Derived::ColsAtCompileTime
                                     : QSubvectorSize)> type;
};

/*
 * Profile results: looks like return value optimization works; a version that
 * sets a reference
 * instead of returning a value is just as fast and this is cleaner.
 */
template <typename Derived>
typename Derived::PlainObject transposeGrad(
    const Eigen::MatrixBase<Derived>& dX, typename Derived::Index rows_X) {
  typename Derived::PlainObject dX_transpose(dX.rows(), dX.cols());
  typename Derived::Index numel = dX.rows();
  typename Derived::Index index = 0;
  for (int i = 0; i < numel; i++) {
    dX_transpose.row(i) = dX.row(index);
    index += rows_X;
    if (index >= numel) {
      index = (index % numel) + 1;
    }
  }
  return dX_transpose;
}

template <typename DerivedA, typename DerivedB, typename DerivedDA,
          typename DerivedDB>
typename MatGradMultMat<DerivedA, DerivedB, DerivedDA>::type matGradMultMat(
    const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedB>& B,
    const Eigen::MatrixBase<DerivedDA>& dA,
    const Eigen::MatrixBase<DerivedDB>& dB) {
  DRAKE_ASSERT(dA.cols() == dB.cols());

  typename MatGradMultMat<DerivedA, DerivedB, DerivedDA>::type ret(
      A.rows() * B.cols(), dA.cols());

  for (int col = 0; col < B.cols(); col++) {
    auto block = ret.template block<DerivedA::RowsAtCompileTime,
                                    DerivedDA::ColsAtCompileTime>(
        col * A.rows(), 0, A.rows(), dA.cols());

    // A * dB part:
    block.noalias() = A *
                      dB.template block<DerivedA::ColsAtCompileTime,
                                        DerivedDA::ColsAtCompileTime>(
                          col * A.cols(), 0, A.cols(), dA.cols());

    for (int row = 0; row < B.rows(); row++) {
      // B * dA part:
      block.noalias() += B(row, col) *
                         dA.template block<DerivedA::RowsAtCompileTime,
                                           DerivedDA::ColsAtCompileTime>(
                             row * A.rows(), 0, A.rows(), dA.cols());
    }
  }
  return ret;

  // much slower and requires eigen/unsupported:
  //  return Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(B.cols(),
  //  B.cols()), A) * dB + Eigen::kroneckerProduct(B.transpose(),
  //  Eigen::MatrixXd::Identity(A.rows(), A.rows())) * dA;
}

template <typename DerivedDA, typename DerivedB>
typename MatGradMult<DerivedDA, DerivedB>::type matGradMult(
    const Eigen::MatrixBase<DerivedDA>& dA,
    const Eigen::MatrixBase<DerivedB>& B) {
  DRAKE_ASSERT(B.rows() == 0 ? dA.rows() == 0 : dA.rows() % B.rows() == 0);
  typename DerivedDA::Index A_rows = B.rows() == 0 ? 0 : dA.rows() / B.rows();
  const int A_rows_at_compile_time =
      (DerivedDA::RowsAtCompileTime == Eigen::Dynamic ||
       DerivedB::RowsAtCompileTime == Eigen::Dynamic)
          ? Eigen::Dynamic
          : static_cast<int>(DerivedDA::RowsAtCompileTime) /
                static_cast<int>(DerivedB::RowsAtCompileTime);

  typename MatGradMult<DerivedDA, DerivedB>::type ret(A_rows * B.cols(),
                                                      dA.cols());
  ret.setZero();
  for (int col = 0; col < B.cols(); col++) {
    auto block = ret.template block<A_rows_at_compile_time,
                                    DerivedDA::ColsAtCompileTime>(
        col * A_rows, 0, A_rows, dA.cols());
    for (int row = 0; row < B.rows(); row++) {
      // B * dA part:
      block.noalias() += B(row, col) *
                         dA.template block<A_rows_at_compile_time,
                                           DerivedDA::ColsAtCompileTime>(
                             row * A_rows, 0, A_rows, dA.cols());
    }
  }
  return ret;
}

// TODO(tkoolen): could save copies once
// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=329 is fixed
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM,
                     const std::vector<int>& rows, const std::vector<int>& cols,
                     typename Derived::Index M_rows, int q_start = 0,
                     typename Derived::Index q_subvector_size = -1) {
  if (q_subvector_size < 0) {
    q_subvector_size = dM.cols() - q_start;
  }
  Eigen::MatrixXd dM_submatrix(rows.size() * cols.size(), q_subvector_size);
  int index = 0;
  for (std::vector<int>::const_iterator col = cols.begin(); col != cols.end();
       ++col) {
    for (std::vector<int>::const_iterator row = rows.begin(); row != rows.end();
         ++row) {
      dM_submatrix.row(index) =
          dM.block(*row + *col * M_rows, q_start, 1, q_subvector_size);
      index++;
    }
  }
  return dM_submatrix;
}

template <int QSubvectorSize, typename Derived, std::size_t NRows,
          std::size_t NCols>
typename GetSubMatrixGradientArray<QSubvectorSize, Derived, NRows, NCols>::type
getSubMatrixGradient(
    const Eigen::MatrixBase<Derived>& dM, const std::array<int, NRows>& rows,
    const std::array<int, NCols>& cols, typename Derived::Index M_rows,
    int q_start = 0,
    typename Derived::Index q_subvector_size = QSubvectorSize) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  Eigen::Matrix<typename Derived::Scalar, NRows * NCols,
                Derived::ColsAtCompileTime> dM_submatrix(NRows * NCols,
                                                         q_subvector_size);
  int index = 0;
  for (typename std::array<int, NCols>::const_iterator col = cols.begin();
       col != cols.end(); ++col) {
    for (typename std::array<int, NRows>::const_iterator row = rows.begin();
         row != rows.end(); ++row) {
      dM_submatrix.row(index++) = dM.template block<1, QSubvectorSize>(
          (*row) + (*col) * M_rows, q_start, 1, q_subvector_size);
    }
  }
  return dM_submatrix;
}

template <int QSubvectorSize, typename Derived>
typename GetSubMatrixGradientSingleElement<QSubvectorSize, Derived>::type
getSubMatrixGradient(
    const Eigen::MatrixBase<Derived>& dM, int row, int col,
    typename Derived::Index M_rows, typename Derived::Index q_start = 0,
    typename Derived::Index q_subvector_size = QSubvectorSize) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  return dM.template block<1, QSubvectorSize>(row + col * M_rows, q_start, 1,
                                              q_subvector_size);
}

template <typename DerivedA, typename DerivedB>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM,
                          const Eigen::MatrixBase<DerivedB>& dM_submatrix,
                          const std::vector<int>& rows,
                          const std::vector<int>& cols,
                          typename DerivedA::Index M_rows,
                          typename DerivedA::Index q_start = 0,
                          typename DerivedA::Index q_subvector_size = -1) {
  if (q_subvector_size < 0) {
    q_subvector_size = dM.cols() - q_start;
  }
  int index = 0;
  for (typename std::vector<int>::const_iterator col = cols.begin();
       col != cols.end(); ++col) {
    for (typename std::vector<int>::const_iterator row = rows.begin();
         row != rows.end(); ++row) {
      dM.block((*row) + (*col) * M_rows, q_start, 1, q_subvector_size) =
          dM_submatrix.row(index++);
    }
  }
}

template <int QSubvectorSize, typename DerivedA, typename DerivedB,
          std::size_t NRows, std::size_t NCols>
void setSubMatrixGradient(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::MatrixBase<DerivedA>& dM,
    const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::array<int, NRows>& rows, const std::array<int, NCols>& cols,
    typename DerivedA::Index M_rows, typename DerivedA::Index q_start = 0,
    typename DerivedA::Index q_subvector_size = QSubvectorSize) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  int index = 0;
  for (typename std::array<int, NCols>::const_iterator col = cols.begin();
       col != cols.end(); ++col) {
    for (typename std::array<int, NRows>::const_iterator row = rows.begin();
         row != rows.end(); ++row) {
      dM.template block<1, QSubvectorSize>((*row) + (*col) * M_rows, q_start, 1,
                                           q_subvector_size) =
          dM_submatrix.row(index++);
    }
  }
}

template <int QSubvectorSize, typename DerivedDM, typename DerivedDMSub>
void setSubMatrixGradient(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::MatrixBase<DerivedDM>& dM,
    const Eigen::MatrixBase<DerivedDMSub>& dM_submatrix, int row, int col,
    typename DerivedDM::Index M_rows, typename DerivedDM::Index q_start = 0,
    typename DerivedDM::Index q_subvector_size = QSubvectorSize) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  dM.template block<1, QSubvectorSize>(row + col * M_rows, q_start, 1,
                                       q_subvector_size) = dM_submatrix;
}

}  // namespace math
}  // namespace drake
