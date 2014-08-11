#ifndef DRAKEGRADIENTUTIL_H_
#define DRAKEGRADIENTUTIL_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <cassert>
#include <vector>
#include <array>

constexpr int gradientNumRows(int A_size, int nq, int derivativeOrder) {
  return (A_size == Eigen::Dynamic || nq == Eigen::Dynamic)
    ? Eigen::Dynamic
    : (derivativeOrder == 1 ? A_size : nq * gradientNumRows(A_size, nq, derivativeOrder - 1));
}

template<typename Derived, int Nq, int DerivativeOrder = 1>
struct Gradient {
  typedef Eigen::Matrix<typename Derived::Scalar, gradientNumRows(Derived::SizeAtCompileTime, Nq, DerivativeOrder), Nq> type;
};

/*
 * Profile results: looks like return value optimization works; a version that sets a reference
 * instead of returning a value is just as fast and this is cleaner.
 */
template <typename Derived>
typename Derived::PlainObject transposeGrad(
    const Eigen::MatrixBase<Derived>& dX, int rows_X)
{
  typename Derived::PlainObject dX_transpose(dX.rows(), dX.cols());
  int numel = dX.rows();
  int index = 0;
  for (int i = 0; i < numel; i++) {
    dX_transpose.row(i) = dX.row(index);
    index += rows_X;
    if (index >= numel) {
      index = (index % numel) + 1;
    }
  }
  return dX_transpose;
}

constexpr int matGradMultMatNumRows(int rows_A, int cols_B) {
  return (rows_A == Eigen::Dynamic || cols_B == Eigen::Dynamic) ? Eigen::Dynamic : (rows_A * cols_B);
}

template <typename DerivedA, typename DerivedB, typename DerivedDA, typename DerivedDB>
Eigen::Matrix<typename DerivedA::Scalar, matGradMultMatNumRows(DerivedA::RowsAtCompileTime, DerivedB::ColsAtCompileTime), DerivedDA::ColsAtCompileTime>
matGradMultMat(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    const Eigen::MatrixBase<DerivedDA>& dA,
    const Eigen::MatrixBase<DerivedDB>& dB) {
  assert(dA.cols() == dB.cols());
  const int nq = dA.cols();
  const int nq_at_compile_time = DerivedDA::ColsAtCompileTime;

  Eigen::Matrix<typename DerivedA::Scalar,
  matGradMultMatNumRows(DerivedA::RowsAtCompileTime, DerivedB::ColsAtCompileTime),
  DerivedDA::ColsAtCompileTime> ret(A.rows() * B.cols(), nq);

  for (int col = 0; col < B.cols(); col++) {
    auto block = ret.template block<DerivedA::RowsAtCompileTime, nq_at_compile_time>(col * A.rows(), 0, A.rows(), nq);

    // A * dB part:
    block.noalias() = A * dB.template block<DerivedA::ColsAtCompileTime, nq_at_compile_time>(col * A.cols(), 0, A.cols(), nq);

    for (int row = 0; row < B.rows(); row++) {
      // B * dA part:
      block.noalias() += B(row, col) * dA.template block<DerivedA::RowsAtCompileTime, nq_at_compile_time>(row * A.rows(), 0, A.rows(), nq);
    }
  }
  return ret;

  // much slower and requires eigen/unsupported:
//  return Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(B.cols(), B.cols()), A) * dB + Eigen::kroneckerProduct(B.transpose(), Eigen::MatrixXd::Identity(A.rows(), A.rows())) * dA;
}

constexpr int matGradMultNumRows(int rows_dA, int rows_b) {
  return (rows_dA == Eigen::Dynamic || rows_b == Eigen::Dynamic) ? Eigen::Dynamic : (rows_dA / rows_b);
}
template<typename DerivedDA, typename Derivedb>
Eigen::Matrix<typename DerivedDA::Scalar, matGradMultNumRows(DerivedDA::RowsAtCompileTime, Derivedb::RowsAtCompileTime), DerivedDA::ColsAtCompileTime>
matGradMult(const Eigen::MatrixBase<DerivedDA>& dA, const Eigen::MatrixBase<Derivedb>& b) {
  const int nq = dA.cols();
  assert(b.cols() == 1);
  assert(dA.rows() % b.rows() == 0);
  const int A_rows = dA.rows() / b.rows();

  Eigen::Matrix<typename DerivedDA::Scalar, matGradMultNumRows(DerivedDA::RowsAtCompileTime, Derivedb::RowsAtCompileTime), DerivedDA::ColsAtCompileTime> ret(A_rows, nq);

  ret.setZero();
  for (int row = 0; row < b.rows(); row++) {
    ret += b(row, 0) * dA.block(row * A_rows, 0, A_rows, nq);
  }
  return ret;
}

// TODO: could save copies once http://eigen.tuxfamily.org/bz/show_bug.cgi?id=329 is fixed
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> getSubMatrixGradient(
    const Eigen::MatrixBase<Derived>& dM, const std::vector<int>& rows, const std::vector<int>& cols,
    int M_rows, int q_start = 0, int q_subvector_size = -1) {
  if (q_subvector_size < 0) {
    q_subvector_size = dM.cols() - q_start;
  }
  Eigen::MatrixXd dM_submatrix(rows.size() * cols.size(), q_subvector_size);
  int index = 0;
  for (int col : cols) {
    for (int row : rows) {
      dM_submatrix.row(index) = dM.block(row + col * M_rows, q_start, 1, q_subvector_size);
      index++;
    }
  }
  return dM_submatrix;
}

template<typename Derived, unsigned long NRows, unsigned long NCols>
Eigen::Matrix<typename Derived::Scalar, NRows * NCols, Derived::ColsAtCompileTime>
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM, const std::array<int, NRows>& rows, const std::array<int, NCols>& cols, int M_rows) {
  Eigen::Matrix<typename Derived::Scalar, NRows * NCols, Derived::ColsAtCompileTime> dM_submatrix(rows.size() * cols.size(), dM.cols());
  int index = 0;
  for (int col : cols) {
    for (int row : rows) {
      dM_submatrix.row(index++) = dM.row(row + col * M_rows);
    }
  }
  return dM_submatrix;
}

constexpr int getRowBlockGradientNumRows(unsigned long M_cols, unsigned long block_rows) {
  return M_cols == Eigen::Dynamic ? Eigen::Dynamic : M_cols * block_rows;
}

template<typename DerivedA, typename DerivedB>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::vector<int>& rows, const std::vector<int>& cols, int M_rows, int q_start = 0, int q_subvector_size = -1) {
  if (q_subvector_size < 0) {
    q_subvector_size = dM.cols() - q_start;
  }
  int index = 0;
  for (int col : cols) {
    for (int row : rows) {
      dM.block(row + col * M_rows, q_start, 1, q_subvector_size) = dM_submatrix.row(index++);
    }
  }
}

template<typename DerivedA, typename DerivedB, unsigned long NRows, unsigned long NCols>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::array<int, NRows>& rows, const std::array<int, NCols>& cols, int M_rows) {
  int index = 0;
  for (int col : cols) {
    for (int row : rows) {
      dM.row(row + col * M_rows) = dM_submatrix.row(index++);
    }
  }
}


#endif /* DRAKEGRADIENTUTIL_H_ */
