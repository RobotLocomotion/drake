#ifndef DRAKEGRADIENTUTIL_H_
#define DRAKEGRADIENTUTIL_H_

#include <Eigen/Core>
#include <cmath>
#include <iostream> // TODO: get rid of this one
#include <cassert>

/*
 * Methods are templated, which is why declaration and implementation are in the same file
 * (otherwise you'd have to include the .cpp anyway).
 */

/*
 * Profile results: looks like return value optimization works; a version that sets a reference
 * instead of returning a value is just as fast and this is cleaner.
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>transposeGrad(
    const Eigen::MatrixBase<Derived>& dX, int rows_X)
{
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> dX_transpose(dX.rows(), dX.cols());
  int numel = dX.rows();
  int index = 0;
  for (int i = 0; i < numel; i++) {
    dX_transpose.row(i) = dX.row(index);
    index += rows_X;
    if (index >= numel) {
      index = (index % numel) + 1;
    }
  }
//  std::cout << "dX_transpose in transposeGrad: " << dX_transpose << std::endl << std::endl;
  return dX_transpose;
}

#define MATGRADMULTMAT_NUMROWS(rows_A, cols_B) (((int)rows_A == Eigen::Dynamic || (int)cols_B == Eigen::Dynamic) ? Eigen::Dynamic \
		: ((int)rows_A * (int)cols_B))

template <typename DerivedA, typename DerivedB, typename DerivedDA, typename DerivedDB>
Eigen::Matrix<typename DerivedA::Scalar, MATGRADMULTMAT_NUMROWS(DerivedA::RowsAtCompileTime, DerivedB::ColsAtCompileTime), DerivedDA::ColsAtCompileTime>
matGradMultMat(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    const Eigen::MatrixBase<DerivedDA>& dA,
    const Eigen::MatrixBase<DerivedDB>& dB) {
  assert(dA.cols() == dB.cols());
  const int nq = dA.cols();

  Eigen::Matrix<typename DerivedA::Scalar,
  MATGRADMULTMAT_NUMROWS(DerivedA::RowsAtCompileTime, DerivedB::ColsAtCompileTime),
  DerivedDA::ColsAtCompileTime> ret(A.rows() * B.cols(), nq);

  for (int col = 0; col < B.cols(); col++) {
    auto block = ret.block(col * A.rows(), 0, A.rows(), nq);

    // A * dB part:
    block = A * dB.block(col * A.cols(), 0, A.cols(), nq);

    for (int row = 0; row < B.rows(); row++) {
      // B * dA part:
      block += B(row, col) * dA.block(row * A.rows(), 0, A.rows(), nq);
    }
  }
  return ret;

  // much slower and requires unsupported eigen/unsupported:
  //  return Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(B.cols(), B.cols()), A) * dB + Eigen::kroneckerProduct(B.transpose(), Eigen::MatrixXd::Identity(A.rows(), A.rows())) * dA;
}


#define MATGRADMULT_NUMROWS(rows_dA, rows_b) (((int)rows_dA == Eigen::Dynamic || (int)rows_b == Eigen::Dynamic) ? Eigen::Dynamic \
		: ((int)rows_dA / (int)rows_b))

template<typename DerivedDA, typename Derivedb>
Eigen::Matrix<typename DerivedDA::Scalar, MATGRADMULT_NUMROWS(DerivedDA::RowsAtCompileTime, Derivedb::RowsAtCompileTime), DerivedDA::ColsAtCompileTime>
matGradMult(const Eigen::MatrixBase<DerivedDA>& dA, const Eigen::MatrixBase<Derivedb>& b) {
  const int nq = dA.cols();
  assert(b.cols() == 1);
  assert(dA.rows() % b.rows() == 0);
  const int A_rows = dA.rows() / b.rows();

  Eigen::Matrix<typename DerivedDA::Scalar, MATGRADMULT_NUMROWS(DerivedDA::RowsAtCompileTime, Derivedb::RowsAtCompileTime), DerivedDA::ColsAtCompileTime> ret(A_rows, nq);

  ret.setZero();
  for (int row = 0; row < b.rows(); row++) {
    ret += b(row, 0) * dA.block(row * A_rows, 0, A_rows, nq);
  }
  return ret;
}

// TODO: move to different file
// WARNING: not reshaping to Matlab geval output format!
template <typename DerivedA, typename DerivedB>
void normalizeVec(
    const Eigen::MatrixBase<DerivedA>& x,
    Eigen::MatrixBase<DerivedB>& x_norm,
    Eigen::MatrixXd* dx_norm = nullptr, // TODO: type
    Eigen::MatrixXd* ddx_norm = nullptr) { // TODO: type

  typename DerivedA::Scalar xdotx = x.squaredNorm();
  typename DerivedA::Scalar norm_x = std::sqrt(xdotx);
  x_norm = x / norm_x;

  if (dx_norm) {
    const int rows = DerivedA::RowsAtCompileTime;
    (*dx_norm) = (Eigen::Matrix<typename DerivedA::Scalar, rows, rows>::Identity(x.rows(), x.rows()) - x * x.transpose() / xdotx) / norm_x;

    if (ddx_norm) {
      auto dx_norm_transpose = transposeGrad(*dx_norm, x.rows());
      auto ddx_norm_times_norm = -matGradMultMat(x_norm, x_norm.transpose(), (*dx_norm), dx_norm_transpose);
      auto dnorm_inv = -x.transpose() / (xdotx * norm_x);
      (*ddx_norm) = ddx_norm_times_norm / norm_x;
      auto temp = (*dx_norm) * norm_x;
      int n = x.rows();
      for (int col = 0; col < n; col++) {
        auto column_as_matrix = (dnorm_inv(0, col) * temp);
        for (int row_block = 0; row_block < n; row_block++) {
          ddx_norm->block(row_block * n, col, n, 1) += column_as_matrix.col(row_block);
        }
      }
    }
  }
}


#endif /* DRAKEGRADIENTUTIL_H_ */
