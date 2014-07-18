#ifndef DRAKEGRADIENTUTIL_H_
#define DRAKEGRADIENTUTIL_H_

#include <Eigen/Core>
#include <cmath>
#include <iostream> // TODO: get rid of this one

/*
 * Methods are templated, which is why declaration and implementation are in the same file
 * (otherwise you'd have to include the .cpp anyway).
 */

/*
 * Profile results: looks like return value optimization works; a version that sets a reference
 * instead of returning a value is just as fast and this is cleaner.
 */
template <typename Derived>
typename Derived::PlainObject transposeGrad(const Eigen::MatrixBase<Derived>& dX, int rows_X)
{
  typename Derived::PlainObject dX_transpose;
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

template <typename DerivedA, typename DerivedB, typename DerivedDA, typename DerivedDB>
Eigen::Matrix<
  typename DerivedA::Scalar,
  Eigen::Dynamic, // TODO: could figure out compile time size to use
  DerivedDA::ColsAtCompileTime> matGradMultMat(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    const Eigen::MatrixBase<DerivedDA>& dA,
    const Eigen::MatrixBase<DerivedDB>& dB)
{
  const int nq = dA.cols();
  Eigen::Matrix<typename DerivedA::Scalar, Eigen::Dynamic, DerivedDA::ColsAtCompileTime> ret(A.rows() * B.cols(), nq);

  for (int row = 0; row < B.cols(); row++) {
    auto block = ret.block(row * A.rows(), 0, A.rows(), nq);

    // A * dB part:
    block = A * dB.block(row * A.cols(), 0, A.cols(), nq);

    for (int col = 0; col < B.rows(); col++) {
      // B * dA part:
      block += (B(col, row) * dA.block(col * A.rows(), 0, A.rows(), nq)).eval();
    }
  }

  return ret;

  // much slower and requires unsupported eigen/unsupported:
//  return Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(B.cols(), B.cols()), A) * dB + Eigen::kroneckerProduct(B.transpose(), Eigen::MatrixXd::Identity(A.rows(), A.rows())) * dA;
}

#endif /* DRAKEGRADIENTUTIL_H_ */
