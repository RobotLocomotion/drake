#ifndef DRAKEGRADIENTUTIL_H_
#define DRAKEGRADIENTUTIL_H_

#include <Eigen/Core>
#include <cmath>
#include <iostream> // TODO: get rid of this one
#include <cassert>
#include <vector>
#include <array>
//#include <Eigen/KroneckerProduct> // unsupported...


/*
 * Methods are templated, which is why declaration and implementation are in the same file
 * (otherwise you'd have to include the .cpp anyway).
 */

template<typename Derived, int Nq, int DerivativeOrder>
struct GradientType {
  static constexpr const int gradientNumRows(int A_size, int nq, int derivativeOrder) {
    return (A_size == Eigen::Dynamic || nq == Eigen::Dynamic)
        ? Eigen::Dynamic
        : (derivativeOrder == 1 ? A_size : nq * gradientNumRows(A_size, nq, derivativeOrder - 1));
  }

  typedef Eigen::Matrix<typename Derived::Scalar, gradientNumRows(Derived::SizeAtCompileTime, Nq, DerivativeOrder), Nq> Type;
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
//  std::cout << "dX_transpose in transposeGrad: " << dX_transpose << std::endl << std::endl;
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

  Eigen::Matrix<typename DerivedA::Scalar,
  matGradMultMatNumRows(DerivedA::RowsAtCompileTime, DerivedB::ColsAtCompileTime),
  DerivedDA::ColsAtCompileTime> ret(A.rows() * B.cols(), nq);

  for (int col = 0; col < B.cols(); col++) {
    auto block = ret.block(col * A.rows(), 0, A.rows(), nq).noalias();

    // A * dB part:
    block = A * dB.block(col * A.cols(), 0, A.cols(), nq);

    for (int row = 0; row < B.rows(); row++) {
      // B * dA part:
      block += B(row, col) * dA.block(row * A.rows(), 0, A.rows(), nq);
    }
  }
  return ret;

  // much slower and requires eigen/unsupported:
//  std::cout << "B:\n" << B << "\n\n";
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

template<typename Scalar, typename DerivedS, typename DerivedQdotToV>
Eigen::Matrix<Scalar, 16, DerivedQdotToV::ColsAtCompileTime> dHomogTrans(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedS>& S,
    const Eigen::MatrixBase<DerivedQdotToV>& qdot_to_v) {
  const int nq_at_compile_time = DerivedQdotToV::ColsAtCompileTime;
  int nq = qdot_to_v.cols();
  auto qdot_to_twist = S * qdot_to_v;

  const int numel = 16;
  Eigen::Matrix<Scalar, numel, nq_at_compile_time> ret(numel, nq);

  const auto& Rx = T.linear().col(0);
  const auto& Ry = T.linear().col(1);
  const auto& Rz = T.linear().col(2);

  const auto& qdot_to_omega_x = qdot_to_twist.row(0);
  const auto& qdot_to_omega_y = qdot_to_twist.row(1);
  const auto& qdot_to_omega_z = qdot_to_twist.row(2);

  ret.middleRows(0, 3) = -Rz * qdot_to_omega_y + Ry * qdot_to_omega_z;
  ret.row(3).setZero();

  ret.middleRows(4, 3) = Rz * qdot_to_omega_x - Rx * qdot_to_omega_z;
  ret.row(7).setZero();

  ret.middleRows(8, 3) = -Ry * qdot_to_omega_x + Rx * qdot_to_omega_y;
  ret.row(11).setZero();

  ret.middleRows(12, 3) = T.linear() * qdot_to_twist.bottomRows(3);
  ret.row(15).setZero();

  return ret;
}

template<typename Scalar, typename DerivedDT>
Eigen::Matrix<Scalar, 16, DerivedDT::ColsAtCompileTime> dHomogTransInv(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedDT>& dT) {
  const int nq_at_compile_time = DerivedDT::ColsAtCompileTime;
  const int nq = dT.cols();

  const auto& R = T.linear();
  const auto& p = T.translation();

  std::array<int, 3> rows {0, 1, 2};
  std::array<int, 3> R_cols {0, 1, 2};
  std::array<int, 1> p_cols {3};

  auto dR = getSubMatrixGradient(dT, rows, R_cols, T.Rows);
  auto dp = getSubMatrixGradient(dT, rows, p_cols, T.Rows);

  auto dinvT_R = transposeGrad(dR, R.rows());
  auto dinvT_p = -R.transpose() * dp - matGradMult(dinvT_R, p);

  const int numel = 16;
  Eigen::Matrix<Scalar, numel, nq_at_compile_time> ret(numel, nq);
  setSubMatrixGradient(ret, dinvT_R, rows, R_cols, T.Rows);
  setSubMatrixGradient(ret, dinvT_p, rows, p_cols, T.Rows);

  // zero out gradient of elements in last row:
  const int last_row = 3;
  for (int col = 0; col < T.HDim; col++) {
    ret.row(last_row + col * T.Rows).setZero();
  }

  return ret;
}

// TODO: move to different file
// WARNING: not reshaping second derivative to Matlab geval output format!
template <typename Derived>
void normalizeVec(
    const Eigen::MatrixBase<Derived>& x,
    typename Derived::PlainObject& x_norm,
    typename GradientType<Derived, Derived::RowsAtCompileTime, 1>::Type* dx_norm = nullptr,
    typename GradientType<Derived, Derived::RowsAtCompileTime, 2>::Type* ddx_norm = nullptr) {

  typename Derived::Scalar xdotx = x.squaredNorm();
  typename Derived::Scalar norm_x = std::sqrt(xdotx);
  x_norm = x / norm_x;

  if (dx_norm) {
    dx_norm->setIdentity(x.rows(), x.rows());
    (*dx_norm) -= x * x.transpose() / xdotx;
    (*dx_norm) /= norm_x;

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
