#include "drakeGradientUtil.h"
#include <Eigen/Core>
#include <Eigen/KroneckerProduct> // unsupported...
#include <iostream>
#include <chrono>
#include <stdexcept>
#include <vector>
#include <array>

using namespace Eigen;

template<typename TimeT = std::chrono::milliseconds>
struct measure
{
  template<typename F, typename ...Args>
  static typename TimeT::rep execution(F func, Args&&... args)
  {
    auto start = std::chrono::system_clock::now();

    // Now call the function with all the parameters you need.
    func(std::forward<Args>(args)...);

    auto duration = std::chrono::duration_cast< TimeT>
    (std::chrono::system_clock::now() - start);

    return duration.count();
  }
};

template<typename Derived>
void setLinearIndices(MatrixBase<Derived>& A)
{
  for (int col = 0; col < A.cols(); col++) {
    for (int row = 0; row < A.rows(); row++) {
      A(row, col) = row + col * A.rows() + 1;
    }
  }
}

void testTransposeGrad(int ntests)
{
  const int rows_X = 8;
  const int cols_X = 6;
  const int numel_X = rows_X * cols_X;
  const int nq = 34;
//  Matrix<double, numel_X, nq> dX;
////  MatrixXd dX(numel_X, nq);

//  Matrix<double, numel_X, nq> dX_transpose;

  for (int testnr = 0; testnr < ntests; testnr++) {
    Matrix<double, numel_X, nq> dX = Matrix<double, numel_X, nq>::Random();
//    std::cout << "dX:\n" << dX << std::endl << std::endl;
    auto dX_transpose = transposeGrad(dX, rows_X);
    volatile auto vol = dX_transpose; // volatile to make sure that the result doesn't get discarded in compiler optimization

//    transposeGrad(dX, rows_X, dX_transpose);
//    std::cout << "dX_transpose:\n" << dX_transpose << std::endl;
  }
}

void testMatGradMultMat(int ntests, bool check)
{
  const int nq = 34;
  for (int testnr = 0; testnr < ntests; testnr++) {

//    MatrixXd A = MatrixXd::Random(8, 6).eval();
//    MatrixXd B = MatrixXd::Random(A.cols(), 9).eval();
    auto A = Matrix<double, 8, 6>::Random().eval();
    auto B = Matrix<double, A.ColsAtCompileTime, 9>::Random().eval();

    MatrixXd dA = MatrixXd::Random(A.size(), nq).eval();
    MatrixXd dB = MatrixXd::Random(B.size(), nq).eval();
//    auto dA = Matrix<double, A.SizeAtCompileTime, nq>::Random().eval();
//    auto dB = Matrix<double, B.SizeAtCompileTime, nq>::Random().eval();

//    //    MatrixXd A = MatrixXd::Random(8, 6).eval();
//    //    MatrixXd B = MatrixXd::Random(A.cols(), 9).eval();
//        auto A = Matrix<double, 5, 3>::Random().eval();
//        auto B = Matrix<double, A.ColsAtCompileTime, 2>::Random().eval();
//
//    //    MatrixXd dA = MatrixXd::Random(A.size(), nq).eval();
//    //    MatrixXd dB = MatrixXd::Random(B.size(), nq).eval();
//        auto dA = Matrix<double, A.SizeAtCompileTime, nq>::Random().eval();
//        auto dB = Matrix<double, B.SizeAtCompileTime, nq>::Random().eval();

    auto dAB = matGradMultMat(A, B, dA, dB).eval();
    volatile auto vol = dAB; // volatile to make sure that the result doesn't get discarded in compiler optimization

    if (check) {
      auto dAB_check = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(B.cols(), B.cols()), A) * dB
          + Eigen::kroneckerProduct(B.transpose(), Eigen::MatrixXd::Identity(A.rows(), A.rows())) * dA;

      if (!dAB.isApprox(dAB_check, 1e-10)) {
        throw std::runtime_error("wrong.");
      }
    }
  }
}

void testMatGradMult(int ntests, bool check) {
  const int nq = 34;
  const int A_rows = 8;
  const int A_cols = 6;
  for (int testnr = 0; testnr < ntests; testnr++) {
//    MatrixXd dA = MatrixXd::Random(A_rows * A_cols, nq).eval();
    auto dA = Matrix<double, A_rows * A_cols, nq>::Random().eval();
    auto b = Matrix<double, A_cols, 1>::Random().eval();
    auto dAb = matGradMult(dA, b).eval();

    if (check) {
      auto A = Matrix<double, A_rows, A_cols>::Random().eval();
      auto db = Matrix<double, b.RowsAtCompileTime, nq>::Zero(b.rows(), nq).eval();
      auto dAb_check = matGradMultMat(A, b, dA, db);
//      std::cout << dAb << std::endl << std::endl;
//      std::cout << dAb_check << std::endl << std::endl;

      if (!dAb.isApprox(dAb_check, 1e-10)) {
        throw std::runtime_error("wrong.");
      }
    }
  }
}

void testGetSubMatrixGradient(int ntests) {
  const int A_rows = 4;
  const int A_cols = 4;
  const int nq = 34;

  std::vector<int> rows {0, 1, 2};
  std::vector<int> cols {0, 1, 2};

//  std::array<int, 3> rows {0, 1, 2};
//  std::array<int, 3> cols {0, 1, 2};

  for (int testnr = 0; testnr < ntests; testnr++) {
    Matrix<double, A_rows * A_cols, nq> dA = Matrix<double, A_rows * A_cols, nq>::Random().eval();
//    Matrix<double, A_rows * A_cols, Dynamic> dA = Matrix<double, A_rows * A_cols, Dynamic>::Random(A_rows * A_cols, nq);

    auto dA_submatrix = getSubMatrixGradient(dA, rows, cols, A_rows, 1, 2);
    volatile auto vol = dA_submatrix.eval();
//    std::cout << "dA:\n" << dA << "\n\n";
//    std::cout << "dA_submatrix:\n" << dA_submatrix << "\n\n";
  }
}

void testSetSubMatrixGradient(int ntests, bool check) {
  const int A_rows = 4;
  const int A_cols = 4;
  const int nq = 34;

  std::vector<int> rows {0, 1, 2};
  std::vector<int> cols {0, 1, 2};

  int q_start = 2;
  int q_subvector_size = 6;
  MatrixXd dA_submatrix = MatrixXd::Random(rows.size(), cols.size());

  for (int testnr = 0; testnr < ntests; testnr++) {
    Matrix<double, A_rows * A_cols, nq> dA = Matrix<double, A_rows * A_cols, nq>::Random().eval();
    setSubMatrixGradient(dA, dA_submatrix, rows, cols, A_rows, q_start, q_subvector_size);

    if (check) {
      auto dA_submatrix_back = getSubMatrixGradient(dA, rows, cols, A_rows, q_start, q_subvector_size);
      if (!dA_submatrix_back.isApprox(dA_submatrix, 1e-10)) {
//        std::cout << "dA_submatrix" << dA_submatrix << std::endl << std::endl;
//        std::cout << "dA_submatrix_back" << dA_submatrix_back << std::endl << std::endl;
        throw std::runtime_error("wrong.");
      }
    }
  }
}

void testNormalizeVec(int ntests) {
  const int x_rows = 4;

  for (int testnr = 0; testnr < ntests; testnr++) {
    auto x = Matrix<double, x_rows, 1>::Random().eval();
    Matrix<double, x_rows, 1> x_norm;
    Matrix<double, x_rows, x_rows> dx_norm;
    Matrix<double, x_rows * x_rows, x_rows> ddx_norm;
    normalizeVec(x, x_norm, &dx_norm, &ddx_norm);

    volatile auto volx_norm = x_norm;
    volatile auto voldx_norm = dx_norm;
    volatile auto volddx_norm = ddx_norm;

//    std::cout << "x_norm:\n" << x_norm << std::endl << std::endl;
//    std::cout << "dx_norm:\n" << dx_norm << std::endl << std::endl;
//    std::cout << "ddx_norm:\n" << ddx_norm << std::endl << std::endl;
  }
}

int main(int argc, char **argv) {
  std::cout << "testTransposeGrad elapsed time: " << measure<>::execution(testTransposeGrad, 100000) << std::endl;

  std::cout << "testMatGradMultMat elapsed time: " << measure<>::execution(testMatGradMultMat, 100000, false) << std::endl;
  testMatGradMultMat(1000, true);

  std::cout << "testMatGradMult elapsed time: " << measure<>::execution(testMatGradMult, 100000, false) << std::endl;
  testMatGradMult(1000, true);

  std::cout << "testGetSubMatrixGradient elapsed time: " << measure<>::execution(testGetSubMatrixGradient, 100000) << std::endl;

  std::cout << "testSetSubMatrixGradient elapsed time: " << measure<>::execution(testSetSubMatrixGradient, 100000, false) << std::endl;
  testSetSubMatrixGradient(1000, true);

  std::cout << "testNormalizeVec elapsed time: " << measure<>::execution(testNormalizeVec, 100000) << std::endl;

  return 0;
}
