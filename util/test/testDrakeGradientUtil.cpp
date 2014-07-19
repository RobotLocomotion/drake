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

void testTransposeGrad(int ntests)
{
  const int rows_X = 8;
  const int cols_X = 6;
  const int numel_X = rows_X * cols_X;
  const int nq = 34;
//  Matrix<double, numel_X, nq> dX;
////  MatrixXd dX(numel_X, nq);
//  for (int col = 0; col < nq; col++) {
//    for (int row = 0; row < numel_X; row++) {
//      dX(row, col) = row + col * numel_X + 1;
//    }
//  }

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

void testSubMatrixGradient(int ntests) {
  const int A_rows = 4;
  const int A_cols = 4;
  const int nq = 34;

//  std::vector<int> rows {0, 1, 2};
//  std::vector<int> cols {0, 1, 2};

  std::array<int, 3> rows {0, 1, 2};
  std::array<int, 3> cols {0, 1, 2};

  for (int testnr = 0; testnr < ntests; testnr++) {
    Matrix<double, A_rows * A_cols, nq> dA = Matrix<double, A_rows * A_cols, nq>::Random().eval();
//    Matrix<double, A_rows * A_cols, Dynamic> dA = Matrix<double, A_rows * A_cols, Dynamic>::Random(A_rows * A_cols, nq);
//    const int numel_A = A_rows * A_cols;
    //  for (int col = 0; col < nq; col++) {
    //    for (int row = 0; row < numel_A; row++) {
    //      dA(row, col) = row + col * numel_A + 1;
    //    }
    //  }

    auto dA_submatrix = getSubMatrixGradient(dA, rows, cols, A_rows);
    volatile auto vol = dA_submatrix.eval();
  }
//  std::cout << "dA:\n" << dA << "\n\n";
//  std::cout << "dA_submatrix:\n" << dA_submatrix << "\n\n";

}

void testNormalizeVec() {
  const int x_rows = 3;

  Matrix<double, x_rows, 1> x;
  x << 1.0, 2.0, 3.0;

  Matrix<double, x_rows, 1> x_norm;
  MatrixXd dx_norm(x_norm.size(), x_rows);
  MatrixXd ddx_norm(dx_norm.size(), x_rows);
  normalizeVec(x, x_norm, &dx_norm, &ddx_norm);
  std::cout << "x_norm: " << x_norm << std::endl << std::endl;
  std::cout << "dx_norm: " << dx_norm << std::endl << std::endl;
  std::cout << "ddx_norm: " << ddx_norm << std::endl << std::endl;
}

int main(int argc, char **argv) {
//  std::cout << "testTransposeGrad elapsed time: " << measure<>::execution(testTransposeGrad, 100000) << std::endl;
//
//  std::cout << "testMatGradMultMat elapsed time: " << measure<>::execution(testMatGradMultMat, 100000, false) << std::endl;
//  testMatGradMultMat(1000, true);
//
//  std::cout << "testMatGradMult elapsed time: " << measure<>::execution(testMatGradMult, 100000, false) << std::endl;
//  testMatGradMult(1000, true);
//
//  testNormalizeVec();

  std::cout << "testSubMatrixGradient elapsed time: " << measure<>::execution(testSubMatrixGradient, 100000) << std::endl;

  return 0;
}
