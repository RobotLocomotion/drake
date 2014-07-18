#include "drakeGradientUtil.h"
#include <Eigen/Core>
#include <Eigen/KroneckerProduct> // unsupported...
#include <iostream>
#include <chrono>
#include <stdexcept>

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
  const int rows_X = 6;
  const int cols_X = 15;
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
    auto dX_transpose = transposeGrad(dX, rows_X); // volatile to make sure that the result doesn't get discarded in compiler optimization
    volatile auto vol = dX_transpose;

//    transposeGrad(dX, rows_X, dX_transpose);
//    std::cout << "dX_transpose:\n" << dX_transpose << std::endl;
  }
}

void testMatGradMultMat(int ntests, bool check)
{
  const int nq = 5; // 34
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

    auto dAB = matGradMultMat(A, B, dA, dB).eval(); // volatile to make sure that the result doesn't get discarded in compiler optimization
    volatile auto vol = dAB;

    if (check) {
      auto dABCheck = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(B.cols(), B.cols()), A) * dB
          + Eigen::kroneckerProduct(B.transpose(), Eigen::MatrixXd::Identity(A.rows(), A.rows())) * dA;

      if (!dAB.isApprox(dABCheck, 1e-10)) {
        throw std::runtime_error("Wrong.");
      }
    }
  }
}

int main(int argc, char **argv) {
//  std::cout << "elapsed time: " << measure<>::execution(testTransposeGrad, 500000) << std::endl;
  std::cout << "elapsed time: " << measure<>::execution(testMatGradMultMat, 100000, false) << std::endl;
  testMatGradMultMat(1000, true);
  return 0;
}
