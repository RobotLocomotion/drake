#include "drakeGradientUtil.h"
#include <Eigen/Core>
#include <iostream>
#include <chrono>
#include <typeinfo>


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

void testMatGradMultMat(int ntests)
{
  int nq = 34;
  for (int testnr = 0; testnr < ntests; testnr++) {
    MatrixXd A = MatrixXd::Random(8, 6);
    MatrixXd B = MatrixXd::Random(A.cols(), 9);
    MatrixXd dA = MatrixXd::Random(A.size(), nq);
    MatrixXd dB = MatrixXd::Random(B.size(), nq);
    auto dAB = matGradMultMat(A, B, dA, dB); // volatile to make sure that the result doesn't get discarded in compiler optimization
//    std::cout << "size of dAB: " << dAB.rows() << "x" << dAB.cols() << std::endl;
    volatile auto vol = dAB;
  }
}

int main(int argc, char **argv) {
//  std::cout << "elapsed time: " << measure<>::execution(testTransposeGrad, 500000) << std::endl;
  std::cout << "elapsed time: " << measure<>::execution(testMatGradMultMat, 100000) << std::endl;
  return 0;
}
