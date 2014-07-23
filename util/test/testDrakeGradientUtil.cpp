#include "drakeGradientUtil.h"
#include "drakeQuatUtil.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/KroneckerProduct> // unsupported...
#include <iostream>
#include <chrono>
#include <stdexcept>
#include <vector>
#include <array>
#include <random>

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
//  MatrixXd dX(numel_X, nq);

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
    //
    //    MatrixXd A = MatrixXd::Random(8, 6).eval();
    //    MatrixXd B = MatrixXd::Random(A.cols(), 9).eval();
    //    auto A = Matrix<double, 5, 3>::Random().eval();
    //    auto B = Matrix<double, A.ColsAtCompileTime, 2>::Random().eval();
    //
    //    MatrixXd dA = MatrixXd::Random(A.size(), nq).eval();
    //    MatrixXd dB = MatrixXd::Random(B.size(), nq).eval();
    //    auto dA = Matrix<double, A.SizeAtCompileTime, nq>::Random().eval();
    //    auto dB = Matrix<double, B.SizeAtCompileTime, nq>::Random().eval();

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

void testDHomogTrans(int ntests) {
  Isometry3d T;
  std::default_random_engine generator;

  for (int testnr = 0; testnr < ntests; testnr++) {
    T = uniformlyRandomQuat(generator);
    //  T.setIdentity();
    //  T = AngleAxisd(M_PI_2, Vector3d(1.0, 0.0, 0.0));

    const int nv = 6;
    const int nq = 7;

    auto S = Matrix<double, 6, nv>::Random().eval();
//    setLinearIndices(S);
//    S.setIdentity();
//    std::cout << S << "\n\n";

    auto qdot_to_v = Matrix<double, nv, nq>::Random().eval();
//    setLinearIndices(qdot_to_v);
//    std::cout << qdot_to_v << "\n\n";

    auto dT = dHomogTrans(T, S, qdot_to_v).eval();
    volatile auto vol = dT;
    //  std::cout << dT << std::endl << std::endl;
  }
}

void testDHomogTransInv(int ntests, bool check) {
  Isometry3d T;
  std::default_random_engine generator;
  for (int testnr = 0; testnr < ntests; testnr++) {
    T = uniformlyRandomQuat(generator) * Translation3d(Vector3d::Random());

    const int nv = 6;
    const int nq = 7;

    auto S = Matrix<double, 6, nv>::Random().eval();
    auto qdot_to_v = Matrix<double, nv, nq>::Random().eval();

    auto dT = dHomogTrans(T, S, qdot_to_v).eval();
    auto dTInv = dHomogTransInv(T, dT);
    volatile auto vol = dTInv;

    if (check) {
      auto dTInvInv = dHomogTransInv(T.inverse(), dTInv);

      if (!dT.matrix().isApprox(dTInvInv.matrix(), 1e-10)) {
        std::cout << "dTInv:\n" << dTInv << "\n\n";
        std::cout << "dT:\n" << dT << "\n\n";
        std::cout << "dTInvInv:\n" << dTInvInv << "\n\n";
        std::cout << "dTInvInv - dT:\n" << dTInvInv - dT << "\n\n";

        throw std::runtime_error("wrong");
      }
    }
  }
}

void testDTransformAdjoint(int ntests, bool debug) {
  const int nv = 6;
  const int nq = 34;
  const int cols_X = 3;

  Isometry3d T;
  std::default_random_engine generator;

  if (debug) {
    //    T.setIdentity();
    T = Translation3d(Vector3d(1.0, 2.0, 3.0)) * AngleAxisd(M_PI_2, Vector3d(1.0, 0.0, 0.0));
    std::cout << "T:\n" << T.matrix() << "\n\n";

    auto S = Matrix<double, 6, nv>::Random().eval();
    setLinearIndices(S);

    auto qdot_to_v = Matrix<double, nv, nq>::Random().eval();
    setLinearIndices(qdot_to_v);

    auto dT = dHomogTrans(T, S, qdot_to_v).eval();
    std::cout << "dT:\n" << dT << "\n\n";

    auto X = Matrix<double, 6, cols_X>::Random().eval();
    setLinearIndices(X);
    std::cout << "X:\n" << X << "\n\n";

    auto dX = MatrixXd::Random(X.size(), nq).eval();
    setLinearIndices(dX);
    std::cout << "dX:\n" << dX << "\n\n";

    auto dAdT_times_X = dTransformAdjoint(T, X, dT, dX).eval();
    std::cout << "dAdT_times_X:\n" << dAdT_times_X << "\n\n";
  }
  else {
    for (int testnr = 0; testnr < ntests; testnr++) {
      T = uniformlyRandomQuat(generator) * Translation3d(Vector3d::Random());
      auto S = Matrix<double, 6, nv>::Random().eval();
      auto qdot_to_v = Matrix<double, nv, nq>::Random().eval();
      auto dT = dHomogTrans(T, S, qdot_to_v).eval();
      auto X = Matrix<double, 6, cols_X>::Random().eval();
      auto dX = MatrixXd::Random(X.size(), nq).eval();
//      auto dX = Matrix<double, X.SizeAtCompileTime, nq>::Random().eval();
      auto dAdT_times_X = dTransformAdjoint(T, X, dT, dX).eval();
      volatile auto vol = dAdT_times_X;
    }
  }
}

void testDTransformAdjointTranspose(int ntests, bool debug) {
  const int nv = 6;
  const int nq = 34;
  const int cols_X = 3;

  Isometry3d T;
  std::default_random_engine generator;

  if (debug) {
    //    T.setIdentity();
    T = Translation3d(Vector3d(1.0, 2.0, 3.0)) * AngleAxisd(M_PI_2, Vector3d(1.0, 0.0, 0.0));
    std::cout << "T:\n" << T.matrix() << "\n\n";

    auto S = Matrix<double, 6, nv>::Random().eval();
    setLinearIndices(S);

    auto qdot_to_v = Matrix<double, nv, nq>::Random().eval();
    setLinearIndices(qdot_to_v);

    auto dT = dHomogTrans(T, S, qdot_to_v).eval();
    std::cout << "dT:\n" << dT << "\n\n";

    auto X = Matrix<double, 6, cols_X>::Random().eval();
    setLinearIndices(X);
    std::cout << "X:\n" << X << "\n\n";

    auto dX = MatrixXd::Random(X.size(), nq).eval();
    setLinearIndices(dX);
    std::cout << "dX:\n" << dX << "\n\n";

    auto dAdTtranspose_times_X = dTransformAdjointTranspose(T, X, dT, dX).eval();
    std::cout << "dAdTtranspose_times_X:\n" << dAdTtranspose_times_X << "\n\n";
  }
  else {
    for (int testnr = 0; testnr < ntests; testnr++) {
      T = uniformlyRandomQuat(generator) * Translation3d(Vector3d::Random());
      auto S = Matrix<double, 6, nv>::Random().eval();
      auto qdot_to_v = Matrix<double, nv, nq>::Random().eval();
      auto dT = dHomogTrans(T, S, qdot_to_v).eval();
      auto X = Matrix<double, 6, cols_X>::Random().eval();
      auto dX = MatrixXd::Random(X.size(), nq).eval();
//      auto dX = Matrix<double, X.SizeAtCompileTime, nq>::Random().eval();
      auto dAdTtranspose_times_X = dTransformAdjointTranspose(T, X, dT, dX).eval();
      volatile auto vol = dAdTtranspose_times_X;
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
//    std::cout << "gradientNumRows: " << gradientNumRows(x_rows, x_rows, 1) << std::endl;

    volatile auto volx_norm = x_norm;
    volatile auto voldx_norm = dx_norm;
    volatile auto volddx_norm = ddx_norm;

//    std::cout << "x_norm:\n" << x_norm << std::endl << std::endl;
//    std::cout << "dx_norm:\n" << dx_norm << std::endl << std::endl;
//    std::cout << "ddx_norm:\n" << ddx_norm << std::endl << std::endl;
  }
}

int main(int argc, char **argv) {
  int ntests = 100000;
  std::cout << "testTransposeGrad elapsed time: " << measure<>::execution(testTransposeGrad, ntests) << std::endl;

  std::cout << "testMatGradMultMat elapsed time: " << measure<>::execution(testMatGradMultMat, ntests, false) << std::endl;
  testMatGradMultMat(1000, true);

  std::cout << "testMatGradMult elapsed time: " << measure<>::execution(testMatGradMult, ntests, false) << std::endl;
  testMatGradMult(1000, true);

  std::cout << "testGetSubMatrixGradient elapsed time: " << measure<>::execution(testGetSubMatrixGradient, ntests) << std::endl;

  std::cout << "testSetSubMatrixGradient elapsed time: " << measure<>::execution(testSetSubMatrixGradient, ntests, false) << std::endl;
  testSetSubMatrixGradient(1000, true);

  std::cout << "testDHomogTrans elapsed time: " << measure<>::execution(testDHomogTrans, ntests) << std::endl;

  std::cout << "testDHomogTransInv elapsed time: " << measure<>::execution(testDHomogTransInv, ntests, false) << std::endl;
  testDHomogTransInv(1000, true);

  std::cout << "testDTransformAdjoint elapsed time: " << measure<>::execution(testDTransformAdjoint, ntests, false) << std::endl;

  std::cout << "testDTransformAdjointTranspose elapsed time: " << measure<>::execution(testDTransformAdjointTranspose, ntests, false) << std::endl;

  std::cout << "testNormalizeVec elapsed time: " << measure<>::execution(testNormalizeVec, ntests) << std::endl;


  return 0;
}
