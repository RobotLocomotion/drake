#include "drake/solvers/semidefinite_relaxation_internal.h"

#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/matrix_util.h"

namespace drake {
namespace solvers {
namespace internal {

GTEST_TEST(MakeSemidefiniteRelaxationInternalTest, TestSparseKron) {
  Eigen::MatrixXd A(3, 3);
  // clang-format off
  A <<  1.77, -4.38, -3.63,
       -2.03, -0.57,  4.56,
       -1.66,  2.15, 10.02;
  // clang-format on
  Eigen::MatrixXd B(2, 2);
  // clang-format off
  B << 7.16, 1.8,
       2.39, 5.77;
  // clang-format on

  Eigen::SparseMatrix<double> C(7, 13);
  std::vector<Eigen::Triplet<double>> C_triplets;
  C_triplets.emplace_back(0, 9, 1.77);
  C_triplets.emplace_back(1, 5, -0.57);
  C_triplets.emplace_back(2, 3, 4.56);
  C_triplets.emplace_back(3, 12, -1.66);
  C_triplets.emplace_back(4, 7, 2.15);
  C_triplets.emplace_back(5, 6, 10.02);
  C_triplets.emplace_back(0, 1, -0.45);
  C.setFromTriplets(C_triplets.begin(), C_triplets.end());

  auto TestKron = [](const Eigen::SparseMatrix<double>& A_test,
                     const Eigen::SparseMatrix<double>& B_test) {
    // AXB = (B.T ⊗ A)vec(X) so we compute this as a numerical sanity check.
    const Eigen::SparseMatrix<double> M1 = B_test.transpose();
    const Eigen::SparseMatrix<double> M2 = A_test;

    Eigen::SparseMatrix<double> kron = SparseKroneckerProduct(M1, M2);
    EXPECT_EQ(kron.rows(), M1.rows() * M2.rows());
    EXPECT_EQ(kron.cols(), M1.cols() * M2.cols());
    EXPECT_EQ(kron.nonZeros(), M1.nonZeros() * M2.nonZeros());

    Eigen::MatrixXd testMatrix(A_test.cols(), B_test.rows());
    for (int i = 0; i < testMatrix.rows(); ++i) {
      for (int j = 0; j < testMatrix.cols(); ++j) {
        // put arbitrary values in testMatrix
        testMatrix(i, j) = 2 * (i + j) / (i + j + 1);
      }
    }
    Eigen::MatrixXd AXB = A_test * testMatrix * B_test;
    Eigen::VectorXd kron_vec = kron * Eigen::Map<const Eigen::VectorXd>(
                                          testMatrix.data(), testMatrix.size());
    EXPECT_TRUE(CompareMatrices(
        Eigen::Map<const Eigen::VectorXd>(AXB.data(), AXB.size()), kron_vec,
        1e-10, MatrixCompareType::absolute));
  };

  TestKron(A.sparseView(), B.sparseView());
  TestKron(B.sparseView(), A.sparseView());
  TestKron(C, A.sparseView());
  TestKron(B.sparseView(), C);
  TestKron(C, B.sparseView());
}

GTEST_TEST(MakeSemidefiniteRelaxationInternalTest,
           TestComputeTensorProductOfSymmetricMatrixToRealVecOperators) {
  const int n = 2;
  const int tril_size = n * (n + 1) / 2;
  Eigen::MatrixXd X(n, n);
  // clang-format off
  X <<   4.1 , -2.11,
        -2.11, -3.59;
  // clang-format on
  Eigen::VectorX<double> x_tril = math::ToLowerTriangularColumnsFromMatrix(X);

  Eigen::VectorXd a_full(n);
  a_full << 1.69, -1.11;
  Eigen::MatrixXd b_full(n, 2);
  // clang-format off
  b_full <<  -3.34, -7.19,
             -4.02, -2.45;
  // clang-format on

  Eigen::MatrixXd a_tril = Eigen::MatrixXd::Zero(n, tril_size);
  Eigen::MatrixXd b_tril = Eigen::MatrixXd::Zero(2 * n, tril_size);
  int start_col = 0;
  for (int i = 0; i < n; ++i) {
    a_tril.block(i, start_col, n - i, n - i) =
        a_full(i) * Eigen::MatrixXd(n - i, n - i).setIdentity();
    a_tril.block(i, start_col, 1, n - i) = a_full.tail(n - i).transpose();

    b_tril.block(i, start_col, n - i, n - i) =
        b_full(i, 0) * Eigen::MatrixXd(n - i, n - i).setIdentity();
    b_tril.block(i, start_col, 1, n - i) =
        b_full.col(0).tail(n - i).transpose();

    b_tril.block(i + n, start_col, n - i, n - i) =
        b_full(i, 1) * Eigen::MatrixXd(n - i, n - i).setIdentity();
    b_tril.block(n + i, start_col, 1, n - i) =
        b_full.col(1).tail(n - i).transpose();

    start_col += n - i;
  }

  // Ensure that a_tril * x_tril == X * a_full.
  Eigen::VectorXd X_times_a_full = X * a_full;
  EXPECT_TRUE(CompareMatrices(X * a_full, a_tril * x_tril, 1e-10));
  // Ensure that b_tril * x_tril == (X * b_tril).flatten().
  Eigen::MatrixXd X_time_b_full = X * b_full;
  EXPECT_TRUE(CompareMatrices(
      Eigen::Map<Eigen::VectorXd>(X_time_b_full.data(), X_time_b_full.size()),
      b_tril * x_tril, 1e-10));

  Eigen::MatrixXd a_full_kron_b_full =
      SparseKroneckerProduct(a_full.sparseView(), b_full.sparseView());
  Eigen::MatrixXd a_tril_tensor_b_tril =
      ComputeTensorProductOfSymmetricMatrixToRealVecOperators(
          a_tril.sparseView(), b_tril.sparseView());

  // A test matrix in the tensor space
  Eigen::MatrixXd X_kron(n * n, n * n);
  // clang-format off
  for(int i=0; i<X_kron.rows(); ++i){
    for(int j=i; j<X_kron.cols(); ++j) {
      // Some arbitrary values.
      X_kron(i,j) = std::pow(-1,2*i+3*j)*(i+7*j)/std::log(2*i+j+7);
    }
  }
  // clang-format on
  Eigen::VectorX<double> x_kron_tril =
      math::ToLowerTriangularColumnsFromMatrix(X_kron);

  // Check that X_kron * a_full_kron_b_full == a_tril_tensor_b_tril *
  // x_kron_tril.
  std::cout << fmt::format("a_full_kron_b_full size = ({}, {})",
                           a_full_kron_b_full.rows(), a_full_kron_b_full.cols())
            << std::endl;
  std::cout << fmt::format("a_tril_tensor_b_tril = ({}, {}) ",
                           a_tril_tensor_b_tril.rows(),
                           a_tril_tensor_b_tril.cols())
            << std::endl;
  std::cout << fmt::format("a_full_kron_b_full:\n{}\n",
                           fmt_eigen(a_full_kron_b_full))
            << std::endl;
  std::cout << fmt::format("a_tril:\n{}\n",
                           fmt_eigen(a_tril))
            << std::endl;
  std::cout << fmt::format("b_tril:\n{}\n",
                           fmt_eigen(b_tril))
            << std::endl;
  std::cout << fmt::format("a_tril_tensor_b_tril:\n{}\n",
                           fmt_eigen(a_tril_tensor_b_tril))
            << std::endl;
  Eigen::MatrixXd test_product = X_kron * a_full_kron_b_full;
  EXPECT_TRUE(CompareMatrices(
      Eigen::Map<Eigen::VectorXd>(test_product.data(), test_product.size()),
      a_tril_tensor_b_tril * x_kron_tril, 1e-10));
}
//
// GTEST_TEST(MakeSemidefiniteRelaxationInternalTest,
//           TestComputeTensorProductOfSymmetricMatrixToRealVecOperators) {
//  Eigen::MatrixXd X(5, 5);
//  // clang-format off
//  X <<   4.1 , -2.11, -2.07, -5.68, -5.95,
//        -2.11, -3.59, -2.84, -1.73, -8.44,
//        -2.07, -2.84, -8.84, -5.65,  8.24,
//        -5.68, -1.73, -5.65, -1.98,  0.48,
//        -5.95, -8.44,  8.24,  0.48,  4.23;
//  // clang-format on
//  Eigen::VectorX<double> x_tril = math::ToLowerTriangularColumnsFromMatrix(X);
//
//  Eigen::VectorXd a_full(5);
//  a_full << 1.69, -1.11, 2.6, -3.81, 0.83;
//  Eigen::MatrixXd b_full(5, 2);
//  // clang-format off
//  b_full <<  -3.34, -7.19,
//             -4.02, -2.45,
//             -0.74,  0.55,
//              1.65, -1.23,
//             -2.98, -0.73;
//  // clang-format on
//
//  Eigen::MatrixXd a_tril = Eigen::MatrixXd::Zero(5, 15);
//  Eigen::MatrixXd b_tril = Eigen::MatrixXd::Zero(10, 15);
//  int start_col = 0;
//  for (int i = 0; i < 5; ++i) {
//    a_tril.block(i, start_col, 5 - i, 5 - i) =
//        a_full(i) * Eigen::MatrixXd(5 - i, 5 - i).setIdentity();
//    a_tril.block(i, start_col, 1, 5 - i) = a_full.tail(5 - i).transpose();
//
//    b_tril.block(i, start_col, 5 - i, 5 - i) =
//        b_full(i, 0) * Eigen::MatrixXd(5 - i, 5 - i).setIdentity();
//    b_tril.block(i, start_col, 1, 5 - i) =
//        b_full.col(0).tail(5 - i).transpose();
//
//    b_tril.block(i + 5, start_col, 5 - i, 5 - i) =
//        b_full(i, 1) * Eigen::MatrixXd(5 - i, 5 - i).setIdentity();
//    b_tril.block(5 + i, start_col, 1, 5 - i) =
//        b_full.col(1).tail(5 - i).transpose();
//
//    start_col += 5 - i;
//  }
//
//  // Ensure that a_tril * x_tril == X * a_full.
//  Eigen::VectorXd X_times_a_full = X * a_full;
//  EXPECT_TRUE(CompareMatrices(X * a_full, a_tril * x_tril, 1e-10));
//  // Ensure that b_tril * x_tril == (X * b_tril).flatten().
//  Eigen::MatrixXd X_time_b_full = X * b_full;
//  EXPECT_TRUE(CompareMatrices(
//      Eigen::Map<Eigen::VectorXd>(X_time_b_full.data(), X_time_b_full.size()),
//      b_tril * x_tril, 1e-10));
//
//  Eigen::MatrixXd a_full_kron_b_full =
//      SparseKroneckerProduct(a_full.sparseView(), b_full.sparseView());
//  Eigen::MatrixXd a_tril_tensor_b_tril =
//      ComputeTensorProductOfSymmetricMatrixToRealVecOperators(
//          a_tril.sparseView(), b_tril.sparseView());
//
//  // A test matrix in the tensor space
//  Eigen::MatrixXd X_kron(25, 25);
//  // clang-format off
//  for(int i=0; i<X_kron.rows(); ++i){
//    for(int j=i; j<X_kron.cols(); ++j) {
//      // Some arbitrary values.
//      X_kron(i,j) = std::pow(-1,2*i+3*j)*(i+7*j)/std::log(2*i+j+7);
//    }
//  }
//  // clang-format on
//  Eigen::VectorX<double> x_kron_tril =
//      math::ToLowerTriangularColumnsFromMatrix(X_kron);
//
//  // Check that X_kron * a_full_kron_b_full == a_tril_tensor_b_tril *
//  // x_kron_tril.
//  std::cout << fmt::format("a_full_kron_b_full size = ({}, {})",
//  a_full_kron_b_full.rows(), a_full_kron_b_full.cols()) << std::endl;
//  std::cout << fmt::format("a_tril_tensor_b_tril = ({}, {}) ",
//  a_tril_tensor_b_tril.rows(), a_tril_tensor_b_tril.cols()) << std::endl;
////  std::cout << fmt::format("a_full_kron_b_full:\n{}\n",
/// fmt_eigen(a_full_kron_b_full)) << std::endl; /  std::cout <<
/// fmt::format("a_tril_tensor_b_tril:\n{}\n", fmt_eigen(a_tril_tensor_b_tril))
///<< std::endl;
// Eigen::MatrixXd test_product = X_kron * a_full_kron_b_full;
// EXPECT_TRUE(CompareMatrices(Eigen::Map<Eigen::VectorXd>(test_product.data(),
// test_product.size()),
//                              a_tril_tensor_b_tril * x_kron_tril, 1e-10));
//}

GTEST_TEST(MakeSemidefiniteRelaxationInternalTest, TestWAdj) {
  Eigen::MatrixXd Y(5, 5);
  // clang-format off
  Y << -3.08, -0.84,  0.32,  0.54,  0.51,
       -0.84,  2.6 ,  1.72,  0.09,  0.79,
        0.32,  1.72,  3.09,  1.19,  0.31,
        0.54,  0.09,  1.19, -0.37, -2.57,
        0.51,  0.79,  0.31, -2.57, -0.08;
  // clang-format on
  // The lower triangular part of Y has 15 entries
  Eigen::VectorXd y_tril(15);
  y_tril = math::ToLowerTriangularColumnsFromMatrix(Y);

  Eigen::SparseMatrix<double> W_adj = GetWAdjForTril(6);
  EXPECT_EQ(W_adj.rows(), 6);
  EXPECT_EQ(W_adj.cols(), y_tril.size());
  EXPECT_EQ(W_adj.nonZeros(), 14 /* 2 * (6-1) + 4 */);

  Eigen::VectorXd result(6);
  result = W_adj * y_tril;
  const double tol{1e-10};
  EXPECT_NEAR(result(0), Y.trace(), tol);
  EXPECT_NEAR(result(1), Y(0, 0) - Y.block(1, 1, 4, 4).trace(), tol);
  for (int i = 2; i < W_adj.rows(); ++i) {
    EXPECT_NEAR(result(i), 2 * Y(0, i - 1), tol);
  }
}

GTEST_TEST(MakeSemidefiniteRelaxationInternalTest, TestSkewAdjoint) {
  Eigen::MatrixXd Y(5, 5);
  // clang-format off
  Y <<  0.  , -2.29, -0.11, -1.17,  2.48,
        2.29,  0.  , -2.71, -3.52, -1.17,
        0.11,  2.71,  0.  , -1.1 ,  3.07,
        1.17,  3.52,  1.1 ,  0.  , -1.6 ,
       -2.48,  1.17, -3.07,  1.6 ,  0.  ;
  // clang-format on

  Eigen::VectorXd neg_two_y(10);
  neg_two_y << -2 * Y(1, 0), -2 * Y(2, 0), -2 * Y(3, 0), -2 * Y(4, 0),
      -2 * Y(2, 1), -2 * Y(3, 1), -2 * Y(4, 1), -2 * Y(3, 2), -2 * Y(4, 2),
      -2 * Y(4, 3);

  Eigen::SparseMatrix<double> W = GetSkewAdjointForLowerTri(5);
  Eigen::VectorXd result = W * math::ToLowerTriangularColumnsFromMatrix(Y);
  const double tol{1e-10};
  EXPECT_TRUE(
      CompareMatrices(result, neg_two_y, tol, MatrixCompareType::absolute));
}

// GTEST_TEST(MakeSemidefiniteRelaxationInternalTest,
//           AddMatrixIsLorentzSeparableConstraint3by4) {
//  MathematicalProgram prog;
//  const int m = 3;
//  const int n = 4;
//
//  auto X = prog.NewContinuousVariables(3, 4, "X");
//  AddMatrixIsLorentzSeparableConstraint(X, &prog);
//
//  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 1);
//  EXPECT_EQ(ssize(prog.linear_equality_constraints()), 2);
//  EXPECT_EQ(ssize(prog.GetAllConstraints()), 3);
//
//  const Binding<PositiveSemidefiniteConstraint> psd_constraint =
//      prog.positive_semidefinite_constraints()[0];
//  EXPECT_EQ(psd_constraint.evaluator()->matrix_rows(), (m - 1) * (n - 1));
//
//  const Binding<LinearEqualityConstraint> X_equal_Wadj_Y_constraint =
//  prog.linear_equality_constraints()[0]; const
//  Binding<LinearEqualityConstraint> skewAdj_Y_constraint =
//  prog.linear_equality_constraints()[1];
//
//  EXPECT_TRUE(false);
//}

}  // namespace internal
}  // namespace solvers
}  // namespace drake