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

GTEST_TEST(MakeSemidefiniteRelaxationInternalTest,
           AddMatrixIsLorentzSeparableConstraint3by4) {
  MathematicalProgram prog;
  const int m = 3;
  const int n = 4;

  auto X = prog.NewContinuousVariables(3, 4, "X");
  AddMatrixIsLorentzSeparableConstraint(X, &prog);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog.linear_equality_constraints()), 2);
  EXPECT_EQ(ssize(prog.GetAllConstraints()), 3);

  const Binding<PositiveSemidefiniteConstraint> psd_constraint =
      prog.positive_semidefinite_constraints()[0];
  EXPECT_EQ(psd_constraint.evaluator()->matrix_rows(), (m - 1) * (n - 1));

  const Binding<LinearEqualityConstraint> X_equal_Wadj_Y_constraint = prog.linear_equality_constraints()[0];
  const Binding<LinearEqualityConstraint> skewAdj_Y_constraint = prog.linear_equality_constraints()[1];

  EXPECT_TRUE(false);
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake