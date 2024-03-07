#include "drake/solvers/semidefinite_relaxation_internal.h"

#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/solve.h"

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
           TestToSymmetricMatrixFromTensorVector) {
  // Check the 2x2 ⊗  3x3 case.
  int num_elts =
      3 * 6;  // There are 3 elements in 2x2 basis and 6 in the 3x3 basis.
  VectorX<symbolic::Variable> y(num_elts);
  for (int i = 0; i < num_elts; ++i) {
    y(i) = symbolic::Variable("y_" + std::to_string(i));
  }
  Eigen::MatrixX<symbolic::Variable> Y =
      ToSymmetricMatrixFromTensorVector(y, 2, 3);
  Eigen::MatrixX<symbolic::Variable> Y_expected(6, 6);
  // clang-format off
  Y_expected << y(0), y(1),  y(2),  y(6),  y(7),  y(8),
                y(1), y(3),  y(4),  y(7),  y(9),  y(10),
                y(2), y(4),  y(5),  y(8),  y(10), y(11),
                y(6), y(7),  y(8),  y(12), y(13), y(14),
                y(7), y(9),  y(10), y(13), y(15), y(16),
                y(8), y(10), y(11), y(14), y(16), y(17);
  // clang-format on
  EXPECT_EQ(Y.rows(), 6);
  EXPECT_EQ(Y.cols(), 6);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_TRUE(Y_expected(i, j).equal_to(Y(i, j)));
    }
  }

  // Check the 4x4 ⊗  3x3 case.
  num_elts = 10 * 6;
  VectorX<symbolic::Variable> x(num_elts);
  for (int i = 0; i < num_elts; ++i) {
    x(i) = symbolic::Variable(fmt::format("x({}),", std::to_string(i)));
  }
  Eigen::MatrixX<symbolic::Variable> X =
      ToSymmetricMatrixFromTensorVector(x, 4, 3);
  Eigen::MatrixX<symbolic::Variable> X_expected(12, 12);
  // The following was checked by hand. It may seem big, but it is necessary to
  // check something this large.
  // clang-format off
  // NOLINT
  X_expected <<  x(0),  x(1),  x(2),  x(6),  x(7),  x(8), x(12), x(13), x(14), x(18), x(19), x(20),// NOLINT
                 x(1),  x(3),  x(4),  x(7),  x(9), x(10), x(13), x(15), x(16), x(19), x(21), x(22),// NOLINT
                 x(2),  x(4),  x(5),  x(8), x(10), x(11), x(14), x(16), x(17), x(20), x(22), x(23),// NOLINT
                 x(6),  x(7),  x(8), x(24), x(25), x(26), x(30), x(31), x(32), x(36), x(37), x(38),// NOLINT
                 x(7),  x(9), x(10), x(25), x(27), x(28), x(31), x(33), x(34), x(37), x(39), x(40),// NOLINT
                 x(8), x(10), x(11), x(26), x(28), x(29), x(32), x(34), x(35), x(38), x(40), x(41),// NOLINT
                x(12), x(13), x(14), x(30), x(31), x(32), x(42), x(43), x(44), x(48), x(49), x(50),// NOLINT
                x(13), x(15), x(16), x(31), x(33), x(34), x(43), x(45), x(46), x(49), x(51), x(52),// NOLINT
                x(14), x(16), x(17), x(32), x(34), x(35), x(44), x(46), x(47), x(50), x(52), x(53),// NOLINT
                x(18), x(19), x(20), x(36), x(37), x(38), x(48), x(49), x(50), x(54), x(55), x(56),// NOLINT
                x(19), x(21), x(22), x(37), x(39), x(40), x(49), x(51), x(52), x(55), x(57), x(58),// NOLINT
                x(20), x(22), x(23), x(38), x(40), x(41), x(50), x(52), x(53), x(56), x(58), x(59);// NOLINT
  // clang-format on
  EXPECT_EQ(X.rows(), 12);
  EXPECT_EQ(X.cols(), 12);
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      EXPECT_TRUE(X_expected(i, j).equal_to(X(i, j)));
    }
  }
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

GTEST_TEST(MakeSemidefiniteRelaxationInternalTest,
           AddMatrixIsLorentzSeparableConstraint3by4) {
  MathematicalProgram prog;
  const int m = 3;
  const int n = 4;

  auto X = prog.NewContinuousVariables(m, n, "X");
  AddMatrixIsLorentzSeparableConstraint(X, &prog);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog.linear_equality_constraints()), 1);
  EXPECT_EQ(ssize(prog.GetAllConstraints()), 2);

  const Binding<PositiveSemidefiniteConstraint> psd_constraint =
      prog.positive_semidefinite_constraints()[0];
  EXPECT_EQ(psd_constraint.evaluator()->matrix_rows(), (m - 1) * (n - 1));

  // A vector in the Lorentz cone of size m.
  Eigen::VectorXd xm_lorentz(m);
  xm_lorentz << 4, -1.7, 0.1;
  // A vector in the Lorentz cone of size n.
  Eigen::VectorXd xn_lorentz(n);
  xn_lorentz << 13, -0.1, 5, -2.3;
  Eigen::MatrixXd X_tensor_lorentz = xm_lorentz * xn_lorentz.transpose();
  auto X_equal_lorentz_tensor_constraint =
      prog.AddLinearEqualityConstraint(X == X_tensor_lorentz);
  auto result = Solve(prog);
  // X is required to be equal to a simple lorentz separable tensor therefore
  // this program should be feasible.
  EXPECT_TRUE(result.is_success());

  prog.RemoveConstraint(X_equal_lorentz_tensor_constraint);
  Eigen::VectorXd ym_lorentz(m);
  ym_lorentz << 9.1, 1.3, 4.2;
  Eigen::VectorXd yn_lorentz(n);
  yn_lorentz << 2, -0.1, 0.5, -0.3;
  // The tensor product of cones is another cone. Hence, a conic combination of
  // two elements in the cone of Lorentz separable matrices must also be Lorentz
  // separable.
  Eigen::MatrixXd Y_tensor_lorentz = 2 * xm_lorentz * xn_lorentz.transpose() +
                                     4 * ym_lorentz * yn_lorentz.transpose();
  auto Y_equal_lorentz_tensor_constraint =
      prog.AddLinearEqualityConstraint(X == Y_tensor_lorentz);
  result = Solve(prog);
  // X is required to be equal to a lorentz separable tensor therefore this
  // program should be feasible.
  EXPECT_TRUE(result.is_success());

  // Two non-lorent vectors.
  Eigen::VectorXd zm(m);
  zm << -1.1, 2.3, 7.9;
  Eigen::VectorXd zn(n);
  zn << 0.1, 10.3, -0.2, 7.7;

  prog.RemoveConstraint(Y_equal_lorentz_tensor_constraint);
  auto bad_constraint =
      prog.AddLinearEqualityConstraint(X == zm * zn.transpose());
  result = Solve(prog);
  // X is required to be equal to something not that is not lorentz separable
  // therefore this should be infeasible.
  EXPECT_FALSE(result.is_success());

  prog.RemoveConstraint(bad_constraint);
  bad_constraint =
      prog.AddLinearEqualityConstraint(X == zm * xn_lorentz.transpose());
  result = Solve(prog);
  // X is required to be equal to something not that is not lorentz separable
  // therefore this should be infeasible.
  EXPECT_FALSE(result.is_success());

  prog.RemoveConstraint(bad_constraint);
  bad_constraint =
      prog.AddLinearEqualityConstraint(X == ym_lorentz * zn.transpose());
  result = Solve(prog);
  // X is required to be equal to something not that is not lorentz separable
  // therefore this should be infeasible.
  EXPECT_FALSE(result.is_success());
}

GTEST_TEST(MakeSemidefiniteRelaxationInternalTest,
           AddMatrixIsLorentzSeparableConstraint4by3) {
  MathematicalProgram prog;
  const int m = 4;
  const int n = 3;

  auto X = prog.NewContinuousVariables(m, n, "X");
  AddMatrixIsLorentzSeparableConstraint(X, &prog);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog.linear_equality_constraints()), 1);
  EXPECT_EQ(ssize(prog.GetAllConstraints()), 2);

  const Binding<PositiveSemidefiniteConstraint> psd_constraint =
      prog.positive_semidefinite_constraints()[0];
  EXPECT_EQ(psd_constraint.evaluator()->matrix_rows(), (m - 1) * (n - 1));

  Eigen::VectorXd xm_lorentz(m);
  xm_lorentz << 5.1, 0.1, 2.3, -0.99;
  Eigen::VectorXd xn_lorentz(n);
  xn_lorentz << 4.7, -0.2, 3.7;
  Eigen::MatrixXd X_tensor_lorentz = xm_lorentz * xn_lorentz.transpose();
  auto X_equal_lorentz_tensor_constraint =
      prog.AddLinearEqualityConstraint(X == X_tensor_lorentz);
  auto result = Solve(prog);
  // X is required to be equal to a simple lorentz separable tensor therefore
  // this program should be feasible.
  EXPECT_TRUE(result.is_success());

  prog.RemoveConstraint(X_equal_lorentz_tensor_constraint);
  Eigen::VectorXd ym_lorentz(m);
  ym_lorentz << 1.7, 0.1, 0.3, -0.99;
  Eigen::VectorXd yn_lorentz(n);
  yn_lorentz << 2.9, -1.1, 0.25;
  Eigen::MatrixXd Y_tensor_lorentz = 2.6 * xm_lorentz * yn_lorentz.transpose() +
                                     7.9 * ym_lorentz * xn_lorentz.transpose();
  auto Y_equal_lorentz_tensor_constraint =
      prog.AddLinearEqualityConstraint(X == Y_tensor_lorentz);
  result = Solve(prog);
  // X is required to be equal to a lorentz separable tensor therefore this
  // program should be feasible.
  EXPECT_TRUE(result.is_success());

  // Two non-lorent vectors.
  Eigen::VectorXd zm(m);
  zm << 1.09, -1.44, 1.58, -0.63;
  Eigen::VectorXd zn(n);
  zn << -2.04, -0.23, -3.87;

  prog.RemoveConstraint(Y_equal_lorentz_tensor_constraint);
  auto bad_constraint =
      prog.AddLinearEqualityConstraint(X == zm * zn.transpose());
  result = Solve(prog);
  // X is required to be equal to something not that is not lorentz separable
  // therefore this should be infeasible.
  EXPECT_FALSE(result.is_success());

  prog.RemoveConstraint(bad_constraint);
  bad_constraint =
      prog.AddLinearEqualityConstraint(X == zm * xn_lorentz.transpose());
  result = Solve(prog);
  // X is required to be equal to something not that is not lorentz separable
  // therefore this should be infeasible.
  EXPECT_FALSE(result.is_success());

  prog.RemoveConstraint(bad_constraint);
  bad_constraint =
      prog.AddLinearEqualityConstraint(X == ym_lorentz * zn.transpose());
  result = Solve(prog);
  // X is required to be equal to something not that is not lorentz separable
  // therefore this should be infeasible.
  EXPECT_FALSE(result.is_success());
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
