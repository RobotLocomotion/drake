#include "drake/math/matrix_util.h"

#include <limits>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace test {

using Eigen::MatrixXd;

GTEST_TEST(TestMatrixUtil, TestIsSymmetric) {
  auto A = Eigen::Matrix3d::Zero();
  EXPECT_TRUE(IsSymmetric(A, 0.0));
  EXPECT_TRUE(IsSymmetric(A.topLeftCorner<2, 2>(), 0.0));
  EXPECT_TRUE(IsSymmetric(Eigen::Matrix4d::Identity(), 0.0));
  Eigen::Matrix3d B;
  B << 1, 2, 3, 2, 4, 5, 3, 5, 6;
  EXPECT_TRUE(IsSymmetric(B, std::numeric_limits<double>::epsilon()));
  EXPECT_FALSE(IsSymmetric(Eigen::Matrix<double, 2, 3>::Zero(), 0.1));
  Eigen::Matrix3d C = B;
  C(0, 2) = 1.0;
  C(2, 0) = -1.0;
  EXPECT_FALSE(IsSymmetric(C, 1E-10));

  // Tests a integer scalar type.
  EXPECT_TRUE(IsSymmetric(Eigen::Matrix2i::Identity(), 0));
  Eigen::Matrix2i D;
  D << 0, 0, 1, 0;
  EXPECT_FALSE(IsSymmetric(D, 0));

  // Tests a symbolic expression.
  Eigen::Matrix<symbolic::Expression, 2, 2> S;
  symbolic::Variable s1{"s1"};
  symbolic::Variable s2{"s2"};
  symbolic::Variable s3{"s3"};
  symbolic::Expression S00{s1};
  symbolic::Expression S01{s2};
  symbolic::Expression S11{s3};
  S << S00, S01, S01, S11;
  EXPECT_TRUE(IsSymmetric(S));
}

GTEST_TEST(TestMatrixUtil, TestToSymmetricMatrixFromLowerTriangularColumns) {
  // Tests a static size vector.
  Eigen::Vector3d x1(1, 2, 3);
  Eigen::Matrix2d X1;
  // clang-format off
  X1 << 1, 2,
        2, 3;
  // clang-format on
  auto X1_result = ToSymmetricMatrixFromLowerTriangularColumns(x1);
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(decltype(X1_result), Eigen::Matrix2d);
  EXPECT_TRUE(CompareMatrices(X1_result, X1));

  // Tests a dynamic size vector.
  Eigen::VectorXd x2(6);
  x2 << 1, 2, 3, 4, 5, 6;
  Eigen::Matrix3d X2;
  // clang-format off
  X2 << 1, 2, 3,
        2, 4, 5,
        3, 5, 6;
  // clang-format on
  EXPECT_TRUE(
      CompareMatrices(ToSymmetricMatrixFromLowerTriangularColumns(x2), X2));
}

GTEST_TEST(TestMatrixUtil, TestToLowerTriangularColumnFromMatrix) {
  // Tests a static size vector.
  Eigen::Vector3d x1(1, 2, 3);
  Eigen::Matrix2d X1;
  // clang-format off
  X1 << 1, 2,
        2, 3;
  // clang-format on
  auto x1_result = ToLowerTriangularColumnsFromMatrix(X1);
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(decltype(x1_result), Eigen::Vector3d);
  EXPECT_TRUE(CompareMatrices(x1, x1_result));

  // Tests a dynamic size vector.
  Eigen::VectorXd x2(6);
  x2 << 1, 2, 3, 4, 5, 6;
  Eigen::Matrix3d X2;
  // clang-format off
  X2 << 1, 7, 8,
        2, 4, 9,
        3, 5, 6;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(ToLowerTriangularColumnsFromMatrix(X2), x2));
}

GTEST_TEST(TestMatrixUtil, IsPositiveDefiniteTest) {
  Eigen::Matrix3d A;
  A << 1, 2, 4, 2, 3, 5, 4, 5, 6;
  EXPECT_FALSE(IsPositiveDefinite(A, 1e-8));

  EXPECT_TRUE(IsPositiveDefinite(A * A.transpose(), 1e-8));

  // A is full rank and all its eigen values >= 0
  // However A is not symmetric, thus not PSD
  A << 3, 2, 1, 2, 3, 1, 1, 2, 3;
  EXPECT_FALSE(IsPositiveDefinite(A, 1e-8));
  // But if the tolerance for symmetry testing is absurd, then it passes:
  EXPECT_TRUE(IsPositiveDefinite(A, 1e-8, 5.));

  // Generate a PSD matrix.
  EXPECT_FALSE(IsPositiveDefinite(
      A * Eigen::Vector3d(1., 0., 1.).asDiagonal() * A.transpose(), 1e-8));
}

GTEST_TEST(TestMatrixUtil, StdVector) {
  MatrixXd A(3, 4);
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
  std::vector<MatrixXd> vec = EigenToStdVector<double>(A);
  EXPECT_EQ(vec.size(), 4);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(vec[i].size(), 3);
    EXPECT_TRUE(CompareMatrices(vec[i], A.col(i)));
  }
  MatrixXd mat = StdVectorToEigen(vec);
  EXPECT_TRUE(CompareMatrices(mat, A));

  EXPECT_EQ(
      EigenToStdVector<double>(Eigen::RowVector3d::LinSpaced(0.0, 1.0)).size(),
      3);
}

GTEST_TEST(TestMatrixUtil, ExtractPrincipalSubmatrixSquareMatrix) {
  int n = 8;
  drake::MatrixX<symbolic::Variable> X(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      X(i, j) = symbolic::Variable(fmt::format("X({},{})", i, j));
    }
  }
  // A set of contiguous indices which does not start at 0.
  std::set<int> submatrix_indices{1, 2};
  drake::MatrixX<symbolic::Variable> submatrix_manual(submatrix_indices.size(),
                                                      submatrix_indices.size());
  // clang-format off
  submatrix_manual << X(1, 1), X(1, 2),
                      X(2, 1), X(2, 2);
  // clang-format on
  auto submatrix = ExtractPrincipalSubmatrix(X, submatrix_indices);
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(decltype(submatrix),
                                       drake::MatrixX<symbolic::Variable>);

  auto TestSubmatrix =
      [&submatrix_indices, &submatrix_manual](
          const drake::MatrixX<symbolic::Variable>& submatrix_tested) {
        EXPECT_EQ(submatrix_tested.rows(), submatrix_tested.cols());
        EXPECT_EQ(static_cast<int>(submatrix_tested.rows()),
                  static_cast<int>(submatrix_indices.size()));
        for (int r = 0; r < submatrix_tested.rows(); ++r) {
          for (int c = 0; c < submatrix_tested.cols(); ++c) {
            EXPECT_TRUE(
                submatrix_tested(r, c).equal_to(submatrix_manual(r, c)));
          }
        }
      };
  TestSubmatrix(submatrix);

  // A set of non-contiguous indices where each block is of size 1.
  submatrix_indices.clear();
  submatrix_indices.insert({0, 2});
  submatrix_manual.resize(submatrix_indices.size(), submatrix_indices.size());
  // clang-format off
  submatrix_manual << X(0, 0), X(0, 2),
                      X(2, 0), X(2, 2);
  // clang-format on
  submatrix = ExtractPrincipalSubmatrix(X, submatrix_indices);
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(decltype(submatrix),
                                       drake::MatrixX<symbolic::Variable>);
  TestSubmatrix(submatrix);

  // A non-contiguous set of indices with blocks of size 2, 3, and 1.
  submatrix_indices.clear();
  submatrix_indices.insert({0, 1, 3, 4, 5, 7});
  submatrix_manual.resize(submatrix_indices.size(), submatrix_indices.size());
  // clang-format off
  submatrix_manual << X(0, 0), X(0, 1), X(0, 3), X(0, 4), X(0, 5), X(0, 7),
                      X(1, 0), X(1, 1), X(1, 3), X(1, 4), X(1, 5), X(1, 7),
                      X(3, 0), X(3, 1), X(3, 3), X(3, 4), X(3, 5), X(3, 7),
                      X(4, 0), X(4, 1), X(4, 3), X(4, 4), X(4, 5), X(4, 7),
                      X(5, 0), X(5, 1), X(5, 3), X(5, 4), X(5, 5), X(5, 7),
                      X(7, 0), X(7, 1), X(7, 3), X(7, 4), X(7, 5), X(7, 7);
  // clang-format on
  submatrix = ExtractPrincipalSubmatrix(X, submatrix_indices);
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(decltype(submatrix),
                                       drake::MatrixX<symbolic::Variable>);
  TestSubmatrix(submatrix);

  // A non-contiguous set of indices with blocks of different sizes none of
  // which start at 0.
  submatrix_indices.clear();
  submatrix_indices.insert({2, 3, 5, 7});
  submatrix_manual.resize(submatrix_indices.size(), submatrix_indices.size());
  // clang-format off
  submatrix_manual << X(2, 2), X(2, 3), X(2, 5), X(2, 7),
                       X(3, 2), X(3, 3), X(3, 5), X(3, 7),
                       X(5, 2), X(5, 3), X(5, 5), X(5, 7),
                       X(7, 2), X(7, 3), X(7, 5), X(7, 7);
  // clang-format on
  submatrix = ExtractPrincipalSubmatrix(X, submatrix_indices);
  TestSubmatrix(submatrix);
}

GTEST_TEST(TestMatrixUtil, ExtractPrincipalSubmatrixSquareMatrixErrors) {
  const int n = 8;
  const int k = 4;
  drake::MatrixX<symbolic::Variable> X(n, k);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < k; ++j) {
      X(i, j) = symbolic::Variable(fmt::format("X({},{})", i, j));
    }
  }

  // Index set is empty.
  std::set<int> indices{};
  EXPECT_ANY_THROW(ExtractPrincipalSubmatrix(X, indices));

  // First index is negative.
  indices.insert(-1);
  EXPECT_ANY_THROW(ExtractPrincipalSubmatrix(X, indices));

  indices.clear();
  // First index is as large as the number of rows.
  indices.insert(n);
  EXPECT_ANY_THROW(ExtractPrincipalSubmatrix(X, indices));

  indices.clear();
  indices.insert(k);
  // First index is as large as the number of columns.
  EXPECT_ANY_THROW(ExtractPrincipalSubmatrix(X, indices));

  indices.insert(0);
  // First index is in range, but later indices are too large.
  EXPECT_ANY_THROW(ExtractPrincipalSubmatrix(X, indices));
}

GTEST_TEST(GeneratePythonCsc, TestNonEmpty) {
  Eigen::Matrix<double, 2, 3> A;
  // clang-format off
  A << 1.5, 0.5, 1,
        0,  0.5, 0;
  // clang-format on
  const Eigen::SparseMatrix<double> A_sparse = A.sparseView();
  EXPECT_EQ(
      GeneratePythonCsc(A_sparse, "A"),
      "A = sparse.csc_matrix((np.array([1.5, 0.5, 0.5, 1], "
      "dtype=np.float64), ([0, 0, 1, 0], [0, 1, 1, 2])), shape=(2, 3))\n");
}

GTEST_TEST(GeneratePythonCsc, TestEmpty) {
  const Eigen::SparseMatrix<double> A_sparse(2, 3);
  EXPECT_EQ(GeneratePythonCsc(A_sparse, "A"),
            "A = sparse.csc_matrix((2, 3))\n");
}

}  // namespace test
}  // namespace math
}  // namespace drake
