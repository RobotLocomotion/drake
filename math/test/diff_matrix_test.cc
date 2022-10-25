#include "drake/math/diff_matrix.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::AutoDiffScalar;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

namespace drake {
namespace math {
namespace {

GTEST_TEST(DiffMatrix, MatrixSelection) {
  MaybeDiffMatrix<double, 3, 4> matrix_d;
  MaybeDiffMatrix<symbolic::Expression, 3, 4> matrix_x;
  MaybeDiffMatrix<AutoDiffXd, 3, 4> matrix_diff;

  // Make a real Matrix<AutoDiffXd> for comparison.
  Eigen::Matrix<AutoDiffXd, 3, 4> matrix_ad;

  EXPECT_FALSE(is_diff_matrix<decltype(matrix_d)>);
  EXPECT_FALSE(is_diff_matrix<decltype(matrix_x)>);
  EXPECT_FALSE(is_diff_matrix<decltype(matrix_ad)>);
  EXPECT_TRUE(is_diff_matrix<decltype(matrix_diff)>);

  matrix_d = Eigen::Matrix<double, 3, 4>::Zero();
  matrix_x = Eigen::Matrix<symbolic::Expression, 3, 4>::Zero();
  matrix_ad = Eigen::Matrix<AutoDiffXd, 3, 4>::Zero();
  matrix_diff = Eigen::Matrix<AutoDiffXd, 3, 4>::Zero();

  // ExtractValue is only defined for Matrix<AutoDiffXd> and
  // DiffMatrix.
  EXPECT_EQ(&ExtractValue(matrix_diff), &matrix_diff.value());

  // DiffExtractValue is defined for any Eigen:Matrix and for
  // DiffMatrix. It is cost-free unless given an
  // Eigen::Matrix<AutoDiffXd> in which case a conversion is required.
  EXPECT_EQ(&DiffExtractValue(matrix_diff), &matrix_diff.value());
  EXPECT_EQ(&DiffExtractValue(matrix_d), &matrix_d);
  EXPECT_EQ(&DiffExtractValue(matrix_x), &matrix_x);
  const auto& HoldTemp = DiffExtractValue(matrix_ad);  // avoid warning
  EXPECT_NE(static_cast<const void*>(&HoldTemp),
            static_cast<const void*>(&matrix_ad));
  EXPECT_TRUE(CompareMatrices(HoldTemp, Eigen::Matrix<double, 3, 4>::Zero()));

  // Check that the MaybeDiffMatrixX variant defaults to dynamic-sized.
  MaybeDiffMatrixX<double> dyn_matrix_d;
  MaybeDiffMatrixX<AutoDiffXd> dyn_matrix_diff;

  EXPECT_FALSE(is_diff_matrix<decltype(dyn_matrix_d)>);
  EXPECT_EQ(decltype(dyn_matrix_d)::RowsAtCompileTime, Eigen::Dynamic);
  EXPECT_EQ(decltype(dyn_matrix_d)::ColsAtCompileTime, Eigen::Dynamic);

  EXPECT_TRUE(is_diff_matrix<decltype(dyn_matrix_diff)>);
  EXPECT_EQ(decltype(dyn_matrix_diff)::RowsAtCompileTime, Eigen::Dynamic);
  EXPECT_EQ(decltype(dyn_matrix_diff)::ColsAtCompileTime, Eigen::Dynamic);
}

GTEST_TEST(DiffMatrix, Misc) {
  // using Matrix34d = Eigen::Matrix<double, 3, 4>;
  // Matrix34d m34d;
  using Matrix34d = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  using Matrix34ad = Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, Eigen::Dynamic>;
  Matrix34d m34d(3, 4);
  m34d <<  1,  2,  3,  4,
           5,  6,  7,  8,
           9, 10, 11, 12;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> grad;
  grad.resize(12, 2);
  for (int i=0; i < 12; ++i) {
    grad.row(i) = Eigen::Vector2d(m34d(i) * 10, m34d(i) * 10 + 1);
  }
  // Eigen::Matrix<AutoDiffXd, 3, 4> m34ad;
  Matrix34ad m34ad;
  InitializeAutoDiff(m34d, grad, &m34ad);
  m34ad(1, 1).derivatives().resize(0);
  m34ad(2, 2).derivatives().resize(0);

  DiffMatrixX m34wd(m34ad);  // Dynamic size.

  EXPECT_TRUE(CompareMatrices(ExtractValue(m34ad),
                              ExtractValue(m34wd.ToAutoDiffXd()), kEpsilon));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(m34ad),
                              ExtractGradient(m34wd.ToAutoDiffXd()), kEpsilon));

  // Returned transposed matrix is always column major.
  DiffMatrixX::TransposeType m34wdt = m34wd.transpose();

  // Force Eigen's transpose into a column major matrix. (ExtractGradient
  // result below depends on storage order.)
  MatrixX<AutoDiffXd> m34adt =
      m34ad.transpose();

  EXPECT_TRUE(CompareMatrices(ExtractValue(m34adt),
                              ExtractValue(m34wdt.ToAutoDiffXd()), kEpsilon));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(m34adt),
                              ExtractGradient(m34wdt.ToAutoDiffXd()),
                              kEpsilon));
}

GTEST_TEST(DiffMatrix, ConformabilityTest) {
  // Fixed size.
  using Matrix34d = Eigen::Matrix<double, 3, 4>;
  using Matrix43d = Eigen::Matrix<double, 4, 3>;
  using Matrix43d_dynamic_cols =
      Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor, 4, 20>;
  using Matrix43d_dynamic_rows =
      Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor, 20, 3>;

  DiffMatrix<3, 4> diff34;

  EXPECT_TRUE(diff34.can_be_multiply_rhs(Matrix43d()));   // Fixed sizes.
  EXPECT_FALSE(diff34.can_be_multiply_rhs(Matrix34d()));  // Fixed sizes.

  // We don't care how many columns there are on the right.
  EXPECT_TRUE(diff34.can_be_multiply_rhs(Matrix43d_dynamic_cols()));
  // Zero rows, so no good.
  EXPECT_FALSE(diff34.can_be_multiply_rhs(Matrix43d_dynamic_rows()));

  // But should work with the right number of rows allocated.
  Matrix43d_dynamic_rows right_size(4, 3);
  EXPECT_TRUE(diff34.can_be_multiply_rhs(right_size));

  const bool good_result_type = std::is_same_v<
      DiffMatrix<3,
                           4>::EigenMultiplyResultType<Matrix43d_dynamic_cols>,
      Eigen::Matrix<AutoDiffXd, 3, Eigen::Dynamic, Eigen::ColMajor, 3, 20>>;
  EXPECT_TRUE(good_result_type);
}

GTEST_TEST(DiffMatrix, Multiply) {
  using Matrix34d = Eigen::Matrix<double, 3, 4>;
  using Matrix43d = Eigen::Matrix<double, 4, 3>;
  using Matrix34ad = Eigen::Matrix<AutoDiffXd, 3, 4>;
  using Matrix43ad = Eigen::Matrix<AutoDiffXd, 4, 3>;

  Matrix34d m34d;
  m34d <<  1,  2,  3,  4,
           5,  6,  7,  8,
           9, 10, 11, 12;
  Matrix43d m43d = -m34d.transpose();
  Eigen::Matrix<double, 12, Eigen::Dynamic> grad34, grad43;
  grad34.resize(12, 4);
  grad43.resize(12, 4);
  for (int i=0; i < 12; ++i) {
    // Note that element 2 is always zero in both gradients.
    grad34.row(i) = Eigen::Vector4d(0, m34d(i) * 10 + 1,
                                    0, m34d(i) * 10 + 2);
    grad43.row(i) = Eigen::Vector4d(m43d(i) * 10, 0,
                                    0, m43d(i) * 10 + 2);
  }

  Matrix34ad m34ad;
  Matrix43ad m43ad;
  InitializeAutoDiff(m34d, grad34, &m34ad);
  InitializeAutoDiff(m43d, grad43, &m43ad);

  DiffMatrix<3, 4> dm34(m34ad);
  DiffMatrix<4, 3> dm43(m43ad);

  auto product = dm34 * dm43;
  const bool type_is_right =
      std::is_same_v<decltype(product),
                     decltype(dm34)::MultiplyResultType<decltype(dm43)>>;
  EXPECT_TRUE(type_is_right);

  EXPECT_EQ(product.num_non_zeros(), 3);
  const std::vector<int> expected_nz{0, 1, 3};
  for (int i=0; i < product.num_non_zeros(); ++i)
    EXPECT_EQ(product.non_zero(i), expected_nz[i]);

  auto ad_product = (m34ad * m43ad).eval();
  EXPECT_TRUE(CompareMatrices(ExtractValue(product), ExtractValue(ad_product)));
  EXPECT_TRUE(
      CompareMatrices(ExtractGradient(product), ExtractGradient(ad_product)));

  auto mixed_product = dm34 * m43ad;
  const bool mixed_type_is_right =
      std::is_same_v<decltype(mixed_product), decltype(ad_product)>;
  EXPECT_TRUE(mixed_type_is_right);
  EXPECT_TRUE(
      CompareMatrices(ExtractValue(mixed_product), ExtractValue(ad_product)));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(mixed_product),
                              ExtractGradient(ad_product)));

  // Now try multiplying by a matrix that has no derivatives. That should
  // still be considered compatible and should produce a result whose shape
  // and number of non-zero derivatives inherits from the non-zero operand.

  Matrix43ad m43adz = InitializeAutoDiff(m43d, 0);  // No derivatives.
  DiffMatrix<4, 3> dm43z(m43adz);
  auto z_product = dm34 * dm43z;
  auto adz_product = m34ad * m43adz;
  EXPECT_EQ(z_product.num_variables(), dm34.num_variables());
  EXPECT_EQ(z_product.num_non_zeros(), dm34.num_non_zeros());
  for (int i = 0; i < z_product.num_non_zeros(); ++i)
    EXPECT_EQ(z_product.non_zero(i), dm34.non_zero(i));
  EXPECT_TRUE(
      CompareMatrices(ExtractValue(z_product), ExtractValue(adz_product)));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(z_product),
                              ExtractGradient(adz_product)));
}

// Check that the cast<U>() method produces the right types with the right
// values.
GTEST_TEST(DiffMatrix, Cast) {
  const Eigen::Matrix<double, 2, 2> value =
      (Eigen::Matrix2d() << 1.0, 2.0, 3.0, 4.0).finished();
  const Eigen::Matrix<AutoDiffXd, 2, 2> ad_matrix = InitializeAutoDiff(value);

  DiffMatrix<2, 2> diff_matrix(ad_matrix);

  decltype(auto) d_matrix = diff_matrix.cast<double>();
  EXPECT_EQ(&d_matrix, &diff_matrix.value());

  decltype(auto) s_matrix = diff_matrix.cast<symbolic::Expression>();
  const bool s_matrix_type_good =
      std::is_same_v<decltype(s_matrix),
                     Eigen::Matrix<symbolic::Expression, 2, 2>>;
  EXPECT_TRUE(s_matrix_type_good);
  EXPECT_TRUE(CompareMatrices(ExtractDoubleOrThrow(s_matrix), value));

  decltype(auto) ad_from_diff_matrix = diff_matrix.cast<AutoDiffXd>();
  const bool ad_from_diff_matrix_type_good =
      std::is_same_v<decltype(ad_from_diff_matrix),
                     Eigen::Matrix<AutoDiffXd, 2, 2>>;
  EXPECT_TRUE(ad_from_diff_matrix_type_good);
  EXPECT_TRUE(CompareMatrices(ExtractValue(ad_from_diff_matrix), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(ad_from_diff_matrix),
                              ExtractGradient(diff_matrix)));
}

}  // namespace
}  // namespace math
}  // namespace drake

