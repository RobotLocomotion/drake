#include "drake/math/differentiable_matrix.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
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

GTEST_TEST(DifferentiableMatrix, Misc) {
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

  DifferentiableMatrix<Matrix34ad> m34wd(m34ad);

  EXPECT_TRUE(CompareMatrices(ExtractValue(m34ad),
                              ExtractValue(m34wd.ToAutoDiffXd()), kEpsilon));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(m34ad),
                              ExtractGradient(m34wd.ToAutoDiffXd()), kEpsilon));

  DifferentiableMatrix<Matrix34ad>::TransposeType m34wdt =
      m34wd.transpose();
  auto m34adt = m34ad.transpose();

  EXPECT_TRUE(CompareMatrices(ExtractValue(m34adt),
                              ExtractValue(m34wdt.ToAutoDiffXd()), kEpsilon));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(m34adt),
                              ExtractGradient(m34wdt.ToAutoDiffXd()),
                              kEpsilon));
}

GTEST_TEST(DifferentiableMatrix, ConformabilityTest) {
  // Fixed size.
  using Matrix34d = Eigen::Matrix<double, 3, 4>;
  using Matrix43d = Eigen::Matrix<double, 4, Eigen::Dynamic, 0, 4, 20>;

  using Mult3443d = DifferentiableMatrixMultiply<Matrix34d, Matrix43d>;
  using Mult3434d = DifferentiableMatrixMultiply<Matrix34d, Matrix34d>;

  EXPECT_TRUE(Mult3443d::shapes_are_compatible);
  EXPECT_FALSE(Mult3443d::need_runtime_shape_test);
  EXPECT_TRUE(Mult3443d::ShapesAreConforming(Matrix34d(), Matrix43d()));

  EXPECT_FALSE(Mult3434d::shapes_are_compatible);
  EXPECT_FALSE(Mult3434d::need_runtime_shape_test);
  EXPECT_FALSE(Mult3434d::ShapesAreConforming(Matrix34d(), Matrix34d()));

  const bool good_result_type =
      std::is_same_v<Mult3443d::MatrixResultType,
                     Eigen::Matrix<double, 3, Eigen::Dynamic, 0, 3, 20>>;
  EXPECT_TRUE(good_result_type);
}

GTEST_TEST(DifferentiableMatrix, Multiply) {
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

  DifferentiableMatrix<Matrix34ad> dm34(m34ad);
  DifferentiableMatrix<Matrix43ad> dm43(m43ad);

  auto product = dm34 * dm43;
  const bool type_is_right =
      std::is_same_v<decltype(product)::MatrixType,
                     DifferentiableMatrixMultiply<
                         decltype(dm34)::MatrixType,
                         decltype(dm43)::MatrixType>::MatrixResultType>;
  EXPECT_TRUE(type_is_right);

  EXPECT_EQ(product.num_non_zeros(), 3);
  const std::vector<int> expected_nz{0, 1, 3};
  for (int i=0; i < product.num_non_zeros(); ++i)
    EXPECT_EQ(product.non_zero(i), expected_nz[i]);

  auto ad_result = m34ad * m43ad;
  EXPECT_TRUE(CompareMatrices(ExtractValue(product), ExtractValue(ad_result)));
  EXPECT_TRUE(
      CompareMatrices(ExtractGradient(product), ExtractGradient(ad_result)));
}

}  // namespace
}  // namespace math
}  // namespace drake

