#include "drake/math/expmap.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::AutoDiffScalar;
using Eigen::Matrix3d;
using Eigen::Vector4d;

namespace drake {
namespace math {
namespace {

// Converts the given quaternion to an expmap and back. Asserts that the
// two expressions are equivalent.
void ConvertQuaternionToExpmapAndBack(const Vector4d& quat) {
  auto quat_autodiff = InitializeAutoDiff(quat);
  auto expmap_autodiff = quat2expmap(quat_autodiff);
  auto expmap = ExtractValue(expmap_autodiff);
  auto expmap_grad = ExtractGradient(expmap_autodiff);
  auto quat_back_autodiff = expmap2quat(InitializeAutoDiff(expmap));
  auto quat_back = ExtractValue(quat_back_autodiff);
  auto quat_back_grad = ExtractGradient(quat_back_autodiff);

  EXPECT_NEAR(std::abs(quat.dot(quat_back)), 1.0, 1e-8);
  Matrix3d identity = Matrix3d::Identity();
  EXPECT_TRUE(CompareMatrices((expmap_grad * quat_back_grad).eval(), identity,
                              1e-10, MatrixCompareType::absolute));
}

GTEST_TEST(ExpmapTest, QuaternionConversion) {
  Vector4d quat;
  quat << 0.5, 0.5, 0.5, 0.5;
  ConvertQuaternionToExpmapAndBack(quat);
}

GTEST_TEST(ExpmapTest, DegeneratePositiveQuaternionConversion) {
  Vector4d quat_degenerate = Vector4d::Zero();
  quat_degenerate(0) = 1.0;
  ConvertQuaternionToExpmapAndBack(quat_degenerate);
}

GTEST_TEST(ExpmapTest, DegenerateNegativeQuaternionConversion) {
  Vector4d quat_degenerate = Vector4d::Zero();
  quat_degenerate(0) = -1.0;
  ConvertQuaternionToExpmapAndBack(quat_degenerate);
}

}  // namespace
}  // namespace math
}  // namespace drake
