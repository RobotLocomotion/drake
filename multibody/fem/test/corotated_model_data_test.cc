#include "drake/multibody/fem/corotated_model_data.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

constexpr int kNumLocations = 1;
constexpr double kTol = 1e-14;

void VerifySizes(const CorotatedModelData<double, kNumLocations>& data) {
  ASSERT_EQ(data.deformation_gradient().size(), kNumLocations);
  ASSERT_EQ(data.R().size(), kNumLocations);
  ASSERT_EQ(data.S().size(), kNumLocations);
  ASSERT_EQ(data.Jm1().size(), kNumLocations);
  ASSERT_EQ(data.JFinvT().size(), kNumLocations);
}

/* Tests that the deformation gradient is initialized to the identity matrix and
 the deformation gradient dependent data are initialized to be consistent with
 the initial deformation gradient. */
GTEST_TEST(CorotatedModelDataTest, Initialization) {
  const CorotatedModelData<double, kNumLocations> corotated_model_data;
  VerifySizes(corotated_model_data);
  EXPECT_TRUE(CompareMatrices(corotated_model_data.deformation_gradient()[0],
                              Matrix3<double>::Identity()));
  EXPECT_TRUE(CompareMatrices(corotated_model_data.R()[0],
                              Matrix3<double>::Identity()));
  EXPECT_TRUE(CompareMatrices(corotated_model_data.S()[0],
                              Matrix3<double>::Identity()));
  EXPECT_EQ(corotated_model_data.Jm1()[0], 0.0);
  EXPECT_TRUE(CompareMatrices(corotated_model_data.JFinvT()[0],
                              Matrix3<double>::Identity()));
}

GTEST_TEST(CorotatedModelDataTest, UpdateData) {
  CorotatedModelData<double, kNumLocations> corotated_model_data;
  /* We set deformation gradient as F = R*S where R is an arbitrary rotation
   matrix and S is an arbitrary sysmmetric positive definite matrix. */
  const Matrix3<double> R =
      math::RotationMatrix<double>(math::RollPitchYaw<double>(1.0, 2.0, 3.0))
          .matrix();
  // clang-format off
  const Matrix3<double> S = (Matrix3<double>() <<
         6, 1, 2,
         1, 4, 1,
         2, 1, 5)
  .finished();
  // clang-format on
  const Matrix3<double> F = R * S;

  corotated_model_data.UpdateData({F});
  EXPECT_TRUE(
      CompareMatrices(corotated_model_data.deformation_gradient()[0], F));
  EXPECT_TRUE(CompareMatrices(corotated_model_data.R()[0], R, kTol));
  EXPECT_TRUE(CompareMatrices(corotated_model_data.S()[0], S, kTol));
  EXPECT_NEAR(corotated_model_data.Jm1()[0], S.determinant() - 1.0, 10 * kTol);
  EXPECT_TRUE(CompareMatrices(corotated_model_data.JFinvT()[0],
                              F.determinant() * F.inverse().transpose(), kTol));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
