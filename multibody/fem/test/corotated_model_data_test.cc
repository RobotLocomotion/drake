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

constexpr double kTol = 1e-14;

/* Tests that the deformation gradient is initialized to the identity matrix and
 the deformation gradient dependent data are initialized to be consistent with
 the initial deformation gradient. */
GTEST_TEST(CorotatedModelDataTest, Initialization) {
  const CorotatedModelData<double> corotated_model_data;
  EXPECT_TRUE(CompareMatrices(corotated_model_data.deformation_gradient(),
                              Matrix3<double>::Identity()));
  EXPECT_TRUE(
      CompareMatrices(corotated_model_data.R(), Matrix3<double>::Identity()));
  EXPECT_TRUE(
      CompareMatrices(corotated_model_data.S(), Matrix3<double>::Identity()));
  EXPECT_EQ(corotated_model_data.Jm1(), 0.0);
  EXPECT_TRUE(CompareMatrices(corotated_model_data.JFinvT(),
                              Matrix3<double>::Identity()));
}

GTEST_TEST(CorotatedModelDataTest, UpdateData) {
  CorotatedModelData<double> corotated_model_data;
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
  const Matrix3<double> F0 = F;

  corotated_model_data.UpdateData(F, F0);
  EXPECT_TRUE(CompareMatrices(corotated_model_data.deformation_gradient(), F));
  EXPECT_TRUE(CompareMatrices(corotated_model_data.R(), R, kTol));
  EXPECT_TRUE(CompareMatrices(corotated_model_data.S(), S, kTol));
  EXPECT_NEAR(corotated_model_data.Jm1(), S.determinant() - 1.0, 10 * kTol);
  EXPECT_TRUE(CompareMatrices(corotated_model_data.JFinvT(),
                              F.determinant() * F.inverse().transpose(), kTol));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
