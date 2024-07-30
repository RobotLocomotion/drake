#include "drake/multibody/fem/linear_corotated_model_data.h"

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
GTEST_TEST(LinearCorotatedModelDataTest, Initialization) {
  const LinearCorotatedModelData<double> linear_corotated_model_data;
  EXPECT_TRUE(
      CompareMatrices(linear_corotated_model_data.deformation_gradient(),
                      Matrix3<double>::Identity()));
  EXPECT_EQ(linear_corotated_model_data.previous_step_deformation_gradient(),
            Matrix3<double>::Identity());
  EXPECT_TRUE(CompareMatrices(linear_corotated_model_data.R0(),
                              Matrix3<double>::Identity()));
  EXPECT_TRUE(CompareMatrices(linear_corotated_model_data.strain(),
                              Matrix3<double>::Zero()));
  EXPECT_EQ(linear_corotated_model_data.trace_strain(), 0.0);
}

GTEST_TEST(LinearCorotatedModelDataTest, UpdateData) {
  LinearCorotatedModelData<double> linear_corotated_model_data;
  /* We set deformation gradient as F0 = R0*S0 where R0 is an arbitrary rotation
   matrix and S0 is an arbitrary symmetric positive definite matrix. */
  const Matrix3<double> R0 =
      math::RotationMatrix<double>(math::RollPitchYaw<double>(1.0, 2.0, 3.0))
          .matrix();
  // clang-format off
  const Matrix3<double> S0 = (Matrix3<double>() <<
         6, 1, 2,
         1, 4, 1,
         2, 1, 5)
  .finished();
  const Matrix3<double> F = (Matrix3<double>() <<
         1.1, 0.1, 0,
         0.2, 1.4, 0.3,
         0.2, 0.3, 1.5)
  .finished();
  // clang-format on
  const Matrix3<double> F0 = R0 * S0;

  linear_corotated_model_data.UpdateData(F, F0);
  EXPECT_TRUE(
      CompareMatrices(linear_corotated_model_data.deformation_gradient(), F));
  EXPECT_EQ(linear_corotated_model_data.previous_step_deformation_gradient(),
            F0);
  EXPECT_TRUE(CompareMatrices(linear_corotated_model_data.R0(), R0, kTol));
  const Matrix3<double> expected_strain =
      0.5 * (R0.transpose() * F + F.transpose() * R0) -
      Matrix3<double>::Identity();
  EXPECT_NEAR(linear_corotated_model_data.trace_strain(),
              expected_strain.trace(), kTol);
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
