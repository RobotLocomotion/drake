#include "drake/multibody/fem/neohookean_model_data.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

constexpr double kTol = 8.0 * std::numeric_limits<double>::epsilon();

/* Tests that the deformation gradient is initialized to the identity matrix and
 the deformation gradient dependent data are initialized to be consistent with
 the initial deformation gradient. */
GTEST_TEST(NeoHookeanModelDataTest, Initialization) {
  const NeoHookeanModelData<double> neohookean_model_data;
  EXPECT_TRUE(CompareMatrices(neohookean_model_data.deformation_gradient(),
                              Matrix3<double>::Identity()));
  EXPECT_EQ(neohookean_model_data.Ic(), 3.0);
  EXPECT_EQ(neohookean_model_data.Jm1(), 0.0);
  EXPECT_TRUE(CompareMatrices(neohookean_model_data.dJdF(),
                              Matrix3<double>::Identity()));
}

GTEST_TEST(NeoHookeanModelDataTest, UpdateData) {
  NeoHookeanModelData<double> neohookean_model_data;
  // clang-format off
  const Matrix3<double> F = (Matrix3<double>() <<
         6, 1, 2,
         1, 4, 1,
         2, 1, 5)
  .finished();
  const Matrix3<double> F0 = (Matrix3<double>() <<
         1, 0, 0,
         0, 1, 0,
         0, 0, 1)
  .finished();
  // clang-format on
  neohookean_model_data.UpdateData(F, F0);
  EXPECT_EQ(neohookean_model_data.Ic(), F.squaredNorm());
  EXPECT_TRUE(CompareMatrices(neohookean_model_data.deformation_gradient(), F));
  EXPECT_NEAR(neohookean_model_data.Jm1(), F.determinant() - 1.0, kTol);
  EXPECT_TRUE(CompareMatrices(neohookean_model_data.dJdF(),
                              F.determinant() * F.inverse().transpose(), kTol));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
