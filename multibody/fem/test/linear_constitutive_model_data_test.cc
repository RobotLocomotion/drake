#include "drake/multibody/fem/linear_constitutive_model_data.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

/* Tests that the deformation gradient is initialized to the identity matrix and
 the deformation gradient dependent data are initialized to be consistent with
 the initial deformation gradient. */
GTEST_TEST(LinearConstitutiveModelDataTest, Initialization) {
  LinearConstitutiveModelData<double> linear_elasticity_data;
  EXPECT_TRUE(CompareMatrices(linear_elasticity_data.deformation_gradient(),
                              Matrix3<double>::Identity()));
  EXPECT_TRUE(CompareMatrices(linear_elasticity_data.strain(),
                              Matrix3<double>::Zero()));
  EXPECT_EQ(linear_elasticity_data.trace_strain(), 0.0);
}

GTEST_TEST(LinearConstitutiveModelDataTest, UpdateData) {
  LinearConstitutiveModelData<double> linear_elasticity_data;

  Matrix3<double> F;
  // clang-format off
  F << 4, 9, 2,
       3, 5, 7,
       8, 1, 6;
  // clang-format on
  Matrix3<double> expected_strain;
  // clang-format off
  expected_strain << 3, 6, 5,
                     6, 4, 4,
                     5, 4, 5;
  // clang-format on
  const double expected_trace_strain = 12.0;
  const auto F0 = F;

  linear_elasticity_data.UpdateData(F, F0);
  EXPECT_TRUE(
      CompareMatrices(linear_elasticity_data.deformation_gradient(), F));
  EXPECT_TRUE(CompareMatrices(linear_elasticity_data.strain(), expected_strain,
                              std::numeric_limits<double>::epsilon()));
  EXPECT_DOUBLE_EQ(linear_elasticity_data.trace_strain(),
                   expected_trace_strain);
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
