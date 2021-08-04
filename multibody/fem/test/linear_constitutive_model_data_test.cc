#include "drake/multibody/fem/linear_constitutive_model_data.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

constexpr int kNumLocations = 1;

void VerifySizes(
    const LinearConstitutiveModelData<double, kNumLocations>& data) {
  ASSERT_EQ(data.deformation_gradient().size(), kNumLocations);
  ASSERT_EQ(data.strain().size(), kNumLocations);
  ASSERT_EQ(data.trace_strain().size(), kNumLocations);
}

/* Tests that the deformation gradient is initialized to the identity matrix and
 the deformation gradient dependent data are initialized to be consistent with
 the initial deformation gradient. */
GTEST_TEST(LinearConstitutiveModelDataTest, Initialization) {
  LinearConstitutiveModelData<double, kNumLocations> linear_elasticity_data;
  VerifySizes(linear_elasticity_data);
  EXPECT_TRUE(CompareMatrices(linear_elasticity_data.deformation_gradient()[0],
                              Matrix3<double>::Identity()));
  EXPECT_TRUE(CompareMatrices(linear_elasticity_data.strain()[0],
                              Matrix3<double>::Zero()));
  EXPECT_EQ(linear_elasticity_data.trace_strain()[0], 0.0);
}

GTEST_TEST(LinearConstitutiveModelDataTest, UpdateData) {
  LinearConstitutiveModelData<double, kNumLocations> linear_elasticity_data;

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

  linear_elasticity_data.UpdateData({F});
  EXPECT_TRUE(
      CompareMatrices(linear_elasticity_data.deformation_gradient()[0], F));
  EXPECT_TRUE(CompareMatrices(linear_elasticity_data.strain()[0],
                              expected_strain,
                              std::numeric_limits<double>::epsilon()));
  EXPECT_DOUBLE_EQ(linear_elasticity_data.trace_strain()[0],
                   expected_trace_strain);
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
