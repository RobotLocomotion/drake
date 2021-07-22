#include "drake/multibody/fixed_fem/dev/linear_constitutive_model_data.h"

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

  const Matrix3<double> F = 3.0 * Matrix3<double>::Identity();
  const Matrix3<double> expected_strain = 2.0 * Matrix3<double>::Identity();
  const double expected_trace_strain = 6.0;

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
