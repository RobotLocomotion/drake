#include "drake/multibody/fem/linear_constitutive_model_data.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

constexpr int kNumQuads = 1;

GTEST_TEST(LinearConstitutiveModelDataTest, UpdateData) {
  LinearConstitutiveModelData<double, kNumQuads> linear_elasticity_data;
  ASSERT_EQ(linear_elasticity_data.deformation_gradient().size(), kNumQuads);
  ASSERT_EQ(linear_elasticity_data.strain().size(), kNumQuads);
  ASSERT_EQ(linear_elasticity_data.trace_strain().size(), kNumQuads);

  const Matrix3<double> F = 2.0 * Matrix3<double>::Identity();
  const Matrix3<double> expected_strain = Matrix3<double>::Identity();
  const double expected_trace_strain = 3.0;

  linear_elasticity_data.UpdateData({F});
  EXPECT_TRUE(CompareMatrices(linear_elasticity_data.deformation_gradient()[0],
                              F, std::numeric_limits<double>::epsilon()));
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
