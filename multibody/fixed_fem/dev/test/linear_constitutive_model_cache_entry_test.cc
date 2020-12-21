#include "drake/multibody/fixed_fem/dev/linear_constitutive_model_cache_entry.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace {
constexpr int kNumQuads = 1;

class LinearConstitutiveModelCacheEntryTest : public ::testing::Test {
 protected:
  void SetUp() {
    linear_elasticity_cache_entry_.UpdateCacheEntry(
        {MakeDeformationGradient()});
  }

  LinearConstitutiveModelCacheEntry<double, kNumQuads>
      linear_elasticity_cache_entry_;

  // Make an arbitrary deformation gradient.
  Matrix3<double> MakeDeformationGradient() {
    Matrix3<double> F;
    // clang-format off
    F << 1.2, 2.3, 3.4,
         4.5, 5.6, 6.7,
         7.8, 8.9, 9.0;
    // clang-format on
    return F;
  }
};

TEST_F(LinearConstitutiveModelCacheEntryTest,
       LinearElasticityCacheEntryInitialization) {
  EXPECT_EQ(linear_elasticity_cache_entry_.deformation_gradient().size(),
            kNumQuads);
  EXPECT_EQ(linear_elasticity_cache_entry_.strain().size(), kNumQuads);
  EXPECT_EQ(linear_elasticity_cache_entry_.trace_strain().size(), kNumQuads);
}

TEST_F(LinearConstitutiveModelCacheEntryTest, UpdateCacheEntry) {
  const Matrix3<double> F = MakeDeformationGradient();
  const Matrix3<double> strain =
      0.5 * (F + F.transpose()) - Matrix3<double>::Identity();
  const double trace_strain = strain.trace();
  EXPECT_EQ(linear_elasticity_cache_entry_.deformation_gradient()[0], F);
  EXPECT_EQ(linear_elasticity_cache_entry_.strain()[0], strain);
  EXPECT_EQ(linear_elasticity_cache_entry_.trace_strain()[0], trace_strain);
}
}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
