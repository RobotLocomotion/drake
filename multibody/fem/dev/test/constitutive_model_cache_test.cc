#include "drake/multibody/fem/dev/constitutive_model_cache.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace {
constexpr int kElementIndex = 3;
constexpr int kNumQuads = 5;
constexpr int kDim = 3;


class ConstitutiveModelCacheTest : public ::testing::Test {
 protected:
  void SetUp() {}
  LinearElasticityCache<double, kDim> linear_elasticity_cache_{kElementIndex,
                                                         kNumQuads};
};

TEST_F(ConstitutiveModelCacheTest, LinearElasticityCacheInitialization) {
  EXPECT_EQ(linear_elasticity_cache_.element_index, kElementIndex);
  EXPECT_EQ(linear_elasticity_cache_.num_quads, kNumQuads);
  EXPECT_EQ(linear_elasticity_cache_.F.size(), kNumQuads);
  EXPECT_EQ(linear_elasticity_cache_.strain.size(), kNumQuads);
  EXPECT_EQ(linear_elasticity_cache_.trace_strain.size(), kNumQuads);
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
