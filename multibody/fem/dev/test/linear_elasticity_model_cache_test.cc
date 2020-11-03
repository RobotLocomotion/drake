#include "drake/multibody/fem/dev/linear_elasticity_model_cache.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace {
const ElementIndex kElementIndex(3);
constexpr int kNumQuads = 1;

class LinearElasticityCacheTest : public ::testing::Test {
 protected:
  LinearElasticityModelCache<double> linear_elasticity_cache_{kElementIndex,
                                                              kNumQuads};
};

TEST_F(LinearElasticityCacheTest, LinearElasticityCacheInitialization) {
  EXPECT_EQ(linear_elasticity_cache_.element_index(), kElementIndex);
  EXPECT_EQ(linear_elasticity_cache_.num_quads(), kNumQuads);
  EXPECT_EQ(linear_elasticity_cache_.deformation_gradient().size(), kNumQuads);
  EXPECT_EQ(linear_elasticity_cache_.strain().size(), kNumQuads);
  EXPECT_EQ(linear_elasticity_cache_.trace_strain().size(), kNumQuads);
}

TEST_F(LinearElasticityCacheTest, UpdateCache) {
  const Matrix3<double> F = Matrix3<double>::Random();
  linear_elasticity_cache_.UpdateCache({F});
  const Matrix3<double> strain =
      0.5 * (F + F.transpose()) - Matrix3<double>::Identity();
  const double trace_strain = strain.trace();
  EXPECT_EQ(linear_elasticity_cache_.deformation_gradient()[0], F);
  EXPECT_EQ(linear_elasticity_cache_.strain()[0], strain);
  EXPECT_EQ(linear_elasticity_cache_.trace_strain()[0], trace_strain);
}

TEST_F(LinearElasticityCacheTest, Clone) {
  const std::unique_ptr<DeformationGradientCache<double>> clone =
      linear_elasticity_cache_.Clone();
  // Test that the Clone() method returns the correct concrete type.
  const auto* linear_elasticity_cache_clone =
      dynamic_cast<const LinearElasticityModelCache<double>*>(clone.get());
  EXPECT_TRUE(linear_elasticity_cache_clone != nullptr);
  // Test that the Clone() method returns an identical copy.
  EXPECT_EQ(linear_elasticity_cache_clone->deformation_gradient(),
            linear_elasticity_cache_.deformation_gradient());
  EXPECT_EQ(linear_elasticity_cache_clone->strain(),
            linear_elasticity_cache_.strain());
  EXPECT_EQ(linear_elasticity_cache_clone->trace_strain(),
            linear_elasticity_cache_.trace_strain());
  EXPECT_EQ(linear_elasticity_cache_clone->num_quads(),
            linear_elasticity_cache_.num_quads());
  EXPECT_EQ(linear_elasticity_cache_clone->element_index(),
            linear_elasticity_cache_.element_index());
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
