#include "drake/multibody/fem/dev/elasticity_element_cache.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/fem/dev/constitutive_model_cache.h"

namespace drake {
namespace multibody {
namespace fem {

class ElasticityElementCacheTest : public ::testing::Test {
 protected:
  static constexpr int kElementIndex = 3;
  static constexpr int kNumQuads = 5;
  static constexpr int kDim = 3;
  void SetUp() {}
  ElasticityElementCache<double, kDim, LinearElasticityCache<double, kDim>>
      elasticity_cache_{kElementIndex, kNumQuads};

  void VerifyAllCacheStale() const {
    EXPECT_TRUE(elasticity_cache_.F_stale());
    EXPECT_TRUE(elasticity_cache_.F0_stale());
    EXPECT_TRUE(elasticity_cache_.constitutive_model_cache_stale());
    EXPECT_TRUE(elasticity_cache_.constitutive_model_x_cache_stale());
    EXPECT_TRUE(elasticity_cache_.constitutive_model_x0_cache_stale());
    EXPECT_TRUE(elasticity_cache_.Psi_stale());
    EXPECT_TRUE(elasticity_cache_.P_stale());
    EXPECT_TRUE(elasticity_cache_.dPdF_stale());
  }

  void VerifyAllCacheFresh() const {
    EXPECT_FALSE(elasticity_cache_.F_stale());
    EXPECT_FALSE(elasticity_cache_.F0_stale());
    EXPECT_FALSE(elasticity_cache_.constitutive_model_cache_stale());
    EXPECT_FALSE(elasticity_cache_.constitutive_model_x_cache_stale());
    EXPECT_FALSE(elasticity_cache_.constitutive_model_x0_cache_stale());
    EXPECT_FALSE(elasticity_cache_.Psi_stale());
    EXPECT_FALSE(elasticity_cache_.P_stale());
    EXPECT_FALSE(elasticity_cache_.dPdF_stale());
  }

  void MarkAllCacheFresh() {
    elasticity_cache_.mark_F_stale(false);
    elasticity_cache_.mark_F0_stale(false);
    elasticity_cache_.mark_constitutive_model_cache_stale(false);
    elasticity_cache_.mark_Psi_stale(false);
    elasticity_cache_.mark_P_stale(false);
    elasticity_cache_.mark_dPdF_stale(false);
  }

  // Create fake values for F.
  std::vector<Matrix3<double>> CreateF() const {
    Matrix3<double> F = Matrix3<double>::Identity();
    return std::vector<Matrix3<double>>(kNumQuads, F);
  }

  // Create fake values for F0.
  std::vector<Matrix3<double>> CreateF0() const {
    Matrix3<double> F0 = 2.0 * Matrix3<double>::Identity();
    return std::vector<Matrix3<double>>(kNumQuads, F0);
  }

  // Create fake value for model cache.
  LinearElasticityCache<double, kDim> CreateModelCache() const {
    LinearElasticityCache<double, kDim> linear_elasticity_cache(kElementIndex,
                                                          kNumQuads);
    linear_elasticity_cache.F = CreateF();
    linear_elasticity_cache.strain = CreateP();
    linear_elasticity_cache.trace_strain = CreatePsi();
    return linear_elasticity_cache;
  }

  // Create fake values for Psi.
  std::vector<double> CreatePsi() const {
    return std::vector<double>(kNumQuads, 0.123);
  }

  // Create fake values for P.
  std::vector<Matrix3<double>> CreateP() const {
    Matrix3<double> P = 3.0 * Matrix3<double>::Identity();
    return std::vector<Matrix3<double>>(kNumQuads, P);
  }

  // Create fake values for dPdF.
  std::vector<Eigen::Matrix<double, 9, 9>> CreatedPdF() const {
    Eigen::Matrix<double, 9, 9> dPdF =
        4.0 * Eigen::Matrix<double, 9, 9>::Identity();
    return std::vector<Eigen::Matrix<double, 9, 9>>(kNumQuads, dPdF);
  }
};

namespace {

TEST_F(ElasticityElementCacheTest, Initialization) {
  // Verify that the element index and the number of quadrature points
  // percolates.
  EXPECT_EQ(elasticity_cache_.element_index(), kElementIndex);
  EXPECT_EQ(elasticity_cache_.num_quads(), kNumQuads);
  EXPECT_EQ(elasticity_cache_.get_F().size(), kNumQuads);
  EXPECT_EQ(elasticity_cache_.get_F0().size(), kNumQuads);
  EXPECT_EQ(elasticity_cache_.get_constitutive_model_cache().element_index,
            kElementIndex);
  EXPECT_EQ(elasticity_cache_.get_constitutive_model_cache().num_quads,
            kNumQuads);
  EXPECT_EQ(elasticity_cache_.get_Psi().size(), kNumQuads);
  EXPECT_EQ(elasticity_cache_.get_P().size(), kNumQuads);
  EXPECT_EQ(elasticity_cache_.get_dPdF().size(), kNumQuads);
  // Verify that all `stale flags` are initialized to true.
  VerifyAllCacheStale();
}

TEST_F(ElasticityElementCacheTest, MarkAsStale) {
  MarkAllCacheFresh();
  VerifyAllCacheFresh();
}

TEST_F(ElasticityElementCacheTest, Getters) {
  MarkAllCacheFresh();
  auto& F = elasticity_cache_.get_mutable_F();
  auto& F0 = elasticity_cache_.get_mutable_F0();
  auto& model_cache = elasticity_cache_.get_mutable_constitutive_model_cache();
  auto& Psi = elasticity_cache_.get_mutable_Psi();
  auto& P = elasticity_cache_.get_mutable_P();
  auto& dPdF = elasticity_cache_.get_mutable_dPdF();
  // Non-const getters should mark the corresponding cache stale.
  VerifyAllCacheStale();
  // Give the cache some fake values.
  F = CreateF();
  F0 = CreateF0();
  model_cache = CreateModelCache();
  Psi = CreatePsi();
  P = CreateP();
  dPdF = CreatedPdF();

  // Test const getters.
  EXPECT_EQ(elasticity_cache_.get_F(), CreateF());
  EXPECT_EQ(elasticity_cache_.get_F0(), CreateF0());
  EXPECT_EQ(elasticity_cache_.get_Psi(), CreatePsi());
  EXPECT_EQ(elasticity_cache_.get_P(), CreateP());
  EXPECT_EQ(elasticity_cache_.get_dPdF(), CreatedPdF());

  const auto& linear_elasticity_cache =
      elasticity_cache_.get_constitutive_model_cache();
  const auto& created_cache = CreateModelCache();
  EXPECT_EQ(linear_elasticity_cache.element_index, created_cache.element_index);
  EXPECT_EQ(linear_elasticity_cache.num_quads, created_cache.num_quads);
  EXPECT_EQ(linear_elasticity_cache.F, created_cache.F);
  EXPECT_EQ(linear_elasticity_cache.strain, created_cache.strain);
  EXPECT_EQ(linear_elasticity_cache.trace_strain, created_cache.trace_strain);
}

TEST_F(ElasticityElementCacheTest, xCache) {
  MarkAllCacheFresh();
  elasticity_cache_.mark_x_cache_stale();
  // F, constitutive_model_x_cache, Psi, P, dPdF depend on x.
  EXPECT_TRUE(elasticity_cache_.F_stale());
  EXPECT_FALSE(elasticity_cache_.F0_stale());
  EXPECT_TRUE(elasticity_cache_.constitutive_model_cache_stale());
  EXPECT_TRUE(elasticity_cache_.constitutive_model_x_cache_stale());
  EXPECT_FALSE(elasticity_cache_.constitutive_model_x0_cache_stale());
  EXPECT_TRUE(elasticity_cache_.Psi_stale());
  EXPECT_TRUE(elasticity_cache_.P_stale());
  EXPECT_TRUE(elasticity_cache_.dPdF_stale());
}

TEST_F(ElasticityElementCacheTest, x0Cache) {
  MarkAllCacheFresh();
  elasticity_cache_.mark_x0_cache_stale();
  // F0, constitutive_model_x0_cache, Psi, P, dPdF depend on x0.
  EXPECT_FALSE(elasticity_cache_.F_stale());
  EXPECT_TRUE(elasticity_cache_.F0_stale());
  EXPECT_TRUE(elasticity_cache_.constitutive_model_cache_stale());
  EXPECT_FALSE(elasticity_cache_.constitutive_model_x_cache_stale());
  EXPECT_TRUE(elasticity_cache_.constitutive_model_x0_cache_stale());
  EXPECT_TRUE(elasticity_cache_.Psi_stale());
  EXPECT_TRUE(elasticity_cache_.P_stale());
  EXPECT_TRUE(elasticity_cache_.dPdF_stale());
}

TEST_F(ElasticityElementCacheTest, vCache) {
  // No cache entry depends on v currently.
  MarkAllCacheFresh();
  elasticity_cache_.mark_v_cache_stale();
  VerifyAllCacheFresh();
}

TEST_F(ElasticityElementCacheTest, v0Cache) {
  // No cache entry depends on v0 currently.
  MarkAllCacheFresh();
  elasticity_cache_.mark_v0_cache_stale();
  VerifyAllCacheFresh();
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
