#include "drake/multibody/fem/dev/linear_elasticity.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {
const ElementIndex kDummyElementIndex(0);
constexpr int kNumQuads = 2;

GTEST_TEST(LinearElasticityTest, Parameters) {
  const auto model = LinearElasticity<double>(100.0, 0.25);
  const double mu = 40.0;
  const double lambda = 40.0;
  EXPECT_EQ(model.youngs_modulus(), 100.0);
  EXPECT_EQ(model.poisson_ratio(), 0.25);
  EXPECT_EQ(model.shear_modulus(), mu);
  EXPECT_EQ(model.lame_first_parameter(), lambda);
}

GTEST_TEST(LinearElasticityTest, InvalidYoungsModulus) {
  DRAKE_EXPECT_THROWS_MESSAGE(LinearElasticity<double>(-1.0, 0.25),
                              std::logic_error,
                              "Young's modulus must be nonnegative.");
}

GTEST_TEST(LinearElasticityTest, InvalidPoissonRatioAtUpperLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(LinearElasticity<double>(100.0, 0.5),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(LinearElasticityTest, InvalidPoissonRatioOverUpperLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(LinearElasticity<double>(100.0, 0.6),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(LinearElasticityTest, InvalidPoissonRatioAtLower) {
  DRAKE_EXPECT_THROWS_MESSAGE(LinearElasticity<double>(100.0, -1.0),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(LinearElasticityTest, InvalidPoissonRatioBelowLowerLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(LinearElasticity<double>(100.0, -1.1),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(LinearElasticityTest, ZeroEnergyDensity) {
  const auto model = LinearElasticity<double>(100.0, 0.25);
  LinearElasticityCache<double> cache(kDummyElementIndex, kNumQuads);
  const auto F =
      std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Identity());
  cache.UpdateCache(F);
  // With zero strain, the energy density should be zero.
  const std::vector<double> analytic_energy_density(kNumQuads, 0);
  EXPECT_EQ(model.CalcPsi(cache), analytic_energy_density);
}

GTEST_TEST(LinearElasticityTest, ArbitraryEnergyDensity) {
  const auto model = LinearElasticity<double>(100.0, 0.25);
  LinearElasticityCache<double> cache(kDummyElementIndex, kNumQuads);
  const auto F =
      std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Random());
  cache.UpdateCache(F);
  const double mu = model.shear_modulus();
  const double lambda = model.lame_first_parameter();
  const auto numerical_energy_density = model.CalcPsi(cache);
  for (int i = 0; i < kNumQuads; ++i) {
    const double analytic_energy_density =
        mu * cache.strain()[i].squaredNorm() +
        0.5 * lambda * cache.trace_strain()[i] * cache.trace_strain()[i];
    EXPECT_EQ(numerical_energy_density[i], analytic_energy_density);
  }
}

GTEST_TEST(LinearElasticityTest, ZeroStress) {
  const auto model = LinearElasticity<double>(100.0, 0.25);
  LinearElasticityCache<double> cache(kDummyElementIndex, kNumQuads);
  const auto F =
      std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Identity());
  cache.UpdateCache(F);
  // With zero strain, the energy density should be zero.
  const std::vector<Matrix3<double>> analytic_stress(kNumQuads,
                                                     Matrix3<double>::Zero());
  EXPECT_EQ(model.CalcP(cache), analytic_stress);
}

GTEST_TEST(LinearElasticityTest, ArbitraryStress) {
  const auto model = LinearElasticity<double>(100.0, 0.25);
  LinearElasticityCache<double> cache(kDummyElementIndex, kNumQuads);
  const auto F =
      std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Random());
  cache.UpdateCache(F);
  const double mu = model.shear_modulus();
  const double lambda = model.lame_first_parameter();
  const auto numerical_stress = model.CalcP(cache);
  for (int i = 0; i < kNumQuads; ++i) {
    const Matrix3<double> analytic_stress =
        2 * mu * cache.strain()[i] +
        lambda * cache.trace_strain()[i] * Matrix3<double>::Identity();
    EXPECT_EQ(numerical_stress[i], analytic_stress);
  }
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
