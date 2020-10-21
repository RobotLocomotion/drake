#include "drake/multibody/fem/dev/linear_elasticity.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace fem {
class LinearElasticityTest : public ::testing::Test {
 protected:
  static constexpr int kDim = 3;
  static constexpr int kNumQuads = 2;

  void SetUp() {}

  LinearElasticity<double, kDim> ConstructLinearElasticity(double E,
                                                           double nu) {
    return LinearElasticity<double, kDim>(E, nu);
  }

  double get_mu(const LinearElasticity<double, kDim>& model) const {
    return model.mu_;
  }

  double get_lambda(const LinearElasticity<double, kDim>& model) const {
    return model.lambda_;
  }

  double get_E(const LinearElasticity<double, kDim>& model) const {
    return model.E_;
  }

  double get_nu(const LinearElasticity<double, kDim>& model) const {
    return model.nu_;
  }
};

namespace {
TEST_F(LinearElasticityTest, Parameters) {
  const auto model = ConstructLinearElasticity(100.0, 0.25);
  double mu = 40.0;
  double lambda = 40.0;
  EXPECT_EQ(get_E(model), 100.0);
  EXPECT_EQ(get_nu(model), 0.25);
  EXPECT_EQ(get_mu(model), mu);
  EXPECT_EQ(get_lambda(model), lambda);
}

TEST_F(LinearElasticityTest, InvalidYoungsModulus) {
  DRAKE_EXPECT_THROWS_MESSAGE(ConstructLinearElasticity(-1.0, 0.25),
                              std::logic_error,
                              "Young's modulus must be nonnegative.");
}

TEST_F(LinearElasticityTest, InvalidPoissonRatioAtUpperLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(ConstructLinearElasticity(100.0, 0.5),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

TEST_F(LinearElasticityTest, InvalidPoissonRatioOverUpperLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(ConstructLinearElasticity(100.0, 0.6),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

TEST_F(LinearElasticityTest, InvalidPoissonRatioAtLower) {
  DRAKE_EXPECT_THROWS_MESSAGE(ConstructLinearElasticity(100.0, -1.0),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

TEST_F(LinearElasticityTest, InvalidPoissonRatioBelowLowerLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(ConstructLinearElasticity(100.0, -1.1),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

TEST_F(LinearElasticityTest, ZeroEnergyDensity) {
  const auto model = ConstructLinearElasticity(100.0, 0.25);
  LinearElasticityCache<double, kDim> cache(0, kNumQuads);
  cache.F = std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Zero());
  cache.strain =
      std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Zero());
  cache.trace_strain = std::vector<double>(kNumQuads, 0);
  // With zero strain, the energy density should be zero.
  std::vector<double> analytic_energy_density(kNumQuads, 0);
  EXPECT_EQ(model.CalcPsi(cache), analytic_energy_density);
}

TEST_F(LinearElasticityTest, ArbitraryEnergyDensity) {
  const auto model = ConstructLinearElasticity(100.0, 0.25);
  LinearElasticityCache<double, kDim> cache(0, kNumQuads);
  // F does not enter the calculation of the energy density.
  cache.F = std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Random());
  cache.strain[0] = Vector3<double>(1, 2, 3).asDiagonal();
  cache.strain[1] = Vector3<double>(2, 3, 4).asDiagonal();
  cache.trace_strain[0] = 6;  // 1+2+3
  cache.trace_strain[1] = 9;  // 2+3+4
  double mu = get_mu(model);
  double lambda = get_lambda(model);
  double analytic_energy_density_0 =
      mu * Vector3<double>(1, 2, 3).squaredNorm() + 0.5 * lambda * 6 * 6;
  double analytic_energy_density_1 =
      mu * Vector3<double>(2, 3, 4).squaredNorm() + 0.5 * lambda * 9 * 9;

  auto numerical_energy_density = model.CalcPsi(cache);
  EXPECT_EQ(numerical_energy_density[0], analytic_energy_density_0);
  EXPECT_EQ(numerical_energy_density[1], analytic_energy_density_1);
}

TEST_F(LinearElasticityTest, ZeroStress) {
  const auto model = ConstructLinearElasticity(100.0, 0.25);
  LinearElasticityCache<double, kDim> cache(0, kNumQuads);
  cache.F = std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Zero());
  cache.strain =
      std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Zero());
  cache.trace_strain = std::vector<double>(kNumQuads, 0);
  // With zero strain, the energy density should be zero.
  std::vector<Matrix3<double>> analytic_stress(kNumQuads,
                                               Matrix3<double>::Zero());
  EXPECT_EQ(model.CalcP(cache), analytic_stress);
}

TEST_F(LinearElasticityTest, ArbitraryStress) {
  const auto model = ConstructLinearElasticity(100.0, 0.25);
  LinearElasticityCache<double, kDim> cache(0, kNumQuads);
  // F does not enter the calculation of the stress.
  cache.F = std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Random());
  cache.strain[0] = Vector3<double>(1, 2, 3).asDiagonal();
  cache.strain[1] = Vector3<double>(2, 3, 4).asDiagonal();
  cache.trace_strain[0] = 6;  // 1+2+3
  cache.trace_strain[1] = 9;  // 2+3+4
  double mu = get_mu(model);
  double lambda = get_lambda(model);
  Matrix3<double> analytic_stress_0 =
      2 * mu * cache.strain[0] + lambda * 6 * Matrix3<double>::Identity();
  Matrix3<double> analytic_stress_1 =
      2 * mu * cache.strain[1] + lambda * 9 * Matrix3<double>::Identity();

  auto numerical_stress = model.CalcP(cache);
  EXPECT_EQ(numerical_stress[0], analytic_stress_0);
  EXPECT_EQ(numerical_stress[1], analytic_stress_1);
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
