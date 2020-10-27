#include "drake/multibody/fem/dev/linear_elasticity_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/jacobian.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {
const ElementIndex kDummyElementIndex(0);
constexpr int kNumQuads = 2;

GTEST_TEST(LinearElasticityTest, Parameters) {
  const auto model = LinearElasticityModel<double>(100.0, 0.25);
  const double mu = 40.0;
  const double lambda = 40.0;
  EXPECT_EQ(model.youngs_modulus(), 100.0);
  EXPECT_EQ(model.poisson_ratio(), 0.25);
  EXPECT_EQ(model.shear_modulus(), mu);
  EXPECT_EQ(model.lame_first_parameter(), lambda);
}

GTEST_TEST(LinearElasticityTest, InvalidYoungsModulus) {
  DRAKE_EXPECT_THROWS_MESSAGE(LinearElasticityModel<double>(-1.0, 0.25),
                              std::logic_error,
                              "Young's modulus must be nonnegative.");
}

GTEST_TEST(LinearElasticityTest, InvalidPoissonRatioAtUpperLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(LinearElasticityModel<double>(100.0, 0.5),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(LinearElasticityTest, InvalidPoissonRatioOverUpperLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(LinearElasticityModel<double>(100.0, 0.6),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(LinearElasticityTest, InvalidPoissonRatioAtLower) {
  DRAKE_EXPECT_THROWS_MESSAGE(LinearElasticityModel<double>(100.0, -1.0),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(LinearElasticityTest, InvalidPoissonRatioBelowLowerLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(LinearElasticityModel<double>(100.0, -1.1),
                              std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(LinearElasticityTest, UndeformedState) {
  const auto model = LinearElasticityModel<double>(100.0, 0.25);
  LinearElasticityModelCache<double> cache(kDummyElementIndex, kNumQuads);
  const auto F =
      std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Identity());
  cache.UpdateCache(F);
  // In undeformed state, the energy density should be zero.
  const std::vector<double> analytic_energy_density(kNumQuads, 0);
  // In undeformaed state, the stress should be zero.
  const std::vector<Matrix3<double>> analytic_stress(kNumQuads,
                                                     Matrix3<double>::Zero());
  EXPECT_EQ(model.CalcPsi(cache), analytic_energy_density);
  EXPECT_EQ(model.CalcP(cache), analytic_stress);
}

GTEST_TEST(LinearElasticityTest, PIsDerivativeOfPsi) {
  const auto model_autodiff = LinearElasticityModel<AutoDiffXd>(100.0, 0.25);
  LinearElasticityModelCache<AutoDiffXd> cache_autodiff(kDummyElementIndex,
                                                        kNumQuads);
  // Create random AutoDiffXd deformation.
  auto Fs = std::vector<Matrix3<double>>(kNumQuads, Matrix3<double>::Random());
  std::vector<Matrix3<AutoDiffXd>> Fs_autodiff(kNumQuads);
  const Eigen::Matrix<double, 9, Eigen::Dynamic> gradient =
      MatrixX<double>::Identity(9, 9);
  for (int i = 0; i < kNumQuads; ++i) {
    auto F_autodiff_flat = math::initializeAutoDiffGivenGradientMatrix(
        Eigen::Map<Eigen::Matrix<double, 9, 1>>(Fs[i].data(), 9), gradient);
    Fs_autodiff[i] =
        Eigen::Map<Matrix3<AutoDiffXd>>(F_autodiff_flat.data(), 3, 3);
  }
  // P should be derivative of Psi.
  cache_autodiff.UpdateCache(Fs_autodiff);
  auto energy = model_autodiff.CalcPsi(cache_autodiff);
  auto P = model_autodiff.CalcP(cache_autodiff);
  for (int i = 0; i < kNumQuads; ++i) {
    EXPECT_TRUE(CompareMatrices(
        Eigen::Map<Matrix3<double>>(energy[i].derivatives().data(), 3, 3),
        P[i]));
  }
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
