#include "drake/multibody/fem/dev/fixed_size_linear_constitutive_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {
const ElementIndex kDummyElementIndex(0);
constexpr int kNumQuads = 1;

// Creates a vector of arbitrary deformation gradients.
std::array<Matrix3<AutoDiffXd>, kNumQuads> MakeDeformationGradients() {
  // Create random AutoDiffXd deformation.
  Matrix3<double> F;
  F << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85;
  const std::array<Matrix3<double>, kNumQuads> Fs{F};
  std::array<Matrix3<AutoDiffXd>, kNumQuads> Fs_autodiff;
  const Eigen::Matrix<double, 9, Eigen::Dynamic> gradient(
      Eigen::Matrix<double, 9, 9>::Identity());
  for (int i = 0; i < kNumQuads; ++i) {
    const auto F_autodiff_flat = math::initializeAutoDiffGivenGradientMatrix(
        Eigen::Map<const Eigen::Matrix<double, 9, 1>>(Fs[i].data(), 9),
        gradient);
    Fs_autodiff[i] =
        Eigen::Map<const Matrix3<AutoDiffXd>>(F_autodiff_flat.data(), 3, 3);
  }
  return Fs_autodiff;
}

GTEST_TEST(FixedSizeLinearConstitutiveModelTest, Parameters) {
  const FixedSizeLinearConstitutiveModel<double, kNumQuads> model(100.0, 0.25);
  const double mu = 40.0;
  const double lambda = 40.0;
  EXPECT_EQ(model.youngs_modulus(), 100.0);
  EXPECT_EQ(model.poisson_ratio(), 0.25);
  EXPECT_EQ(model.shear_modulus(), mu);
  EXPECT_EQ(model.lame_first_parameter(), lambda);
}

GTEST_TEST(FixedSizeLinearConstitutiveModelTest, InvalidYoungsModulus) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      (FixedSizeLinearConstitutiveModel<double, kNumQuads>(-1.0, 0.25)),
      std::logic_error, "Young's modulus must be nonnegative.");
}

GTEST_TEST(FixedSizeLinearConstitutiveModelTest,
           InvalidPoissonRatioAtUpperLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      (FixedSizeLinearConstitutiveModel<double, kNumQuads>(100.0, 0.5)),
      std::logic_error, "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(FixedSizeLinearConstitutiveModelTest,
           InvalidPoissonRatioOverUpperLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      (FixedSizeLinearConstitutiveModel<double, kNumQuads>(100.0, 0.6)),
      std::logic_error, "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(FixedSizeLinearConstitutiveModelTest, InvalidPoissonRatioAtLower) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      (FixedSizeLinearConstitutiveModel<double, kNumQuads>(100.0, -1.0)),
      std::logic_error, "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(FixedSizeLinearConstitutiveModelTest,
           InvalidPoissonRatioBelowLowerLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      (FixedSizeLinearConstitutiveModel<double, kNumQuads>(100.0, -1.1)),
      std::logic_error, "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(FixedSizeLinearConstitutiveModelTest, UndeformedState) {
  const FixedSizeLinearConstitutiveModel<double, kNumQuads> model(100.0, 0.25);
  FixedSizeLinearConstitutiveModelCacheEntry<double, kNumQuads> cache_entry(
      kDummyElementIndex);
  const std::array<Matrix3<double>, kNumQuads> F{Matrix3<double>::Identity()};
  cache_entry.UpdateCacheEntry(F);
  // In undeformed state, the energy density should be zero.
  const std::array<double, kNumQuads> analytic_energy_density{0};
  // In undeformaed state, the stress should be zero.
  const std::array<Matrix3<double>, kNumQuads> analytic_stress{
      Matrix3<double>::Zero()};
  std::array<double, kNumQuads> energy_density;
  model.CalcElasticEnergyDensity(cache_entry, &energy_density);
  EXPECT_EQ(energy_density, analytic_energy_density);
  std::array<Matrix3<double>, kNumQuads> stress;
  model.CalcFirstPiolaStress(cache_entry, &stress);
  EXPECT_EQ(stress, analytic_stress);
}

GTEST_TEST(FixedSizeLinearConstitutiveModelTest, PIsDerivativeOfPsi) {
  const FixedSizeLinearConstitutiveModel<AutoDiffXd, kNumQuads> model(100.0,
                                                                      0.3);
  FixedSizeLinearConstitutiveModelCacheEntry<AutoDiffXd, kNumQuads> cache_entry(
      kDummyElementIndex);
  const std::array<Matrix3<AutoDiffXd>, kNumQuads> Fs =
      MakeDeformationGradients();
  // P should be derivative of Psi.
  cache_entry.UpdateCacheEntry(Fs);
  std::array<AutoDiffXd, kNumQuads> energy;
  model.CalcElasticEnergyDensity(cache_entry, &energy);
  std::array<Matrix3<AutoDiffXd>, kNumQuads> P;
  model.CalcFirstPiolaStress(cache_entry, &P);
  for (int i = 0; i < kNumQuads; ++i) {
    EXPECT_TRUE(CompareMatrices(
        Eigen::Map<const Matrix3<double>>(energy[i].derivatives().data(), 3, 3),
        P[i]));
  }
}
// TODO(xuchenhan-tri): This test applies to all ConstitutiveModels. Consider
// moving it to ConstitutiveModelTest.
GTEST_TEST(LinearConstitutiveModelTest, dPdFIsDerivativeOfP) {
  const FixedSizeLinearConstitutiveModel<AutoDiffXd, kNumQuads> model(100.0,
                                                                      0.3);
  FixedSizeLinearConstitutiveModelCacheEntry<AutoDiffXd, kNumQuads> cache_entry(
      kDummyElementIndex);
  const std::array<Matrix3<AutoDiffXd>, kNumQuads> Fs =
      MakeDeformationGradients();
  cache_entry.UpdateCacheEntry(Fs);
  std::array<Matrix3<AutoDiffXd>, kNumQuads> P;
  model.CalcFirstPiolaStress(cache_entry, &P);
  std::array<Eigen::Matrix<AutoDiffXd, 9, 9>, kNumQuads> dPdF;
  model.CalcFirstPiolaStressDerivative(cache_entry, &dPdF);
  for (int q = 0; q < kNumQuads; ++q) {
    for (int i = 0; i < kSpaceDimension; ++i) {
      for (int j = 0; j < kSpaceDimension; ++j) {
        Matrix3<double> dPijdF;
        for (int k = 0; k < kSpaceDimension; ++k) {
          for (int l = 0; l < kSpaceDimension; ++l) {
            dPijdF(k, l) = dPdF[q](3 * j + i, 3 * l + k).value();
          }
        }
        EXPECT_TRUE(CompareMatrices(Eigen::Map<const Matrix3<double>>(
                                        P[q](i, j).derivatives().data(), 3, 3),
                                    dPijdF));
      }
    }
  }
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
