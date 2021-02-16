#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fixed_fem/dev/corotated_model.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/test/test_utilities.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace {
const ElementIndex kDummyElementIndex(0);
constexpr int kNumQuads = 1;
const double kTol = 1e-12;

// Creates a vector of arbitrary deformation gradients.
std::array<Matrix3<AutoDiffXd>, kNumQuads> MakeDeformationGradients() {
  // Create an arbitrary AutoDiffXd deformation.
  Matrix3<double> F;
  // clang-format off
  F << 0.18, 0.63, 0.54,
       0.13, 0.92, 0.17,
       0.03, 0.86, 0.85;
  // clang-format on
  const std::array<Matrix3<double>, kNumQuads> deformation_gradients{F};
  std::array<Matrix3<AutoDiffXd>, kNumQuads> deformation_gradients_autodiff;
  const Eigen::Matrix<double, 9, Eigen::Dynamic> gradient(
      Eigen::Matrix<double, 9, 9>::Identity());
  for (int i = 0; i < kNumQuads; ++i) {
    const auto F_autodiff_flat = math::initializeAutoDiffGivenGradientMatrix(
        Eigen::Map<const Eigen::Matrix<double, 9, 1>>(
            deformation_gradients[i].data(), 9),
        gradient);
    deformation_gradients_autodiff[i] =
        Eigen::Map<const Matrix3<AutoDiffXd>>(F_autodiff_flat.data(), 3, 3);
  }
  return deformation_gradients_autodiff;
}

/* Tests the constructor and the accessors.
@tparam Model    Must be instantiations of LinearConstitutiveModel or
CorotatedModel. */
template <class Model>
void TestParameters() {
  const Model model(100.0, 0.25);
  const double mu = 40.0;
  const double lambda = 40.0;
  EXPECT_EQ(model.youngs_modulus(), 100.0);
  EXPECT_EQ(model.poisson_ratio(), 0.25);
  EXPECT_EQ(model.shear_modulus(), mu);
  EXPECT_EQ(model.lame_first_parameter(), lambda);

  DRAKE_EXPECT_THROWS_MESSAGE((Model(-1.0, 0.25)), std::logic_error,
                              "Young's modulus must be nonnegative.");

  DRAKE_EXPECT_THROWS_MESSAGE((Model(100.0, 0.5)), std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");

  DRAKE_EXPECT_THROWS_MESSAGE((Model(100.0, 0.6)), std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");

  DRAKE_EXPECT_THROWS_MESSAGE((Model(100.0, -1.0)), std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");

  DRAKE_EXPECT_THROWS_MESSAGE((Model(100.0, -1.1)), std::logic_error,
                              "Poisson ratio must be in \\(-1, 0.5\\).");
}

/* Tests that the energy density and the stress are zero at the undeformed
state.
@tparam Model    Must be instantiations of LinearConstitutiveModel or
CorotatedModel. */
template <class Model>
void TestUndeformedState() {
  const Model model(100.0, 0.25);
  typename Model::Traits::DeformationGradientCacheEntryType cache_entry;
  const std::array<Matrix3<double>, kNumQuads> F{Matrix3<double>::Identity()};
  cache_entry.UpdateCacheEntry(F);
  // At the undeformed state, the energy density should be zero.
  const std::array<double, kNumQuads> analytic_energy_density{0};
  // At the undeformed state, the stress should be zero.
  const std::array<Matrix3<double>, kNumQuads> analytic_stress{
      Matrix3<double>::Zero()};
  std::array<double, kNumQuads> energy_density;
  model.CalcElasticEnergyDensity(cache_entry, &energy_density);
  EXPECT_EQ(energy_density, analytic_energy_density);
  std::array<Matrix3<double>, kNumQuads> stress;
  model.CalcFirstPiolaStress(cache_entry, &stress);
  EXPECT_EQ(stress, analytic_stress);
}

/* Tests that the energy density and the stress are consistent by verifying the
stress matches the derivative of energy density produced by automatic
differentiation.
@tparam Model    Must be AutoDiffXd instantiations of LinearConstitutiveModel or
CorotatedModel. */
template <class Model>
void TestPIsDerivativeOfPsi() {
  const Model model(100.0, 0.3);
  typename Model::Traits::DeformationGradientCacheEntryType cache_entry;
  const std::array<Matrix3<AutoDiffXd>, kNumQuads> deformation_gradients =
      MakeDeformationGradients();
  // P should be derivative of Psi.
  cache_entry.UpdateCacheEntry(deformation_gradients);
  std::array<AutoDiffXd, kNumQuads> energy;
  model.CalcElasticEnergyDensity(cache_entry, &energy);
  std::array<Matrix3<AutoDiffXd>, kNumQuads> P;
  model.CalcFirstPiolaStress(cache_entry, &P);
  for (int i = 0; i < kNumQuads; ++i) {
    EXPECT_TRUE(CompareMatrices(
        Eigen::Map<const Matrix3<double>>(energy[i].derivatives().data(), 3, 3),
        P[i], test::CalcConditionNumber<AutoDiffXd>(P[i]) * kTol));
  }
}

/* Tests that the stress and the stress derivatives are consistent by verifying
the handcrafted derivative matches that produced by automatic differentiation.
@tparam Model    Must be AutoDiffXd instantiations of LinearConstitutiveModel or
CorotatedModel. */
template <class Model>
void TestdPdFIsDerivativeOfP() {
  const Model model(100.0, 0.3);
  typename Model::Traits::DeformationGradientCacheEntryType cache_entry;
  const std::array<Matrix3<AutoDiffXd>, kNumQuads> deformation_gradients =
      MakeDeformationGradients();
  cache_entry.UpdateCacheEntry(deformation_gradients);
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
        EXPECT_TRUE(CompareMatrices(
            Eigen::Map<const Matrix3<double>>(P[q](i, j).derivatives().data(),
                                              3, 3),
            dPijdF, test::CalcConditionNumber<double>(dPijdF) * kTol));
      }
    }
  }
}

GTEST_TEST(LinearConstitutiveModelTest, Parameters) {
  TestParameters<LinearConstitutiveModel<double, kNumQuads>>();
  TestParameters<CorotatedModel<double, kNumQuads>>();
}

GTEST_TEST(LinearConstitutiveModelTest, UndeformedState) {
  TestUndeformedState<LinearConstitutiveModel<double, kNumQuads>>();
  TestUndeformedState<CorotatedModel<double, kNumQuads>>();
}

GTEST_TEST(LinearConstitutiveModelTest, PIsDerivativeOfPsi) {
  TestPIsDerivativeOfPsi<LinearConstitutiveModel<AutoDiffXd, kNumQuads>>();
  TestPIsDerivativeOfPsi<CorotatedModel<AutoDiffXd, kNumQuads>>();
}

GTEST_TEST(LinearConstitutiveModelTest, dPdFIsDerivativeOfP) {
  TestdPdFIsDerivativeOfP<LinearConstitutiveModel<AutoDiffXd, kNumQuads>>();
  TestdPdFIsDerivativeOfP<CorotatedModel<AutoDiffXd, kNumQuads>>();
}
}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
