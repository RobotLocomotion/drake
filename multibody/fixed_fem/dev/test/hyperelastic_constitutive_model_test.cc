#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fixed_fem/dev/corotated_model.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/test/test_utilities.h"

// TODO(xuchenhan-tri): Rename this file to constitutive_model_test.cc.
namespace drake {
namespace multibody {
namespace fem {
namespace internal {

using Eigen::Matrix3d;

/* Minimal required data type to be used in the derived constitutive model
 traits. */
struct DummyData {
  static constexpr int num_locations = 2;
};

struct InvalidModelTraits {
  using Scalar = double;
  using Data = DummyData;
};

/* An invalid ConstitutiveModel that is missing the
 DoCalcElasticEnergyDensity(), DoCalcFirstPiolaStress(), and
 DoCalcFirstPiolaStressDerivative() methods. This class is used to test that an
 exception is thrown in the case where the derived class of ConstitutiveModel
 doesn't shadow these methods. */
class InvalidModel
    : public ConstitutiveModel<InvalidModel, InvalidModelTraits> {};

namespace {
GTEST_TEST(ConstitutiveModelTest, InvalidModel) {
  const InvalidModel model;
  const DummyData data;
  std::array<double, DummyData::num_locations> energy_density;
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.CalcElasticEnergyDensity(data, &energy_density), std::exception,
      fmt::format("The derived class {} must provide a shadow definition of "
                  "DoCalcElasticEnergyDensity.. to be correct.",
                  NiceTypeName::Get(model)));

  std::array<Matrix3d, DummyData::num_locations> P;
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.CalcFirstPiolaStress(data, &P), std::exception,
      fmt::format("The derived class {} must provide a shadow definition of "
                  "DoCalcFirstPiolaStress.. to be correct.",
                  NiceTypeName::Get(model)));

  std::array<Eigen::Matrix<double, 9, 9>, DummyData::num_locations> dPdF;
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.CalcFirstPiolaStressDerivative(data, &dPdF), std::exception,
      fmt::format("The derived class {} must provide a shadow definition of "
                  "DoCalcFirstPiolaStressDerivative.. to be correct.",
                  NiceTypeName::Get(model)));
}

constexpr int kNumLocations = 1;
const double kTol = 1e-12;

// Creates an array of arbitrary autodiff deformation gradients.
std::array<Matrix3<AutoDiffXd>, kNumLocations> MakeDeformationGradients() {
  // Create an arbitrary AutoDiffXd deformation.
  Matrix3<double> F;
  // clang-format off
  F << 0.18, 0.63, 0.54,
       0.13, 0.92, 0.17,
       0.03, 0.86, 0.85;
  // clang-format on
  const std::array<Matrix3<double>, kNumLocations> deformation_gradients{F};
  std::array<Matrix3<AutoDiffXd>, kNumLocations> deformation_gradients_autodiff;
  const Eigen::Matrix<double, 9, Eigen::Dynamic> derivatives(
      Eigen::Matrix<double, 9, 9>::Identity());
  for (int i = 0; i < kNumLocations; ++i) {
    const auto F_autodiff_flat = math::initializeAutoDiffGivenGradientMatrix(
        Eigen::Map<const Eigen::Matrix<double, 9, 1>>(
            deformation_gradients[i].data(), 9),
        derivatives);
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
  constexpr double kYoungsModulus = 100.0;
  constexpr double kPoissonRatio = 0.25;
  const Model model(kYoungsModulus, kPoissonRatio);
  constexpr double kExpectedMu = 40.0;
  constexpr double kExpectedLambda = 40.0;
  EXPECT_EQ(model.youngs_modulus(), kYoungsModulus);
  EXPECT_EQ(model.poisson_ratio(), kPoissonRatio);
  EXPECT_EQ(model.shear_modulus(), kExpectedMu);
  EXPECT_EQ(model.lame_first_parameter(), kExpectedLambda);

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
  constexpr double kYoungsModulus = 100.0;
  constexpr double kPoissonRatio = 0.25;
  const Model model(kYoungsModulus, kPoissonRatio);
  typename Model::Traits::Data data;
  const std::array<Matrix3<double>, kNumLocations> F{
      Matrix3<double>::Identity()};
  data.UpdateData(F);
  // At the undeformed state, the energy density should be zero.
  const std::array<double, kNumLocations> analytic_energy_density{0};
  // At the undeformed state, the stress should be zero.
  const std::array<Matrix3<double>, kNumLocations> analytic_stress{
      Matrix3<double>::Zero()};
  std::array<double, kNumLocations> energy_density;
  model.CalcElasticEnergyDensity(data, &energy_density);
  EXPECT_EQ(energy_density, analytic_energy_density);
  std::array<Matrix3<double>, kNumLocations> stress;
  model.CalcFirstPiolaStress(data, &stress);
  EXPECT_EQ(stress, analytic_stress);
}

/* Tests that the energy density and the stress are consistent by verifying the
stress matches the derivative of energy density produced by automatic
differentiation.
@tparam Model    Must be AutoDiffXd instantiations of LinearConstitutiveModel or
CorotatedModel. */
template <class Model>
void TestPIsDerivativeOfPsi() {
  constexpr double kYoungsModulus = 100.0;
  constexpr double kPoissonRatio = 0.3;
  const Model model(kYoungsModulus, kPoissonRatio);
  typename Model::Traits::Data data;
  const std::array<Matrix3<AutoDiffXd>, kNumLocations> deformation_gradients =
      MakeDeformationGradients();
  data.UpdateData(deformation_gradients);
  std::array<AutoDiffXd, kNumLocations> energy;
  model.CalcElasticEnergyDensity(data, &energy);
  std::array<Matrix3<AutoDiffXd>, kNumLocations> P;
  model.CalcFirstPiolaStress(data, &P);
  for (int i = 0; i < kNumLocations; ++i) {
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
  constexpr double kYoungsModulus = 100.0;
  constexpr double kPoissonRatio = 0.3;
  const Model model(kYoungsModulus, kPoissonRatio);
  typename Model::Traits::Data data;
  const std::array<Matrix3<AutoDiffXd>, kNumLocations> deformation_gradients =
      MakeDeformationGradients();
  data.UpdateData(deformation_gradients);
  std::array<Matrix3<AutoDiffXd>, kNumLocations> P;
  model.CalcFirstPiolaStress(data, &P);
  std::array<Eigen::Matrix<AutoDiffXd, 9, 9>, kNumLocations> dPdF;
  model.CalcFirstPiolaStressDerivative(data, &dPdF);
  for (int q = 0; q < kNumLocations; ++q) {
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
  TestParameters<LinearConstitutiveModel<double, kNumLocations>>();
  TestParameters<CorotatedModel<double, kNumLocations>>();
}

GTEST_TEST(LinearConstitutiveModelTest, UndeformedState) {
  TestUndeformedState<LinearConstitutiveModel<double, kNumLocations>>();
  TestUndeformedState<CorotatedModel<double, kNumLocations>>();
}

GTEST_TEST(LinearConstitutiveModelTest, PIsDerivativeOfPsi) {
  TestPIsDerivativeOfPsi<LinearConstitutiveModel<AutoDiffXd, kNumLocations>>();
  TestPIsDerivativeOfPsi<CorotatedModel<AutoDiffXd, kNumLocations>>();
}

GTEST_TEST(LinearConstitutiveModelTest, dPdFIsDerivativeOfP) {
  TestdPdFIsDerivativeOfP<LinearConstitutiveModel<AutoDiffXd, kNumLocations>>();
  TestdPdFIsDerivativeOfP<CorotatedModel<AutoDiffXd, kNumLocations>>();
}
}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
