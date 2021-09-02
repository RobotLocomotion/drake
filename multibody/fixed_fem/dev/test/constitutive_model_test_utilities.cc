#include "drake/multibody/fixed_fem/dev/test/constitutive_model_test_utilities.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/corotated_model.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/test/test_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

using Eigen::Matrix3d;
const double kTolerance = 1e-12;

/* Creates an array of arbitrary autodiff deformation gradients. */
template <int num_locations>
std::array<Matrix3<AutoDiffXd>, num_locations> MakeDeformationGradients() {
  /* Create an arbitrary AutoDiffXd deformation. */
  Matrix3d F;
  // clang-format off
  F << 0.18, 0.63, 0.54,
       0.13, 0.92, 0.17,
       0.03, 0.86, 0.85;
  // clang-format on
  const std::array<Matrix3d, num_locations> deformation_gradients{F};
  std::array<Matrix3<AutoDiffXd>, num_locations> deformation_gradients_autodiff;
  const Eigen::Matrix<double, 9, Eigen::Dynamic> derivatives(
      Eigen::Matrix<double, 9, 9>::Identity());
  for (int i = 0; i < num_locations; ++i) {
    const auto F_autodiff_flat =
        math::InitializeAutoDiff(Eigen::Map<const Eigen::Matrix<double, 9, 1>>(
                                     deformation_gradients[i].data(), 9),
                                 derivatives);
    deformation_gradients_autodiff[i] =
        Eigen::Map<const Matrix3<AutoDiffXd>>(F_autodiff_flat.data(), 3, 3);
  }
  return deformation_gradients_autodiff;
}

/* Tests the constructors correctly initializes St.Venant-Kichhoff like
constitutive models and rejects invalid Young's modulus and poisson ratio.
@tparam Model    Must be instantiations of LinearConstitutiveModel or
CorotatedModel. */
template <class Model>
void TestParameters() {
  using T = typename Model::T;
  const T kYoungsModulus = 100.0;
  const T kPoissonRatio = 0.25;
  const Model model(kYoungsModulus, kPoissonRatio);
  const T kExpectedMu = 40.0;
  const T kExpectedLambda = 40.0;
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
  constexpr int num_locations = Model::Data::num_locations;
  using T = typename Model::T;

  const T kYoungsModulus = 100.0;
  const T kPoissonRatio = 0.25;
  const Model model(kYoungsModulus, kPoissonRatio);
  typename Model::Traits::Data data;
  const std::array<Matrix3<T>, num_locations> F{Matrix3<T>::Identity()};
  data.UpdateData(F);
  /* At the undeformed state, the energy density should be zero. */
  const std::array<T, num_locations> analytic_energy_density{0};
  /* At the undeformed state, the stress should be zero. */
  const std::array<Matrix3<T>, num_locations> analytic_stress{Matrix3d::Zero()};
  std::array<T, num_locations> energy_density;
  model.CalcElasticEnergyDensity(data, &energy_density);
  EXPECT_EQ(energy_density, analytic_energy_density);
  std::array<Matrix3<T>, num_locations> stress;
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
  constexpr int num_locations = Model::Data::num_locations;
  constexpr double kYoungsModulus = 100.0;
  constexpr double kPoissonRatio = 0.3;
  const Model model(kYoungsModulus, kPoissonRatio);
  typename Model::Traits::Data data;
  const std::array<Matrix3<AutoDiffXd>, num_locations> deformation_gradients =
      MakeDeformationGradients<num_locations>();
  data.UpdateData(deformation_gradients);
  std::array<AutoDiffXd, num_locations> energy;
  model.CalcElasticEnergyDensity(data, &energy);
  std::array<Matrix3<AutoDiffXd>, num_locations> P;
  model.CalcFirstPiolaStress(data, &P);
  for (int i = 0; i < num_locations; ++i) {
    EXPECT_TRUE(CompareMatrices(
        Eigen::Map<const Matrix3d>(energy[i].derivatives().data(), 3, 3), P[i],
        fem::test::CalcConditionNumber<AutoDiffXd>(P[i]) * kTolerance));
  }
}

/* Tests that the stress and the stress derivatives are consistent by verifying
the handcrafted derivative matches that produced by automatic differentiation.
@tparam Model    Must be AutoDiffXd instantiations of LinearConstitutiveModel or
CorotatedModel. */
template <class Model>
void TestdPdFIsDerivativeOfP() {
  constexpr int num_locations = Model::Data::num_locations;
  constexpr double kYoungsModulus = 100.0;
  constexpr double kPoissonRatio = 0.3;
  const Model model(kYoungsModulus, kPoissonRatio);
  typename Model::Traits::Data data;
  const std::array<Matrix3<AutoDiffXd>, num_locations> deformation_gradients =
      MakeDeformationGradients<num_locations>();
  data.UpdateData(deformation_gradients);
  std::array<Matrix3<AutoDiffXd>, num_locations> P;
  model.CalcFirstPiolaStress(data, &P);
  std::array<Eigen::Matrix<AutoDiffXd, 9, 9>, num_locations> dPdF;
  model.CalcFirstPiolaStressDerivative(data, &dPdF);
  for (int q = 0; q < num_locations; ++q) {
    for (int i = 0; i < kSpaceDimension; ++i) {
      for (int j = 0; j < kSpaceDimension; ++j) {
        Matrix3d dPijdF;
        for (int k = 0; k < kSpaceDimension; ++k) {
          for (int l = 0; l < kSpaceDimension; ++l) {
            dPijdF(k, l) = dPdF[q](3 * j + i, 3 * l + k).value();
          }
        }
        EXPECT_TRUE(CompareMatrices(
            Eigen::Map<const Matrix3d>(P[q](i, j).derivatives().data(), 3, 3),
            dPijdF,
            fem::test::CalcConditionNumber<AutoDiffXd>(P[q]) * kTolerance));
      }
    }
  }
}

template void TestParameters<LinearConstitutiveModel<double, 1>>();
template void TestParameters<CorotatedModel<double, 1>>();
template void TestParameters<LinearConstitutiveModel<AutoDiffXd, 1>>();
template void TestParameters<CorotatedModel<AutoDiffXd, 1>>();

template void TestUndeformedState<LinearConstitutiveModel<double, 1>>();
template void TestUndeformedState<CorotatedModel<double, 1>>();
template void TestUndeformedState<LinearConstitutiveModel<AutoDiffXd, 1>>();
template void TestUndeformedState<CorotatedModel<AutoDiffXd, 1>>();

template void TestPIsDerivativeOfPsi<LinearConstitutiveModel<AutoDiffXd, 1>>();
template void TestPIsDerivativeOfPsi<CorotatedModel<AutoDiffXd, 1>>();

template void TestdPdFIsDerivativeOfP<LinearConstitutiveModel<AutoDiffXd, 1>>();
template void TestdPdFIsDerivativeOfP<CorotatedModel<AutoDiffXd, 1>>();

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
