#include "drake/multibody/fem/test/constitutive_model_test_utilities.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/constitutive_model.h"
#include "drake/multibody/fem/corotated_model.h"
#include "drake/multibody/fem/linear_constitutive_model.h"
#include "drake/multibody/fem/linear_corotated_model.h"
#include "drake/multibody/fem/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

using Eigen::Matrix3d;

namespace {

/* Creates an array of arbitrary autodiff deformation gradients. */
Matrix3<AutoDiffXd> MakeDeformationGradientsWithDerivatives() {
  /* Create an arbitrary AutoDiffXd deformation. */
  Matrix3d F;
  // clang-format off
  F << 0.18, 0.63, 0.54,
       0.13, 0.92, 0.17,
       0.03, 0.86, 0.85;
  // clang-format on
  const Matrix3d deformation_gradients{F};
  Matrix3<AutoDiffXd> deformation_gradients_autodiff;
  const Eigen::Matrix<double, 9, Eigen::Dynamic> derivatives(
      Eigen::Matrix<double, 9, 9>::Identity());
  const auto F_autodiff_flat =
      math::InitializeAutoDiff(Eigen::Map<const Eigen::Matrix<double, 9, 1>>(
                                   deformation_gradients.data(), 9),
                               derivatives);
  deformation_gradients_autodiff =
      Eigen::Map<const Matrix3<AutoDiffXd>>(F_autodiff_flat.data(), 3, 3);
  return deformation_gradients_autodiff;
}

Matrix3<AutoDiffXd> MakeDeformationGradientsWithoutDerivatives() {
  /* Create an arbitrary AutoDiffXd deformation. */
  Matrix3<AutoDiffXd> F;
  // clang-format off
  F << 0.18, 0.63, 0.54,
       0.13, 0.92, 0.17,
       0.03, 0.86, 0.85;
  // clang-format on
  return F;
}

}  // namespace

/* Tests the constructors correctly initializes St.Venant-Kichhoff like
constitutive models and rejects invalid Young's modulus and Poisson's ratio.
@tparam Model    Must be instantiations of LinearConstitutiveModel,
LinearCorotatedModel, or CorotatedModel. */
template <class Model>
void TestParameters() {
  using T = typename Model::T;
  const T kYoungsModulus = 100.0;
  const T kPoissonRatio = 0.25;
  const Model model(kYoungsModulus, kPoissonRatio);
  const T kExpectedMu = 40.0;
  const T kExpectedLambda = 40.0;
  EXPECT_EQ(model.youngs_modulus(), kYoungsModulus);
  EXPECT_EQ(model.poissons_ratio(), kPoissonRatio);
  EXPECT_EQ(model.shear_modulus(), kExpectedMu);
  EXPECT_EQ(model.lame_first_parameter(), kExpectedLambda);

  DRAKE_EXPECT_THROWS_MESSAGE((Model(-1.0, 0.25)), "Young's modulus must .*");

  DRAKE_EXPECT_THROWS_MESSAGE((Model(100.0, 0.5)),
                              "Poisson's ratio must be in .*");

  DRAKE_EXPECT_THROWS_MESSAGE((Model(100.0, 0.6)),
                              "Poisson's ratio must be in .*");

  DRAKE_EXPECT_THROWS_MESSAGE((Model(100.0, -1.0)),
                              "Poisson's ratio must be in .*");

  DRAKE_EXPECT_THROWS_MESSAGE((Model(100.0, -1.1)),
                              "Poisson's ratio must be in .*");
}

/* Tests that the energy density and the stress are zero at the undeformed
state.
@tparam Model    Must be instantiations of LinearConstitutiveModel,
LinearCorotatedModel, or CorotatedModel. */
template <class Model>
void TestUndeformedState() {
  using T = typename Model::T;

  const T kYoungsModulus = 100.0;
  const T kPoissonRatio = 0.25;
  const Model model(kYoungsModulus, kPoissonRatio);
  typename Model::Traits::Data data;
  const Matrix3<T> F{Matrix3<T>::Identity()};
  const Matrix3<T> F0{Matrix3<T>::Identity()};
  data.UpdateData(F, F0);
  /* At the undeformed state, the energy density should be zero. */
  const T analytic_energy_density{0};
  /* At the undeformed state, the stress should be zero. */
  const Matrix3<T> analytic_stress{Matrix3d::Zero()};
  T energy_density;
  model.CalcElasticEnergyDensity(data, &energy_density);
  EXPECT_EQ(energy_density, analytic_energy_density);
  Matrix3<T> stress;
  model.CalcFirstPiolaStress(data, &stress);
  EXPECT_EQ(stress, analytic_stress);
}

/* Tests that the energy density and the stress are consistent by verifying the
stress matches the derivative of energy density produced by automatic
differentiation.
@tparam Model    Must be instantiations of LinearConstitutiveModel,
LinearCorotatedModel, or CorotatedModel. */
template <class Model>
void TestPIsDerivativeOfPsi() {
  const double kTolerance = 1e-12;
  constexpr double kYoungsModulus = 100.0;
  constexpr double kPoissonRatio = 0.3;
  const Model model(kYoungsModulus, kPoissonRatio);
  typename Model::Traits::Data data;
  const Matrix3<AutoDiffXd> deformation_gradients =
      MakeDeformationGradientsWithDerivatives();
  const Matrix3<AutoDiffXd> previous_step_deformation_gradients =
      MakeDeformationGradientsWithoutDerivatives();
  data.UpdateData(deformation_gradients, previous_step_deformation_gradients);
  AutoDiffXd energy;
  model.CalcElasticEnergyDensity(data, &energy);
  Matrix3<AutoDiffXd> P;
  model.CalcFirstPiolaStress(data, &P);
  EXPECT_TRUE(CompareMatrices(
      Eigen::Map<const Matrix3d>(energy.derivatives().data(), 3, 3), P,
      CalcConditionNumberOfInvertibleMatrix<AutoDiffXd>(P) * kTolerance));
}

/* Tests that the stress and the stress derivatives are consistent by verifying
the handcrafted derivative matches that produced by automatic differentiation.
@tparam Model    Must be instantiations of LinearConstitutiveModel,
LinearCorotatedModel, or CorotatedModel. */
template <class Model>
void TestdPdFIsDerivativeOfP() {
  constexpr int kSpaceDimension = 3;
  const double kTolerance = 1e-12;
  constexpr double kYoungsModulus = 100.0;
  constexpr double kPoissonRatio = 0.3;
  const Model model(kYoungsModulus, kPoissonRatio);
  typename Model::Traits::Data data;
  const Matrix3<AutoDiffXd> deformation_gradients =
      MakeDeformationGradientsWithDerivatives();
  const Matrix3<AutoDiffXd> previous_step_deformation_gradients =
      MakeDeformationGradientsWithoutDerivatives();
  data.UpdateData(deformation_gradients, previous_step_deformation_gradients);
  Matrix3<AutoDiffXd> P;
  model.CalcFirstPiolaStress(data, &P);
  Eigen::Matrix<AutoDiffXd, 9, 9> dPdF;
  model.CalcFirstPiolaStressDerivative(data, &dPdF);
  for (int i = 0; i < kSpaceDimension; ++i) {
    for (int j = 0; j < kSpaceDimension; ++j) {
      Matrix3d dPijdF;
      for (int k = 0; k < kSpaceDimension; ++k) {
        for (int l = 0; l < kSpaceDimension; ++l) {
          dPijdF(k, l) = dPdF(3 * j + i, 3 * l + k).value();
        }
      }
      EXPECT_TRUE(CompareMatrices(
          Eigen::Map<const Matrix3d>(P(i, j).derivatives().data(), 3, 3),
          dPijdF,
          CalcConditionNumberOfInvertibleMatrix<AutoDiffXd>(P) * kTolerance));
    }
  }
}

template void TestParameters<LinearConstitutiveModel<double>>();
template void TestParameters<LinearConstitutiveModel<AutoDiffXd>>();
template void TestUndeformedState<LinearConstitutiveModel<double>>();
template void TestUndeformedState<LinearConstitutiveModel<AutoDiffXd>>();
template void TestPIsDerivativeOfPsi<LinearConstitutiveModel<AutoDiffXd>>();
template void TestdPdFIsDerivativeOfP<LinearConstitutiveModel<AutoDiffXd>>();

template void TestParameters<CorotatedModel<double>>();
template void TestParameters<CorotatedModel<AutoDiffXd>>();
template void TestUndeformedState<CorotatedModel<double>>();
template void TestUndeformedState<CorotatedModel<AutoDiffXd>>();
template void TestPIsDerivativeOfPsi<CorotatedModel<AutoDiffXd>>();
template void TestdPdFIsDerivativeOfP<CorotatedModel<AutoDiffXd>>();

template void TestParameters<LinearCorotatedModel<double>>();
template void TestParameters<LinearCorotatedModel<AutoDiffXd>>();
template void TestUndeformedState<LinearCorotatedModel<double>>();
template void TestUndeformedState<LinearCorotatedModel<AutoDiffXd>>();
template void TestPIsDerivativeOfPsi<LinearCorotatedModel<AutoDiffXd>>();
template void TestdPdFIsDerivativeOfP<LinearCorotatedModel<AutoDiffXd>>();

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
