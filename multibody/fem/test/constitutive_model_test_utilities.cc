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
#include "drake/multibody/fem/neohookean_model.h"

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
LinearCorotatedModel, NeoHookeanModel, or CorotatedModel. */
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
                 LinearCorotatedModel, NeoHookeanModel or CorotatedModel. */
template <class Model>
void TestUndeformedState(bool nonzero_rest_state) {
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
  if (!nonzero_rest_state) {
    T energy_density;
    model.CalcElasticEnergyDensity(data, &energy_density);
    EXPECT_EQ(energy_density, analytic_energy_density);
  }
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
  math::internal::FourthOrderTensor<AutoDiffXd> dPdF;
  model.CalcFirstPiolaStressDerivative(data, &dPdF);
  for (int i = 0; i < kSpaceDimension; ++i) {
    for (int j = 0; j < kSpaceDimension; ++j) {
      Matrix3d dPijdF;
      for (int k = 0; k < kSpaceDimension; ++k) {
        for (int l = 0; l < kSpaceDimension; ++l) {
          dPijdF(k, l) = dPdF(i, j, k, l).value();
        }
      }
      EXPECT_TRUE(CompareMatrices(
          Eigen::Map<const Matrix3d>(P(i, j).derivatives().data(), 3, 3),
          dPijdF,
          CalcConditionNumberOfInvertibleMatrix<AutoDiffXd>(P) * kTolerance));
    }
  }
}

template <class Model>
void TestSpdness() {
  using T = typename Model::T;
  const T kYoungsModulus = 100.0;
  const T kPoissonRatio = 0.25;
  const Model model(kYoungsModulus, kPoissonRatio);
  typename Model::Traits::Data data;
  Matrix3<T> F;
  // clang-format off
  F << 0.18, 0.63, 0.54,
       0.13, 0.92, 0.17,
       0.03, 0.86, 0.85;
  // clang-format on
  const Matrix3<T> F0 = F;
  data.UpdateData(F, F0);
  math::internal::FourthOrderTensor<T> unfiltered;
  math::internal::FourthOrderTensor<T> filtered;

  model.CalcFirstPiolaStressDerivative(data, &unfiltered);
  model.CalcFilteredHessian(data, &filtered);

  using Matrix9T = Eigen::Matrix<T, 9, 9>;
  using Vector9d = Eigen::Matrix<double, 9, 1>;
  /* Check that the filtered Hessian is SPD. */
  const Matrix9T filtered_matrix = filtered.data();
  const Eigen::SelfAdjointEigenSolver<Matrix9T> eigensolver(filtered_matrix);
  ASSERT_TRUE(eigensolver.info() == Eigen::Success);
  const Vector9d eigenvalues = math::DiscardGradient(eigensolver.eigenvalues());
  const Eigen::SelfAdjointEigenSolver<Matrix9T> eigensolver2(unfiltered.data());
  ASSERT_TRUE(eigensolver2.info() == Eigen::Success);
  const Vector9d unfiltered_eigenvalues =
      math::DiscardGradient(eigensolver2.eigenvalues());
  /* For the filtered Hessian, the eigenvalues should either be the same of the
   unfiltered hessian and positive or clamped at zero. */
  const double kTol = 1e-12;
  for (int i = 0; i < 9; ++i) {
    const double eigenvalue = eigenvalues(i);
    const double unfiltered_eigenvalue = unfiltered_eigenvalues(i);
    EXPECT_GE(eigenvalue, -kTol);
    if (eigenvalue > kTol) {
      EXPECT_NEAR(eigenvalue, unfiltered_eigenvalue, kTol);
    } else {
      EXPECT_NEAR(eigenvalue, 0, kTol);
    }
  }
  /* Check that the filtering is actively doing something by checking that the
   filtered Hessian is not equal to the unfiltered Hessian. */
  if (!model.is_linear) {
    EXPECT_FALSE(CompareMatrices(filtered_matrix, unfiltered.data(), 0.1));
  }
}

template void TestParameters<LinearConstitutiveModel<double>>();
template void TestParameters<LinearConstitutiveModel<AutoDiffXd>>();
template void TestUndeformedState<LinearConstitutiveModel<double>>(bool);
template void TestUndeformedState<LinearConstitutiveModel<AutoDiffXd>>(bool);
template void TestSpdness<LinearConstitutiveModel<double>>();
template void TestSpdness<LinearConstitutiveModel<AutoDiffXd>>();
template void TestPIsDerivativeOfPsi<LinearConstitutiveModel<AutoDiffXd>>();
template void TestdPdFIsDerivativeOfP<LinearConstitutiveModel<AutoDiffXd>>();

template void TestParameters<CorotatedModel<double>>();
template void TestParameters<CorotatedModel<AutoDiffXd>>();
template void TestUndeformedState<CorotatedModel<double>>(bool);
template void TestUndeformedState<CorotatedModel<AutoDiffXd>>(bool);
template void TestSpdness<CorotatedModel<double>>();
template void TestSpdness<CorotatedModel<AutoDiffXd>>();
template void TestPIsDerivativeOfPsi<CorotatedModel<AutoDiffXd>>();
template void TestdPdFIsDerivativeOfP<CorotatedModel<AutoDiffXd>>();

template void TestParameters<LinearCorotatedModel<double>>();
template void TestParameters<LinearCorotatedModel<AutoDiffXd>>();
template void TestUndeformedState<LinearCorotatedModel<double>>(bool);
template void TestUndeformedState<LinearCorotatedModel<AutoDiffXd>>(bool);
template void TestSpdness<LinearCorotatedModel<double>>();
template void TestSpdness<LinearCorotatedModel<AutoDiffXd>>();
template void TestPIsDerivativeOfPsi<LinearCorotatedModel<AutoDiffXd>>();
template void TestdPdFIsDerivativeOfP<LinearCorotatedModel<AutoDiffXd>>();

template void TestParameters<NeoHookeanModel<double>>();
template void TestParameters<NeoHookeanModel<AutoDiffXd>>();
template void TestUndeformedState<NeoHookeanModel<double>>(bool);
template void TestUndeformedState<NeoHookeanModel<AutoDiffXd>>(bool);
template void TestSpdness<NeoHookeanModel<double>>();
template void TestSpdness<NeoHookeanModel<AutoDiffXd>>();
template void TestPIsDerivativeOfPsi<NeoHookeanModel<AutoDiffXd>>();
template void TestdPdFIsDerivativeOfP<NeoHookeanModel<AutoDiffXd>>();

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
