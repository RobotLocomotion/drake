#include "drake/fem/hyperelastic_constitutive_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace fem {
namespace {
using Hessian = Eigen::Matrix<double, 9, 9>;

/*
 Implements a HyperelasticConsitutiveModel for testing purposes only.
 We want to test that HyperelasticConstitutiveModel for:
   - Storing elastic and damping parameters and calculating Lamé parameters.
   - NVI methods successfully call the DoFoo() methods.
 Note that the TestConstitutiveModel is NOT necessarily valid, meaning that
 the First Piola Stress returned is NOT necessarily the derivative of the
 energy density, nor is the First Piola derivative necessarily equal to the
 actual derivative of the First Piola stress. One should NOT use this model in
 anything outside of this unit test.
*/
template <typename T>
class TestConstitutiveModel final : public HyperelasticConstitutiveModel<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestConstitutiveModel)

  TestConstitutiveModel(T E, T nu, T alpha, T beta)
      : HyperelasticConstitutiveModel<T>(E, nu, alpha, beta) {}

  ~TestConstitutiveModel() = default;

  Matrix3<T> get_strain() const { return strain_; }

 protected:
  virtual void DoUpdateDeformationBasedState(
      const Eigen::Ref<const Matrix3<T>>& F) {
    strain_ = 0.5 * (F.transpose() + F) - Matrix3<T>::Identity();
  }

  virtual T DoCalcEnergyDensity() const {
    return strain_.trace() * strain_.trace();
  }

  virtual Matrix3<T> DoCalcFirstPiola() const {
    return 2.0 * strain_.trace() * Matrix3<T>::Identity();
  }

  virtual Matrix3<T> DoCalcFirstPiolaDifferential(
      const Eigen::Ref<const Matrix3<T>>& dF) const {
    return 2.0 * strain_.trace() * dF.trace() * Matrix3<T>::Identity();
  }

  virtual Eigen::Matrix<T, 9, 9> DoCalcFirstPiolaDerivative() const {
    return Eigen::Matrix<T, 9, 9>::Identity();
  }

  Matrix3<T> strain_;
};

GTEST_TEST(HyperelasticConstitutiveModel, Parameters) {
  const TestConstitutiveModel<double> model(100.0, 0.25, 3.14, 2.7);
  double mu = 40.0;
  double lambda = 40.0;
  EXPECT_EQ(model.get_E(), 100.0);
  EXPECT_EQ(model.get_nu(), 0.25);
  EXPECT_EQ(model.get_alpha(), 3.14);
  EXPECT_EQ(model.get_beta(), 2.7);
  EXPECT_EQ(model.get_mu(), mu);
  EXPECT_EQ(model.get_lambda(), lambda);
}

GTEST_TEST(HyperelasticConstitutiveModel, InvalidYoungsModulus) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      TestConstitutiveModel<double> model(-1.0, 0.25, 3.14, 2.7),
      std::logic_error, "Young's modulus must be nonnegative.");
}

GTEST_TEST(HyperelasticConstitutiveModel, InvalidPoissonRatioAtUpperLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      TestConstitutiveModel<double> model(100.0, 0.5, 3.14, 2.7),
      std::logic_error, "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(HyperelasticConstitutiveModel, InvalidPoissonRatioOverUpperLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      TestConstitutiveModel<double> model(100.0, 0.6, 3.14, 2.7),
      std::logic_error, "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(HyperelasticConstitutiveModel, InvalidPoissonRatioAtLower) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      TestConstitutiveModel<double> model(100.0, -1.0, 3.14, 2.7),
      std::logic_error, "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(HyperelasticConstitutiveModel, InvalidPoissonRatioBelowLowerLimit) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      TestConstitutiveModel<double> model(100.0, -1.1, 3.14, 2.7),
      std::logic_error, "Poisson ratio must be in \\(-1, 0.5\\).");
}

GTEST_TEST(HyperelasticConstitutiveModel, InvalidMassDamping) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      TestConstitutiveModel<double> model(100.0, 0.25, -0.1, 2.7),
      std::logic_error, "Mass damping parameter must be nonnegative.");
}

GTEST_TEST(HyperelasticConstitutiveModel, InvalidStiffnessDamping) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      TestConstitutiveModel<double> model(100.0, 0.25, 3.14, -0.1),
      std::logic_error, "Stiffness damping parameter must be nonnegative.");
}

GTEST_TEST(HyperelasticConstitutiveModel, UpdateDeformationBasedState) {
  TestConstitutiveModel<double> model(100.0, 0.25, 3.14, 2.7);
  Matrix3<double> F;
  F << 1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 7.0, -1.0, 10.0;
  model.UpdateDeformationBasedState(F);
  Matrix3<double> strain;
  strain << 0.0, 3.0, 5.0, 3.0, 1.0, 2.0, 5.0, 2.0, 9.0;
  EXPECT_EQ(model.get_strain(), strain);
}

GTEST_TEST(HyperelasticConstitutiveModel, UpdatePositionBasedState) {
  TestConstitutiveModel<double> model(100.0, 0.25, 3.14, 2.7);
  Eigen::Matrix<double, 3, 4> q = Eigen::Matrix<double, 3, 4>::Zero();
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.UpdatePositionBasedState(q), std::runtime_error,
      "DoUpdatePositionBasedState\\(\\): Concrete type "
      "drake::fem::\\(anonymous\\)::TestConstitutiveModel<double> must provide "
      "an implementation.");
}

GTEST_TEST(HyperelasticConstitutiveModel, EnergyDensity) {
  TestConstitutiveModel<double> model(100.0, 0.25, 3.14, 2.7);
  Matrix3<double> F;
  F << 1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 7.0, -1.0, 10.0;
  model.UpdateDeformationBasedState(F);
  /* The diagonals of strain are 0, 1, 9. So trace of strain is 10.
   The energy density = strain.trace() * strain.trace()
                      = 10 * 10
                      = 100. */
  double analytic_energy_density = 100.0;
  EXPECT_EQ(model.CalcEnergyDensity(), analytic_energy_density);
}

GTEST_TEST(HyperelasticConstitutiveModel, FirstPiolaStress) {
  TestConstitutiveModel<double> model(100.0, 0.25, 3.14, 2.7);
  Matrix3<double> F;
  F << 1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 7.0, -1.0, 10.0;
  model.UpdateDeformationBasedState(F);
  /* The diagonals of strain are 0, 1, 9. So trace of strain is 10.
   The first Piola stress = 2 * strain.trace() * I₃
                          = 20 * I₃. */
  Matrix3<double> analytic_P = 20.0 * Matrix3<double>::Identity();
  EXPECT_TRUE(CompareMatrices(model.CalcFirstPiola(), analytic_P));
}

GTEST_TEST(HyperelasticConstitutiveModel, FirstPiolaStressDifferential) {
  TestConstitutiveModel<double> model(100.0, 0.25, 3.14, 2.7);
  Matrix3<double> F;
  F << 1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 7.0, -1.0, 10.0;
  Matrix3<double> dF = Matrix3<double>::Identity();
  model.UpdateDeformationBasedState(F);
  /* The diagonals of strain are 0, 1, 9. So trace of strain is 10.
   The first Piola stress differential = 2 * strain.trace() * dF.trace() * I₃
                                       = 2 * 10 * 3 * I₃
                                       = 60 * I₃. */
  Matrix3<double> analytic_dP = 60.0 * Matrix3<double>::Identity();
  EXPECT_TRUE(
      CompareMatrices(model.CalcFirstPiolaDifferential(dF), analytic_dP));
}

GTEST_TEST(HyperelasticConstitutiveModel, FirstPiolaStressDerivative) {
  TestConstitutiveModel<double> model(100.0, 0.25, 3.14, 2.7);
  Hessian analytic_dPdF = Hessian::Identity();
  EXPECT_TRUE(CompareMatrices(model.CalcFirstPiolaDerivative(), analytic_dPdF));
}

}  // namespace
}  // namespace fem
}  // namespace drake
