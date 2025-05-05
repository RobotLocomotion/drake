#include "drake/multibody/fem/linear_constitutive_model.h"

#include <gtest/gtest.h>

#include "drake/multibody/fem/test/constitutive_model_test_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

GTEST_TEST(LinearConstitutiveModelTest, Parameters) {
  TestParameters<LinearConstitutiveModel<double>>();
  TestParameters<LinearConstitutiveModel<AutoDiffXd>>();
  LinearConstitutiveModel<double> model(100, 0.4);
  EXPECT_TRUE(model.is_linear);
}

GTEST_TEST(LinearConstitutiveModelTest, UndeformedState) {
  TestUndeformedState<LinearConstitutiveModel<double>>();
  TestUndeformedState<LinearConstitutiveModel<AutoDiffXd>>();
}

GTEST_TEST(LinearConstitutiveModelTest, PIsDerivativeOfPsi) {
  TestPIsDerivativeOfPsi<LinearConstitutiveModel<AutoDiffXd>>();
}

GTEST_TEST(LinearConstitutiveModelTest, dPdFIsDerivativeOfP) {
  TestdPdFIsDerivativeOfP<LinearConstitutiveModel<AutoDiffXd>>();
}

GTEST_TEST(LinearConstitutiveModelTest, FilteredHessian) {
  TestSpdness<LinearConstitutiveModel<double>>();
  TestSpdness<LinearConstitutiveModel<AutoDiffXd>>();
}

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
