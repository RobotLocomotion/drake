#include "drake/multibody/fem/corotated_model.h"

#include <gtest/gtest.h>

#include "drake/multibody/fem/test/constitutive_model_test_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

GTEST_TEST(CorotatedModelTest, Parameters) {
  TestParameters<CorotatedModel<double>>();
  TestParameters<CorotatedModel<AutoDiffXd>>();
  CorotatedModel<double> model(100, 0.4);
  EXPECT_FALSE(model.is_linear);
}

GTEST_TEST(CorotatedModelTest, UndeformedState) {
  TestUndeformedState<CorotatedModel<double>>();
  TestUndeformedState<CorotatedModel<AutoDiffXd>>();
}

GTEST_TEST(CorotatedModelTest, PIsDerivativeOfPsi) {
  TestPIsDerivativeOfPsi<CorotatedModel<AutoDiffXd>>();
}

GTEST_TEST(CorotatedModelTest, dPdFIsDerivativeOfP) {
  TestdPdFIsDerivativeOfP<CorotatedModel<AutoDiffXd>>();
}

GTEST_TEST(CorotatedModelTest, FilteredHessian) {
  TestSpdness<CorotatedModel<double>>();
  TestSpdness<CorotatedModel<AutoDiffXd>>();
}

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
