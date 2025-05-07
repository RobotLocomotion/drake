#include "drake/multibody/fem/linear_corotated_model.h"

#include <gtest/gtest.h>

#include "drake/multibody/fem/test/constitutive_model_test_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

GTEST_TEST(LinearLinearCorotatedModelTest, Parameters) {
  TestParameters<LinearCorotatedModel<double>>();
  TestParameters<LinearCorotatedModel<AutoDiffXd>>();
  LinearCorotatedModel<double> model(100, 0.4);
  EXPECT_TRUE(model.is_linear);
}

GTEST_TEST(LinearLinearCorotatedModelTest, UndeformedState) {
  TestUndeformedState<LinearCorotatedModel<double>>();
  TestUndeformedState<LinearCorotatedModel<AutoDiffXd>>();
}

GTEST_TEST(LinearLinearCorotatedModelTest, PIsDerivativeOfPsi) {
  TestPIsDerivativeOfPsi<LinearCorotatedModel<AutoDiffXd>>();
}

GTEST_TEST(LinearLinearCorotatedModelTest, dPdFIsDerivativeOfP) {
  TestdPdFIsDerivativeOfP<LinearCorotatedModel<AutoDiffXd>>();
}

GTEST_TEST(LinearCorotatedModelTest, FilteredHessian) {
  TestSpdness<LinearCorotatedModel<double>>();
  TestSpdness<LinearCorotatedModel<AutoDiffXd>>();
}

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
