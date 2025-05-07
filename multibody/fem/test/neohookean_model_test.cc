#include "drake/multibody/fem/neohookean_model.h"

#include <gtest/gtest.h>

#include "drake/multibody/fem/test/constitutive_model_test_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

GTEST_TEST(NeoHookeanModelTest, Parameters) {
  TestParameters<NeoHookeanModel<double>>();
  TestParameters<NeoHookeanModel<AutoDiffXd>>();
  NeoHookeanModel<double> model(100, 0.4);
  EXPECT_FALSE(model.is_linear);
}

GTEST_TEST(NeoHookeanModelTest, UndeformedState) {
  TestUndeformedState<NeoHookeanModel<double>>(/* nonzero rest state */ true);
  TestUndeformedState<NeoHookeanModel<AutoDiffXd>>(
      /* nonzero_rest_state */ true);
}

GTEST_TEST(NeoHookeanModelTest, PIsDerivativeOfPsi) {
  TestPIsDerivativeOfPsi<NeoHookeanModel<AutoDiffXd>>();
}

GTEST_TEST(NeoHookeanModelTest, dPdFIsDerivativeOfP) {
  TestdPdFIsDerivativeOfP<NeoHookeanModel<AutoDiffXd>>();
}

GTEST_TEST(NeoHookeanModelTest, FilteredHessian) {
  TestSpdness<NeoHookeanModel<double>>();
  TestSpdness<NeoHookeanModel<AutoDiffXd>>();
}

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
