#include "drake/multibody/fem/linear_corotated_model.h"

#include <gtest/gtest.h>

#include "drake/multibody/fem/test/constitutive_model_test_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

constexpr int kNumLocations = 1;

GTEST_TEST(LinearLinearCorotatedModelTest, Parameters) {
  TestParameters<LinearCorotatedModel<double, kNumLocations>>();
  TestParameters<LinearCorotatedModel<AutoDiffXd, kNumLocations>>();
}

GTEST_TEST(LinearLinearCorotatedModelTest, UndeformedState) {
  TestUndeformedState<LinearCorotatedModel<double, kNumLocations>>();
  TestUndeformedState<LinearCorotatedModel<AutoDiffXd, kNumLocations>>();
}

GTEST_TEST(LinearLinearCorotatedModelTest, PIsDerivativeOfPsi) {
  TestPIsDerivativeOfPsi<LinearCorotatedModel<AutoDiffXd, kNumLocations>>();
}

GTEST_TEST(LinearLinearCorotatedModelTest, dPdFIsDerivativeOfP) {
  TestdPdFIsDerivativeOfP<LinearCorotatedModel<AutoDiffXd, kNumLocations>>();
}

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
