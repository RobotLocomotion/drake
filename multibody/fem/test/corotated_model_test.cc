#include "drake/multibody/fem/corotated_model.h"

#include <gtest/gtest.h>

#include "drake/multibody/fem/test/constitutive_model_test_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

constexpr int kNumLocations = 1;

GTEST_TEST(CorotatedModelTest, Parameters) {
  TestParameters<CorotatedModel<double, kNumLocations>>();
  TestParameters<CorotatedModel<AutoDiffXd, kNumLocations>>();
  CorotatedModel<double, kNumLocations> model(100, 0.4);
  EXPECT_FALSE(model.is_linear);
}

GTEST_TEST(CorotatedModelTest, UndeformedState) {
  TestUndeformedState<CorotatedModel<double, kNumLocations>>();
  TestUndeformedState<CorotatedModel<AutoDiffXd, kNumLocations>>();
}

GTEST_TEST(CorotatedModelTest, PIsDerivativeOfPsi) {
  TestPIsDerivativeOfPsi<CorotatedModel<AutoDiffXd, kNumLocations>>();
}

GTEST_TEST(CorotatedModelTest, dPdFIsDerivativeOfP) {
  TestdPdFIsDerivativeOfP<CorotatedModel<AutoDiffXd, kNumLocations>>();
}

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
