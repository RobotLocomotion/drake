#include "drake/multibody/fem/linear_constitutive_model.h"

#include <gtest/gtest.h>

#include "drake/multibody/fem/test/constitutive_model_test_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

constexpr int kNumLocations = 1;

GTEST_TEST(LinearConstitutiveModelTest, Parameters) {
  TestParameters<LinearConstitutiveModel<double, kNumLocations>>();
  TestParameters<LinearConstitutiveModel<AutoDiffXd, kNumLocations>>();
  LinearConstitutiveModel<double, kNumLocations> model(100, 0.4);
  EXPECT_TRUE(model.is_linear);
}

GTEST_TEST(LinearConstitutiveModelTest, UndeformedState) {
  TestUndeformedState<LinearConstitutiveModel<double, kNumLocations>>();
  TestUndeformedState<LinearConstitutiveModel<AutoDiffXd, kNumLocations>>();
}

GTEST_TEST(LinearConstitutiveModelTest, PIsDerivativeOfPsi) {
  TestPIsDerivativeOfPsi<LinearConstitutiveModel<AutoDiffXd, kNumLocations>>();
}

GTEST_TEST(LinearConstitutiveModelTest, dPdFIsDerivativeOfP) {
  TestdPdFIsDerivativeOfP<LinearConstitutiveModel<AutoDiffXd, kNumLocations>>();
}

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
