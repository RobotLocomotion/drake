#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"

#include <gtest/gtest.h>

#include "drake/multibody/fixed_fem/dev/test/constitutive_model_test_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

constexpr int kNumLocations = 1;

GTEST_TEST(LinearConstitutiveModelTest, Parameters) {
  TestParameters<LinearConstitutiveModel<double, kNumLocations>>();
  TestParameters<LinearConstitutiveModel<AutoDiffXd, kNumLocations>>();
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
