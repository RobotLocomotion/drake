#include "drake/multibody/fem/damping_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

GTEST_TEST(DampingModelTest, Getters) {
  const double mass_coeff = 1.0;
  const double stiffness_coeff = 2.0;
  const DampingModel<double> model(mass_coeff, stiffness_coeff);
  EXPECT_EQ(model.mass_coeff(), mass_coeff);
  EXPECT_EQ(model.stiffness_coeff(), stiffness_coeff);
}

GTEST_TEST(DampingModelTest, InvalidModel) {
  /* Negative coefficients are not allowed. */
  DRAKE_EXPECT_THROWS_MESSAGE(
      DampingModel<double>(1.0, -1.0), std::exception,
      "Mass and stiffness damping coefficients must be non-negative.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      DampingModel<double>(1.0, -1.0), std::exception,
      "Mass and stiffness damping coefficients must be non-negative.");
  /* Zero coefficients are OK. */
  EXPECT_NO_THROW(DampingModel<double>(0, 0));
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
