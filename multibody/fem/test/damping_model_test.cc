#include "drake/multibody/fem/damping_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

GTEST_TEST(DampingModelTest, Getters) {
  const double mass_coeff_alpha = 1.0;
  const double stiffness_coeff_beta = 2.0;
  const DampingModel<double> model(mass_coeff_alpha, stiffness_coeff_beta);
  EXPECT_EQ(model.mass_coeff_alpha(), mass_coeff_alpha);
  EXPECT_EQ(model.stiffness_coeff_beta(), stiffness_coeff_beta);
}

GTEST_TEST(DampingModelTest, InvalidModel) {
  /* Negative coefficients are not allowed. */
  EXPECT_THROW(DampingModel<double>(1.0, -1.0), std::exception);
  EXPECT_THROW(DampingModel<double>(1.0, -1.0), std::exception);
  /* Zero coefficients are OK. */
  EXPECT_NO_THROW(DampingModel<double>(0, 0));
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
