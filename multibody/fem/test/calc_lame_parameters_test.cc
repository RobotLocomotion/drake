#include "drake/multibody/fem/calc_lame_parameters.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

GTEST_TEST(CalcLameParametersTest, InvalidParameters) {
  DRAKE_EXPECT_THROWS_MESSAGE(CalcLameParameters<double>(-1.0, 0.25),
                              "Young's modulus must be nonnegative.");

  DRAKE_EXPECT_THROWS_MESSAGE(CalcLameParameters<double>(100.0, 0.5),
                              "Poisson's ratio must be in .*");

  DRAKE_EXPECT_THROWS_MESSAGE(CalcLameParameters<double>(100.0, 0.6),
                              "Poisson's ratio must be in .*");

  DRAKE_EXPECT_THROWS_MESSAGE(CalcLameParameters<double>(100.0, -1.0),
                              "Poisson's ratio must be in .*");

  DRAKE_EXPECT_THROWS_MESSAGE(CalcLameParameters<double>(100.0, -1.1),
                              "Poisson's ratio must be in .*");
}

/* Verify that the calculated Lamé parameters match pen and paper calculation.
 */
GTEST_TEST(CalcLameParametersTest, AnalyticResults) {
  const LameParameters<double> lame_params =
      CalcLameParameters<double>(280, 0.4);
  EXPECT_DOUBLE_EQ(lame_params.mu, 100.0);
  EXPECT_DOUBLE_EQ(lame_params.lambda, 400.0);
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
