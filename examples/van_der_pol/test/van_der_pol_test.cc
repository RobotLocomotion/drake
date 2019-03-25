#include "drake/examples/van_der_pol/van_der_pol.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace examples {
namespace van_der_pol {
namespace {

GTEST_TEST(VanDerPolTest, ScalarConversionTest) {
  VanDerPolOscillator<double> vdp;
  EXPECT_TRUE(is_autodiffxd_convertible(vdp));
  EXPECT_TRUE(is_symbolic_convertible(vdp));
}

GTEST_TEST(VanDerPolTest, LimitCycle) {
  const auto cycle = VanDerPolOscillator<double>::CalcLimitCycle();

  const int N = cycle.cols();
  EXPECT_NEAR(cycle(0, 0), cycle(0, N - 1), 1e-2);
  EXPECT_NEAR(cycle(1, 0), cycle(1, N - 1), 5e-3);
}

}  // namespace
}  // namespace van_der_pol
}  // namespace examples
}  // namespace drake
