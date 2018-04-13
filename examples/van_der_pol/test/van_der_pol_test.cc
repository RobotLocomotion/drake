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

}  // namespace
}  // namespace van_der_pol
}  // namespace examples
}  // namespace drake
