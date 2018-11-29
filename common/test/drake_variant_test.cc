#include "drake/common/drake_variant.h"

#include <stdexcept>

#include <gtest/gtest.h>

namespace drake {
namespace {

struct Squared {
  double operator()(double arg) { return arg * arg; }
};

GTEST_TEST(VariantTest, BasicTest) {
  // Default.
  variant<int, double> dut;
  EXPECT_EQ(dut.index(), 0);
  EXPECT_EQ(get<int>(dut), 0);
  EXPECT_EQ(get<0>(dut), 0);

  // Assign int.
  dut = 11;
  EXPECT_EQ(dut.index(), 0);
  EXPECT_EQ(get<int>(dut), 11);
  EXPECT_EQ(get<0>(dut), 11);

  // Assign double.
  dut = 1.23;
  EXPECT_EQ(dut.index(), 1);
  EXPECT_EQ(get<1>(dut), 1.23);
  EXPECT_EQ(get<double>(dut), 1.23);

  // Comparison.
  dut = 22;
  variant<int, double> other = 1.1;
  EXPECT_LT(dut, other);
  EXPECT_NE(dut, other);
  EXPECT_EQ(dut, dut);

  // Visiting.
  dut = 2;
  EXPECT_EQ(visit(Squared{}, dut), 4.0);
  dut = 0.5;
  EXPECT_EQ(visit(Squared{}, dut), 0.25);

  // Checking the alternatives.
  dut = 22;
  EXPECT_TRUE(holds_alternative<int>(dut));
  EXPECT_FALSE(holds_alternative<double>(dut));

  // Bad access.
  dut = 0;
  EXPECT_THROW(get<double>(dut), std::logic_error);
}

}  // namespace
}  // namespace drake
