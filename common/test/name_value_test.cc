#include "drake/common/name_value.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(NameValueTest, SmokeTest) {
  int foo{1};
  auto dut = DRAKE_NVP(foo);
  EXPECT_EQ(dut.name(), std::string("foo"));
  EXPECT_EQ(dut.value(), &foo);
  EXPECT_EQ(dut.get_value(), 1);
  dut.set_value(2);
  EXPECT_EQ(dut.get_value(), 2);
  EXPECT_EQ(foo, 2);
}

}  // namespace
}  // namespace drake
