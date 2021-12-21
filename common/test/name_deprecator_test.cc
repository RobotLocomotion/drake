#include "drake/common/name_deprecator.h"

#include <gtest/gtest.h>

namespace drake {
namespace internal {
namespace {

GTEST_TEST(NameDeprecatorTest, SmokeTest) {
  NameDeprecator dut;
  std::set<std::string> correct_names({"ca", "cb"});
  EXPECT_FALSE(dut.HasScope());
  dut.DeclareScope(
      "test",
      [&correct_names](const std::string& x) {
        return correct_names.count(x);
      });
  EXPECT_TRUE(dut.HasScope());
  dut.DeclareDeprecatedName("2453-11-11", "da", "ca");
  // Multiple aliases to one target is ok.
  dut.DeclareDeprecatedName("2453-11-11", "dda", "ca");
  dut.DeclareDeprecatedName("2453-11-11", "db", "cb");

  // First requests for deprecated names warn.
  EXPECT_EQ(dut.MaybeTranslate("da"), "ca");
  EXPECT_EQ(dut.MaybeTranslate("dda"), "ca");
  EXPECT_EQ(dut.MaybeTranslate("db"), "cb");

  // Subsequent requests for deprecated names are silent; check the console
  // output for this test to see.
  EXPECT_EQ(dut.MaybeTranslate("da"), "ca");
  EXPECT_EQ(dut.MaybeTranslate("dda"), "ca");
  EXPECT_EQ(dut.MaybeTranslate("db"), "cb");

  // Correct names pass through.
  EXPECT_EQ(dut.MaybeTranslate("cb"), "cb");

  // Random strings pass through.
  EXPECT_EQ(dut.MaybeTranslate("xq"), "xq");
}

}  // namespace
}  // namespace internal
}  // namespace drake

