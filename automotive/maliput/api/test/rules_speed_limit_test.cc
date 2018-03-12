/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

namespace test {

/// Predicate-formatter which tests equality of SpeedLimitRule::Severity.
// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::SpeedLimitRule::Severity a,
                                   rules::SpeedLimitRule::Severity b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of SpeedLimitRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::SpeedLimitRule& a,
                                   const rules::SpeedLimitRule& b) {
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.severity(), b.severity()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.max(), b.max()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.min(), b.min()));
  return c.result();
}

}  // namespace test

namespace {

const LaneSRange kZone(LaneId("the_lane"), SRange(13., 15.));


GTEST_TEST(SpeedLimitRuleTest, Construction) {
  EXPECT_NO_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                                 SpeedLimitRule::Severity::kStrict,
                                 33., 77.));
  EXPECT_NO_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                                 SpeedLimitRule::Severity::kStrict,
                                 0., 77.));
  EXPECT_NO_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                                 SpeedLimitRule::Severity::kStrict,
                                 0., 0.));

  // Min must not be greater than max.
  EXPECT_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                              SpeedLimitRule::Severity::kStrict,
                              90., 77.),
               std::runtime_error);
  // Negative limits are not allowed.
  EXPECT_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                              SpeedLimitRule::Severity::kStrict,
                              -8., 77.),
               std::runtime_error);
  EXPECT_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                              SpeedLimitRule::Severity::kStrict,
                              -8., -6.),
               std::runtime_error);
}


GTEST_TEST(SpeedLimitRuleTest, Accessors) {
  const SpeedLimitRule dut(SpeedLimitRule::Id("dut_id"), kZone,
                           SpeedLimitRule::Severity::kStrict,
                           5., 8.);
  EXPECT_EQ(dut.id(), SpeedLimitRule::Id("dut_id"));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.zone(), kZone));
  EXPECT_EQ(dut.severity(), SpeedLimitRule::Severity::kStrict);
  EXPECT_EQ(dut.min(), 5.);
  EXPECT_EQ(dut.max(), 8.);

  // Test another severity and the effectively-no-minimum case as well.
  const SpeedLimitRule dut2(SpeedLimitRule::Id("dut_id"), kZone,
                            SpeedLimitRule::Severity::kAdvisory,
                            0., 8.);
  EXPECT_EQ(dut2.severity(), SpeedLimitRule::Severity::kAdvisory);
  EXPECT_EQ(dut2.min(), 0.);
}


GTEST_TEST(SpeedLimitRuleTest, Copying) {
  const SpeedLimitRule source(SpeedLimitRule::Id("dut_id"), kZone,
                              SpeedLimitRule::Severity::kStrict,
                              5., 8.);

  const SpeedLimitRule dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}


GTEST_TEST(SpeedLimitRuleTest, Assignment) {
  const SpeedLimitRule source(SpeedLimitRule::Id("dut_id"), kZone,
                              SpeedLimitRule::Severity::kStrict,
                              0., 8.);
  SpeedLimitRule dut(SpeedLimitRule::Id("other_id"), kZone,
                     SpeedLimitRule::Severity::kAdvisory,
                     70., 90.);

  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}


}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
