/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
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

/// Predicate-formatter which tests equality of RightOfWayRule::Type.
// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::RightOfWayRule::Type a,
                                   rules::RightOfWayRule::Type b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of RightOfWayRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::RightOfWayRule& a,
                                   const rules::RightOfWayRule& b) {
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.controlled_zone(),
                                         b.controlled_zone()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.type(), b.type()));
  return c.result();
}

}  // namespace test

namespace {

const LaneSRoute kZone({LaneSRange(LaneId("a"), {0., 9.}),
                        LaneSRange(LaneId("b"), {17., 12.})});


GTEST_TEST(RightOfWayRuleTest, Construction) {
  EXPECT_NO_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"), kZone,
                                 RightOfWayRule::Type::kStopThenGo));
  EXPECT_NO_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"), kZone,
                                 RightOfWayRule::Type::kDynamic));
}


GTEST_TEST(RightOfWayRuleTest, Accessors) {
  const RightOfWayRule dut(RightOfWayRule::Id("dut_id"), kZone,
                           RightOfWayRule::Type::kStopThenGo);
  EXPECT_EQ(dut.id(), RightOfWayRule::Id("dut_id"));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.controlled_zone(), kZone));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.type(), RightOfWayRule::Type::kStopThenGo));
}


GTEST_TEST(RightOfWayRuleTest, Copying) {
  const RightOfWayRule source(RightOfWayRule::Id("dut_id"), kZone,
                              RightOfWayRule::Type::kStopThenGo);

  const RightOfWayRule dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}


GTEST_TEST(RightOfWayRuleTest, Assignment) {
  const RightOfWayRule source(RightOfWayRule::Id("dut_id"), kZone,
                              RightOfWayRule::Type::kStopThenGo);
  RightOfWayRule dut(RightOfWayRule::Id("other_id"),
                     LaneSRoute({{LaneId("z"), {3., 4.}}}),
                     RightOfWayRule::Type::kYield);
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}


}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
