/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
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
