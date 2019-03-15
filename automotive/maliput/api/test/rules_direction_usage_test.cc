/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/direction_usage_rule.h"
/* clang-format on */
// TODO(andrew.best@tri.global) Satisfy clang-format via rules tests
//                              directory reorg.

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/test_utilities/rules_direction_usage_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

const LaneSRange kZone(LaneId("the_lane"), SRange(13., 15.));

GTEST_TEST(DirectionUsageRuleTest, Construction) {
  EXPECT_NO_THROW(DirectionUsageRule(DirectionUsageRule::Id("some_id"), kZone,
                                     DirectionUsageRule::Severity::kStrict,
                                     DirectionUsageRule::Direction::kWithS));
  EXPECT_NO_THROW(DirectionUsageRule(DirectionUsageRule::Id("some_id"), kZone,
                                     DirectionUsageRule::Severity::kPreferred,
                                     DirectionUsageRule::Direction::kWithS));
  EXPECT_NO_THROW(DirectionUsageRule(DirectionUsageRule::Id("some_id"), kZone,
                                     DirectionUsageRule::Severity::kPreferred,
                                     DirectionUsageRule::Direction::kAgainstS));
  EXPECT_NO_THROW(DirectionUsageRule(DirectionUsageRule::Id("some_id"), kZone,
                                     DirectionUsageRule::Severity::kPreferred,
                                     DirectionUsageRule::Direction::kBoth));
}

GTEST_TEST(DirectionUsageRuleTest, Accessors) {
  const DirectionUsageRule dut(DirectionUsageRule::Id("dut_id"), kZone,
                               DirectionUsageRule::Severity::kStrict,
                               DirectionUsageRule::Direction::kWithS);
  EXPECT_EQ(dut.id(), DirectionUsageRule::Id("dut_id"));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.zone(), kZone));
  EXPECT_EQ(dut.severity(), DirectionUsageRule::Severity::kStrict);
  EXPECT_EQ(dut.direction(), DirectionUsageRule::Direction::kWithS);

  // Test another severity and direction.
  const DirectionUsageRule dut2(DirectionUsageRule::Id("dut_id"), kZone,
                                DirectionUsageRule::Severity::kPreferred,
                                DirectionUsageRule::Direction::kAgainstS);
  EXPECT_EQ(dut2.severity(), DirectionUsageRule::Severity::kPreferred);
  EXPECT_EQ(dut2.direction(), DirectionUsageRule::Direction::kAgainstS);
}

GTEST_TEST(DirectionUsageRuleTest, Copying) {
  const DirectionUsageRule source(DirectionUsageRule::Id("dut_id"), kZone,
                                  DirectionUsageRule::Severity::kStrict,
                                  DirectionUsageRule::Direction::kWithS);
  const DirectionUsageRule dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(DirectionUsageRuleTest, Assignment) {
  const DirectionUsageRule source(DirectionUsageRule::Id("dut_id"), kZone,
                                  DirectionUsageRule::Severity::kStrict,
                                  DirectionUsageRule::Direction::kWithS);
  DirectionUsageRule dut(DirectionUsageRule::Id("dut_id"), kZone,
                         DirectionUsageRule::Severity::kPreferred,
                         DirectionUsageRule::Direction::kAgainstS);
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
