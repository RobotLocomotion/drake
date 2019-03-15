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

class DirectionUsageTest : public ::testing::Test {
 protected:
  DirectionUsageRule MakeDefaultRule() {
    return DirectionUsageRule(
        DirectionUsageRule::Id("default_rule"), kZone_,
        std::vector<DirectionUsageRule::State>{DirectionUsageRule::State(
            DirectionUsageRule::State::Id("some_state"),
            DirectionUsageRule::State::Type::kWithS,
            DirectionUsageRule::State::Severity::kStrict)});
  }

  DirectionUsageRule MakeFromSeverityType(
      DirectionUsageRule::State::Severity severity,
      DirectionUsageRule::State::Type type) {
    DirectionUsageRule::State the_state(
        DirectionUsageRule::State::Id("some_state"), type, severity);
    std::vector<DirectionUsageRule::State> states{the_state};
    return DirectionUsageRule(DirectionUsageRule::Id("severity_type_rule"),
                              kZone_, states);
  }
  LaneSRange kZone_{LaneId("the_lane"), SRange(13., 15.)};

  std::vector<std::pair<DirectionUsageRule::State::Severity,
                        DirectionUsageRule::State::Type>>
      test_rules_{
          {DirectionUsageRule::State::Severity::kStrict,
           DirectionUsageRule::State::Type::kWithS},
          {DirectionUsageRule::State::Severity::kPreferred,
           DirectionUsageRule::State::Type::kAgainstS},
          {DirectionUsageRule::State::Severity::kStrict,
           DirectionUsageRule::State::Type::kBidirectional},
          {DirectionUsageRule::State::Severity::kPreferred,
           DirectionUsageRule::State::Type::kNoUse},
          {DirectionUsageRule::State::Severity::kStrict,
           DirectionUsageRule::State::Type::kParking},
      };
};

TEST_F(DirectionUsageTest, Construction) {
  for (auto rule_config : test_rules_) {
    DirectionUsageRule::State the_state(
        DirectionUsageRule::State::Id("some_state"), rule_config.second,
        rule_config.first);
    std::vector<DirectionUsageRule::State> states{the_state};

    EXPECT_NO_THROW(
        DirectionUsageRule(DirectionUsageRule::Id("some_id"), kZone_, states));
  }
}

TEST_F(DirectionUsageTest, AccessCopyAssign) {
  for (auto rule_config : test_rules_) {
    DirectionUsageRule source =
        MakeFromSeverityType(rule_config.first, rule_config.second);
    DirectionUsageRule dut1(source);
    DirectionUsageRule dut2 = MakeDefaultRule();
    // Verify copy and assign.
    EXPECT_EQ(source.id(), dut1.id());
    EXPECT_NE(source.id(), dut2.id());
    dut2 = source;
    EXPECT_TRUE(MALIPUT_IS_EQUAL(source, dut2));
    EXPECT_TRUE(MALIPUT_IS_EQUAL(source, dut1));

    EXPECT_TRUE(MALIPUT_IS_EQUAL(source.zone(), dut1.zone()));
    EXPECT_TRUE(MALIPUT_IS_EQUAL(source.is_static(), dut2.is_static()));
    EXPECT_TRUE(MALIPUT_IS_EQUAL(source.static_state(), dut2.static_state()));
    EXPECT_TRUE(MALIPUT_IS_EQUAL(source.static_state().type(),
                                 dut1.static_state().type()));
    EXPECT_TRUE(MALIPUT_IS_EQUAL(source.static_state().severity(),
                                 dut2.static_state().severity()));
  }
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
