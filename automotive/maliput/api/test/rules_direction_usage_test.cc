/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/direction_usage_rule.h"
/* clang-format on */
// TODO(andrew.best@tri.global) Satisfy clang-format via rules tests
//                              directory reorg.

#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/test_utilities/rules_direction_usage_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

using Severity = DirectionUsageRule::State::Severity;
using Type = DirectionUsageRule::State::Type;

class DirectionUsageTest : public ::testing::Test {
 protected:
  LaneSRange kZone{LaneId("the_lane"), SRange(13., 15.)};

  const DirectionUsageRule MakeDefaultRule() const {
    return DirectionUsageRule(
        DirectionUsageRule::Id("default_rule"), kZone,
        std::vector<DirectionUsageRule::State>{DirectionUsageRule::State(
            DirectionUsageRule::State::Id("some_state"), Type::kWithS,
            Severity::kStrict)});
  }

  const DirectionUsageRule MakeFromSeverityAndType(Severity severity,
                                                   Type type) const {
    const DirectionUsageRule::State the_state(
        DirectionUsageRule::State::Id("some_state"), type, severity);
    const std::vector<DirectionUsageRule::State> states{the_state};
    return DirectionUsageRule(DirectionUsageRule::Id("severity_type_rule"),
                              kZone, states);
  }

  // Provides an example of each Type and alternates Severity for iteration.
  // Does not imply coverage on the entire set of combinations, but provides
  // at least one example of each Type and Severity.
  const std::vector<std::pair<Severity, Type>> test_rules_{
      {Severity::kStrict, Type::kWithS},
      {Severity::kPreferred, Type::kAgainstS},
      {Severity::kStrict, Type::kBidirectional},
      {Severity::kPreferred, Type::kNoUse},
      {Severity::kStrict, Type::kParking},
      {Severity::kPreferred, Type::kBidirectionalTurnOnly},
  };
};

TEST_F(DirectionUsageTest, Construction) {
  for (const auto rule_config : test_rules_) {
    const DirectionUsageRule::State the_state(
        DirectionUsageRule::State::Id("some_state"), rule_config.second,
        rule_config.first);
    const std::vector<DirectionUsageRule::State> states{the_state};

    EXPECT_NO_THROW(
        DirectionUsageRule(DirectionUsageRule::Id("some_id"), kZone, states));
  }
}

TEST_F(DirectionUsageTest, AccessCopyAssign) {
  for (const auto rule_config : test_rules_) {
    const DirectionUsageRule source =
        MakeFromSeverityAndType(rule_config.first, rule_config.second);
    const DirectionUsageRule dut1(source);
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

TEST_F(DirectionUsageTest, StateTypeMapperTest) {
  const auto dut = DirectionUsageRule::StateTypeMapper();
  const std::vector<DirectionUsageRule::State::Type> expected_types{
      DirectionUsageRule::State::Type::kWithS,
      DirectionUsageRule::State::Type::kAgainstS,
      DirectionUsageRule::State::Type::kBidirectional,
      DirectionUsageRule::State::Type::kBidirectionalTurnOnly,
      DirectionUsageRule::State::Type::kNoUse,
      DirectionUsageRule::State::Type::kParking};
  EXPECT_EQ(dut.size(), expected_types.size());
  for (DirectionUsageRule::State::Type type : expected_types) {
    EXPECT_EQ(dut.count(type), 1);
  }
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
