#include "drake/automotive/maliput/base/manual_rule_state_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"

namespace drake {
namespace maliput {
namespace {

using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::RuleStateProvider;

GTEST_TEST(ManualRuleStateProviderTest, BasicTest) {
  ManualRuleStateProvider dut;
  const RightOfWayRule::Id kRuleId("foo");
  const RightOfWayRule::State::Id kStateId("bar");

  // No rule with specified ID exists.
  EXPECT_FALSE(dut.GetState(kRuleId).has_value());
  EXPECT_THROW(dut.SetState(kRuleId, kStateId), std::out_of_range);

  dut.AddState(kRuleId, kStateId);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetState(kRuleId).value(),
      (RuleStateProvider::RightOfWayResult{kStateId, nullopt})));

  // Attempting to add duplicate state.
  EXPECT_THROW(dut.AddState(kRuleId, kStateId), std::logic_error);

  const RightOfWayRule::State::Id kOtherStateId("baz");
  dut.SetState(kRuleId, kOtherStateId);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetState(kRuleId).value(),
      (RuleStateProvider::RightOfWayResult{kOtherStateId, nullopt})));
}

}  // namespace
}  // namespace maliput
}  // namespace drake
