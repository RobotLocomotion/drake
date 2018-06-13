#include "drake/automotive/trivial_right_of_way_state_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::RightOfWayStateProvider;

GTEST_TEST(TrivialRightOfWayStateProviderTest, BasicTest) {
  TrivialRightOfWayStateProvider dut;
  const RightOfWayRule::Id kRuleId("foo");
  const RightOfWayRule::State::Id kStateId("bar");

  // No rule with specified ID exists.
  EXPECT_FALSE(dut.GetState(kRuleId).has_value());
  EXPECT_THROW(dut.SetState(kRuleId, kStateId), std::out_of_range);

  dut.AddState(kRuleId, kStateId);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetState(kRuleId).value(),
      (RightOfWayStateProvider::Result{kStateId, nullopt})));

  // Attempting to add duplicate state.
  EXPECT_THROW(dut.AddState(kRuleId, kStateId), std::logic_error);

  const RightOfWayRule::State::Id kOtherStateId("baz");
  dut.SetState(kRuleId, kOtherStateId);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetState(kRuleId).value(),
      (RightOfWayStateProvider::Result{kOtherStateId, nullopt})));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
