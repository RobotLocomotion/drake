#include "drake/automotive/trivial_right_of_way_state_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

namespace drake {
namespace automotive {
namespace {

using maliput::api::rules::RightOfWayRule;

GTEST_TEST(TrivialRightOfWayStateProviderTest, BasicTest) {
  TrivialRightOfWayStateProvider dut;
  RightOfWayRule::Id id("foo");

  // No rule with specified ID exists.
  EXPECT_THROW(dut.GetState(id), std::out_of_range);
  EXPECT_THROW(dut.SetState(id, RightOfWayRule::DynamicState::kUncontrolled),
               std::out_of_range);

  dut.AddState(id, RightOfWayRule::DynamicState::kGo);
  EXPECT_EQ(dut.GetState(id), RightOfWayRule::DynamicState::kGo);

  // Attempting to add duplicate state.
  EXPECT_THROW(dut.AddState(id, RightOfWayRule::DynamicState::kStop),
               std::logic_error);

  dut.SetState(id, RightOfWayRule::DynamicState::kPrepareToStop);
  EXPECT_EQ(dut.GetState(id), RightOfWayRule::DynamicState::kPrepareToStop);
}

}  // namespace
}  // namespace automotive
}  // namespace drake

