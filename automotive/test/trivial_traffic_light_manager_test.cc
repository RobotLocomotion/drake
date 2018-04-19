#include <gtest/gtest.h>

#include "drake/automotive/trivial_traffic_light_manager.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::rules::RightOfWayRule;

GTEST_TEST(TrivialTrafficLightManagerTest, BasicTest) {
  TrivialTrafficLightManager dut;
  RightOfWayRule::Id id("foo");

  // No rule with specified ID exists.
  EXPECT_THROW(dut.GetState(id), std::runtime_error);
  EXPECT_THROW(dut.SetState(id, RightOfWayRule::DynamicState::kUncontrolled),
               std::runtime_error);

  dut.AddState(id, RightOfWayRule::DynamicState::kGo);
  EXPECT_EQ(dut.GetState(id), RightOfWayRule::DynamicState::kGo);

  // Attempting to add duplicate state.
  EXPECT_THROW(dut.AddState(id, RightOfWayRule::DynamicState::kStop),
               std::runtime_error);

  dut.SetState(id, RightOfWayRule::DynamicState::kPrepareToStop);
  EXPECT_EQ(dut.GetState(id), RightOfWayRule::DynamicState::kPrepareToStop);
}

}  // namespace
}  // namespace automotive
}  // namespace drake

