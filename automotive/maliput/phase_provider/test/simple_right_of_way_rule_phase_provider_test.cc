#include "drake/automotive/maliput/phase_provider/simple_right_of_way_rule_phase_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule_phase_provider.h"

namespace drake {
namespace maliput {
namespace phase_provider {
namespace {

using api::rules::RightOfWayRule;
using api::rules::RightOfWayRulePhase;

GTEST_TEST(SimpleRightOfWayRulePhaseProviderTest, DefaultConstructor) {
  SimpleRightOfWayRulePhaseProvider dut;
}

GTEST_TEST(SimpleRightOfWayRulePhaseProviderTest, AddGetRemoveRightOfWay) {
  const RightOfWayRulePhase::Id kPhaseId("foo");
  SimpleRightOfWayRulePhaseProvider dut;
  EXPECT_EQ(dut.GetPhase(kPhaseId), nullopt);
  EXPECT_EQ(dut.GetCurrentPhase(), nullopt);
  EXPECT_THROW(dut.SetCurrentPhase(kPhaseId), std::runtime_error);

  const RightOfWayRule::Id kRuleId("huron_southbound_straight");
  const RightOfWayRule::State::Id kStateId("go");
  RightOfWayRulePhase phase(kPhaseId);
  phase.rule_states.emplace(kRuleId, kStateId);
  const RightOfWayRulePhase phase_copy = phase;
  EXPECT_NO_THROW(dut.AddPhase(phase));
  EXPECT_THROW(dut.AddPhase(phase), std::runtime_error);
  EXPECT_THROW(dut.AddPhase(phase_copy), std::runtime_error);
  const drake::optional<const RightOfWayRulePhase&> returned_phase =
      dut.GetPhase(kPhaseId);
  EXPECT_NE(returned_phase, nullopt);
  EXPECT_EQ(returned_phase->id, kPhaseId);
  EXPECT_EQ(returned_phase->rule_states.size(), phase.rule_states.size());
  EXPECT_EQ(returned_phase->rule_states.at(kRuleId), kStateId);
  EXPECT_NO_THROW(dut.SetCurrentPhase(kPhaseId));
  const optional<api::rules::RightOfWayRulePhase::Id> returned_current_phase =
      dut.GetCurrentPhase();
  EXPECT_EQ(returned_current_phase.value(), kPhaseId);
}

}  // namespace
}  // namespace phase_provider
}  // namespace maliput
}  // namespace drake
