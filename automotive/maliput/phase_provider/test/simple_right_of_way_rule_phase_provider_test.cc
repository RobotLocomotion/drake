#include "drake/automotive/maliput/phase_provider/simple_right_of_way_rule_phase_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule_phase_provider.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace phase_provider {
namespace {

using api::rules::RightOfWayRule;
using api::rules::RightOfWayRulePhase;
using api::rules::RightOfWayRulePhaseProvider;

// Fixture for testing the SimpleRightOfWayRulePhaseProvider.
struct SimpleRightOfWayRulePhaseProviderTest : public ::testing::Test {
  SimpleRightOfWayRulePhaseProviderTest()
      : phase_id("foo"), rule_id("huron_southbound_straight"), state_id("go"),
        phase(phase_id) {
  }
  void SetUp() override {
    phase.rule_states.emplace(rule_id, state_id);
  }

  const RightOfWayRulePhase::Id phase_id;
  const RightOfWayRule::Id rule_id;
  const RightOfWayRule::State::Id state_id;
  RightOfWayRulePhase phase;
  SimpleRightOfWayRulePhaseProvider dut;
};

TEST_F(SimpleRightOfWayRulePhaseProviderTest, EmptyProvider) {
  EXPECT_EQ(dut.GetPhase(phase_id), nullopt);
  EXPECT_EQ(dut.GetCurrentPhase(), nullopt);
  EXPECT_THROW(dut.SetCurrentPhase(phase_id), std::runtime_error);
}

// Tests situation where only the current phase is specified.
TEST_F(SimpleRightOfWayRulePhaseProviderTest, CurrentPhaseOnly) {
  EXPECT_NO_THROW(dut.AddPhase(phase));
  EXPECT_THROW(dut.AddPhase(phase), std::runtime_error);
  const RightOfWayRulePhase phase_copy = phase;
  EXPECT_THROW(dut.AddPhase(phase_copy), std::runtime_error);
  const drake::optional<const RightOfWayRulePhase&> returned_phase =
      dut.GetPhase(phase_id);
  EXPECT_EQ(returned_phase->id, phase_id);
  EXPECT_EQ(returned_phase->rule_states.size(), phase.rule_states.size());
  EXPECT_EQ(returned_phase->rule_states.at(rule_id), state_id);
  EXPECT_NO_THROW(dut.SetCurrentPhase(phase_id));
  const optional<RightOfWayRulePhaseProvider::Result> current_phase =
      dut.GetCurrentPhase();
  EXPECT_EQ(current_phase->id, phase_id);
  EXPECT_EQ(current_phase->next, nullopt);
}

// Tests situation where both the current and next phases are specified.
TEST_F(SimpleRightOfWayRulePhaseProviderTest, CurrentAndNextPhase) {
  const RightOfWayRulePhase::Id kNextPhaseId("bar");
  constexpr double kDurationUntil{5.4};
  const RightOfWayRulePhase next_phase(kNextPhaseId);
  EXPECT_NO_THROW(dut.AddPhase(phase));
  EXPECT_NO_THROW(dut.AddPhase(next_phase));
  dut.SetNextPhase(kNextPhaseId, kDurationUntil);
  optional<RightOfWayRulePhaseProvider::Result> current_phase =
      dut.GetCurrentPhase();
  // Setting next phase without setting the current phase results in
  // dut.GetCurrentPhase() returning nullopt.
  EXPECT_EQ(current_phase, nullopt);
  EXPECT_NO_THROW(dut.SetCurrentPhase(phase_id));
  current_phase = dut.GetCurrentPhase();
  EXPECT_EQ(current_phase->id, phase_id);
  EXPECT_EQ(current_phase->next->id, kNextPhaseId);
  EXPECT_EQ(current_phase->next->duration_until, kDurationUntil);
  EXPECT_NO_THROW(dut.SetNextPhase(kNextPhaseId, nullopt));
  current_phase = dut.GetCurrentPhase();
  EXPECT_EQ(current_phase->next->duration_until, nullopt);
  dut.ResetNextPhase();
  current_phase = dut.GetCurrentPhase();
  EXPECT_EQ(current_phase->next, nullopt);
}

}  // namespace
}  // namespace phase_provider
}  // namespace maliput
}  // namespace drake
