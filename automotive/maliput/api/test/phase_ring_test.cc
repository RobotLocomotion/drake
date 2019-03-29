/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/phase_ring.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <exception>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

Phase CreateFullPhase(const Phase::Id& id) {
  return Phase{
      id,
      {{RightOfWayRule::Id("rule_a"), RightOfWayRule::State::Id("GO")},
       {RightOfWayRule::Id("rule_b"), RightOfWayRule::State::Id("STOP")}},
      {{{Bulb::Id("rule_a_green"), BulbState::kOn},
        {Bulb::Id("rule_a_red"), BulbState::kOff},
        {Bulb::Id("rule_b_green"), BulbState::kOff},
        {Bulb::Id("rule_b_red"), BulbState::kOn}}}};
}

Phase CreatePhaseWithMissingRuleStates(const Phase::Id& id) {
  const Phase mock_phase = CreateFullPhase(id);
  RuleStates rule_states = mock_phase.rule_states();
  rule_states.erase(rule_states.begin());
  return Phase(id, rule_states, mock_phase.bulb_states());
}

Phase CreatePhaseWithMissingBulbStates(const Phase::Id& id) {
  const Phase mock_phase = CreateFullPhase(id);
  BulbStates bulb_states = *mock_phase.bulb_states();
  bulb_states.erase(bulb_states.begin());
  return Phase(id, mock_phase.rule_states(), bulb_states);
}

GTEST_TEST(PhaseRingTest, Constructor) {
  EXPECT_NO_THROW(
      PhaseRing(PhaseRing::Id("dut"),
                {CreateFullPhase(Phase::Id("my_phase_1")),
                 CreateFullPhase(Phase::Id("my_phase_2"))}));

  // No phases.
  EXPECT_THROW(PhaseRing(PhaseRing::Id("dut"), {}), std::exception);

  // Duplicate phases.
  EXPECT_THROW(PhaseRing(PhaseRing::Id("dut"),
                         {CreateFullPhase(Phase::Id("my_phase")),
                          CreateFullPhase(Phase::Id("my_phase"))}),
               std::exception);

  // Phases that differ in RightOfWayRule coverage.
  EXPECT_THROW(PhaseRing(PhaseRing::Id("dut"),
                         {CreateFullPhase(Phase::Id("full")),
                          CreatePhaseWithMissingRuleStates(
                              Phase::Id("missing_rules"))}),
               std::exception);

  // Phases that differ in bulb state coverage.
  EXPECT_THROW(PhaseRing(PhaseRing::Id("dut"),
                         {CreateFullPhase(Phase::Id("full")),
                          CreatePhaseWithMissingBulbStates(
                              Phase::Id("missing_rules"))}),
               std::exception);
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
