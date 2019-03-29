/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/phase_ring.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <exception>
#include <stdexcept>

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

Phase CreatePhaseWithMissingRuleStates(
    const Phase::Id& id) {
  const Phase mock_phase = CreateFullPhase(id);
  RuleStates rule_states = mock_phase.rule_states();
  rule_states.erase(rule_states.begin());
  return Phase(id, rule_states, mock_phase.bulb_states());
}

Phase CreatePhaseWithMissingBulbStates(
    const Phase::Id& id) {
  const Phase mock_phase = CreateFullPhase(id);
  BulbStates bulb_states = *mock_phase.bulb_states();
  bulb_states.erase(bulb_states.begin());
  return Phase(id, mock_phase.rule_states(), bulb_states);
}

class PhaseRingTest : public ::testing::Test {
 protected:
  PhaseRingTest()
      : id_("my_ring"), phase_id_1_("phase_1"), phase_id_2_("phase_2") {}

  const PhaseRing::Id id_;
  const Phase::Id phase_id_1_;
  const Phase::Id phase_id_2_;
};

TEST_F(PhaseRingTest, Constructor) {
  EXPECT_NO_THROW(PhaseRing(
      id_, {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)}));

  // No phases.
  EXPECT_THROW(PhaseRing(id_, {}), std::exception);

  // Duplicate phases.
  EXPECT_THROW(PhaseRing(id_, {CreateFullPhase(phase_id_1_),
                               CreateFullPhase(phase_id_1_)}),
               std::exception);

  // Phases that differ in RightOfWayRule coverage.
  EXPECT_THROW(PhaseRing(id_, {CreateFullPhase(phase_id_1_),
                               CreatePhaseWithMissingRuleStates(phase_id_2_)}),
               std::exception);

  // Phases that differ in bulb state coverage.
  EXPECT_THROW(PhaseRing(id_, {CreateFullPhase(phase_id_1_),
                               CreatePhaseWithMissingBulbStates(phase_id_2_)}),
               std::exception);

  // Next phases does not fully cover phases.
  const std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>>
      partial_next_phases = {
          {phase_id_1_, std::vector<PhaseRing::NextPhase>()}};
  EXPECT_THROW(
      PhaseRing(id_,
                {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)},
                partial_next_phases),
      std::exception);

  // Next phases defines an unknown phase.
  const std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>>
      unknown_next_phases = {{Phase::Id("unknown"),
                              std::vector<PhaseRing::NextPhase>()}};
  EXPECT_THROW(
      PhaseRing(id_,
                {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)},
                unknown_next_phases),
      std::exception);
}

TEST_F(PhaseRingTest, Accessors) {
  const PhaseRing dut(
      id_, {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)});
  EXPECT_EQ(dut.id(), id_);
  EXPECT_EQ(dut.phases().size(), 2);
  EXPECT_EQ(dut.next_phases().size(), 2);
  EXPECT_EQ(dut.next_phases().at(phase_id_1_).size(), 0);
  EXPECT_EQ(dut.next_phases().at(phase_id_2_).size(), 0);
}

TEST_F(PhaseRingTest, NextPhases) {
  const double kDuration1{30};
  const double kDuration2{60};
  std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>>
      next_phases;
  next_phases.emplace(std::make_pair(
      phase_id_1_,
      std::vector<PhaseRing::NextPhase>{{phase_id_2_, kDuration1}}));
  next_phases.emplace(std::make_pair(
      phase_id_2_,
      std::vector<PhaseRing::NextPhase>{{phase_id_1_, kDuration2}}));
  const PhaseRing dut(
      id_, {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)},
      next_phases);
  const std::vector<PhaseRing::NextPhase> next_1 =
      dut.next_phases().at(phase_id_1_);
  const std::vector<PhaseRing::NextPhase> next_2 =
      dut.next_phases().at(phase_id_2_);
  EXPECT_EQ(next_1.size(), 1);
  EXPECT_EQ(next_2.size(), 1);
  EXPECT_EQ(next_1.at(0).id, phase_id_2_);
  EXPECT_EQ(next_2.at(0).id, phase_id_1_);
  EXPECT_EQ(*next_1.at(0).duration_until, kDuration1);
  EXPECT_EQ(*next_2.at(0).duration_until, kDuration2);
  EXPECT_EQ(dut.GetNextPhases(phase_id_1_).at(0).id, phase_id_2_);
  EXPECT_EQ(dut.GetNextPhases(phase_id_2_).at(0).id, phase_id_1_);
  EXPECT_THROW(dut.GetNextPhases(Phase::Id("Unknown")), std::out_of_range);
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
