/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/phase_ring.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <exception>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/right_of_way_phase.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

RightOfWayPhase CreateFullPhase(const RightOfWayPhase::Id& id) {
  return RightOfWayPhase{
      id,
      {{RightOfWayRule::Id("rule_a"), RightOfWayRule::State::Id("GO")},
       {RightOfWayRule::Id("rule_b"), RightOfWayRule::State::Id("STOP")}},
      {{{Bulb::Id("rule_a_green"), BulbState::kOn},
        {Bulb::Id("rule_a_red"), BulbState::kOff},
        {Bulb::Id("rule_b_green"), BulbState::kOff},
        {Bulb::Id("rule_b_red"), BulbState::kOn}}}};
}

RightOfWayPhase CreatePhaseWithMissingRuleStates(
    const RightOfWayPhase::Id& id) {
  const RightOfWayPhase mock_phase = CreateFullPhase(id);
  RuleStates rule_states = mock_phase.rule_states();
  rule_states.erase(rule_states.begin());
  return RightOfWayPhase(id, rule_states, mock_phase.bulb_states());
}

RightOfWayPhase CreatePhaseWithMissingBulbStates(
    const RightOfWayPhase::Id& id) {
  const RightOfWayPhase mock_phase = CreateFullPhase(id);
  BulbStates bulb_states = *mock_phase.bulb_states();
  bulb_states.erase(bulb_states.begin());
  return RightOfWayPhase(id, mock_phase.rule_states(), bulb_states);
}

class PhaseRingTest : public ::testing::Test {
 protected:
  PhaseRingTest()
      : id_("my_ring"), phase_id_1_("phase_1"), phase_id_2_("phase_2") {}

  const PhaseRing::Id id_;
  const RightOfWayPhase::Id phase_id_1_;
  const RightOfWayPhase::Id phase_id_2_;
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
  const std::unordered_map<RightOfWayPhase::Id,
                           std::vector<PhaseRing::NextPhase>>
      partial_next_phases = {
          {phase_id_1_, std::vector<PhaseRing::NextPhase>()}};
  EXPECT_THROW(
      PhaseRing(id_,
                {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)},
                partial_next_phases),
      std::exception);

  // Next phases defines an unknown phase.
  const std::unordered_map<RightOfWayPhase::Id,
                           std::vector<PhaseRing::NextPhase>>
      unknown_next_phases = {{RightOfWayPhase::Id("unknown"),
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
  std::unordered_map<RightOfWayPhase::Id, std::vector<PhaseRing::NextPhase>>
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
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
