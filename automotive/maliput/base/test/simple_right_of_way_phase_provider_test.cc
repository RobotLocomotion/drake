#include "drake/automotive/maliput/base/simple_right_of_way_phase_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_provider.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace {

using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::RightOfWayRule;
using api::rules::RightOfWayPhaseProvider;
using api::rules::RuleStates;

// Fixture for testing the SimpleRightOfWayPhaseProvider.
struct SimpleRightOfWayPhaseProviderTest : public ::testing::Test {
  SimpleRightOfWayPhaseProviderTest()
      : phase_id_1("foo"), phase_id_2("bar"), phase_ring_id("bar") {
  }

  const Phase::Id phase_id_1;
  const Phase::Id phase_id_2;
  const PhaseRing::Id phase_ring_id;
  SimpleRightOfWayPhaseProvider dut;
};

TEST_F(SimpleRightOfWayPhaseProviderTest, EmptyProvider) {
  EXPECT_EQ(dut.GetPhase(phase_ring_id), nullopt);
  EXPECT_THROW(dut.SetPhase(phase_ring_id, phase_id_1), std::out_of_range);
}

// Tests situation where only the current phase is specified.
TEST_F(SimpleRightOfWayPhaseProviderTest, CurrentPhaseOnly) {
  EXPECT_NO_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1));
  EXPECT_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1), std::logic_error);
  drake::optional<RightOfWayPhaseProvider::Result> returned_phase =
      dut.GetPhase(phase_ring_id);
  EXPECT_EQ(returned_phase->id, phase_id_1);
  EXPECT_EQ(returned_phase->next, nullopt);
  EXPECT_NO_THROW(dut.SetPhase(phase_ring_id, phase_id_2));
  returned_phase = dut.GetPhase(phase_ring_id);
  EXPECT_EQ(returned_phase->id, phase_id_2);
  EXPECT_EQ(returned_phase->next, nullopt);
}

// Tests that an exception is thrown if the phases within a PhaseRing cover
// different sets of RightOfWayRules.
GTEST_TEST(PhaseRingTest, InvalidPhases) {
  const RuleStates rule_states_1 {{RightOfWayRule::Id("a"),
                                   RightOfWayRule::State::Id("1")},
                                  {RightOfWayRule::Id("b"),
                                   RightOfWayRule::State::Id("2")}};
  const RuleStates rule_states_2 {{RightOfWayRule::Id("a"),
                                   RightOfWayRule::State::Id("1")}};
  Phase phase_1(Phase::Id("bar"), rule_states_1);
  Phase phase_2(Phase::Id("baz"), rule_states_2);
  const std::vector<Phase> phases{phase_1, phase_2};
  EXPECT_THROW(PhaseRing(PhaseRing::Id("foo"), phases), std::exception);
}

}  // namespace
}  // namespace maliput
}  // namespace drake
