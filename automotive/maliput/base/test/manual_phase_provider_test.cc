#include "drake/automotive/maliput/base/manual_phase_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_provider.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace {

using api::rules::Phase;
using api::rules::PhaseProvider;
using api::rules::PhaseRing;
using api::rules::RightOfWayRule;
using api::rules::RuleStates;

constexpr double kDurationUntil{10.};  // Arbitrarily chosen.

// Fixture for testing the ManualPhaseProvider.
struct ManualPhaseProviderTest : public ::testing::Test {
  ManualPhaseProviderTest()
      : phase_id_1("foo"), phase_id_2("bar"), phase_ring_id("bar") {}

  const Phase::Id phase_id_1;
  const Phase::Id phase_id_2;
  const PhaseRing::Id phase_ring_id;
  ManualPhaseProvider dut;
};

TEST_F(ManualPhaseProviderTest, EmptyProvider) {
  EXPECT_EQ(dut.GetPhase(phase_ring_id), nullopt);
  EXPECT_THROW(dut.SetPhase(phase_ring_id, phase_id_1), std::exception);
}

// Tests that an exception is thrown if duration-until is specified but the next
// phase is not when adding a PhaseRing.
TEST_F(ManualPhaseProviderTest, AddPhaseRingDurationSpecifiedOnly) {
  EXPECT_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1,
                                nullopt /* next phase */, kDurationUntil),
               std::logic_error);
}

// Tests that an exception is thrown if duration-until is specified but the next
// phase is not when setting the phase of a PhaseRing.
TEST_F(ManualPhaseProviderTest, SetPhaseRingDurationSpecifiedOnly) {
  EXPECT_NO_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1));
  EXPECT_THROW(dut.SetPhase(phase_ring_id, phase_id_1, nullopt /* next phase */,
                            kDurationUntil),
               std::logic_error);
}

// Tests situation where only the current phase is specified.
TEST_F(ManualPhaseProviderTest, CurrentPhaseOnly) {
  EXPECT_NO_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1));
  EXPECT_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1), std::logic_error);
  drake::optional<PhaseProvider::Result> returned_phase =
      dut.GetPhase(phase_ring_id);
  EXPECT_EQ(returned_phase->id, phase_id_1);
  EXPECT_EQ(returned_phase->next, nullopt);
  EXPECT_NO_THROW(dut.SetPhase(phase_ring_id, phase_id_2));
  returned_phase = dut.GetPhase(phase_ring_id);
  EXPECT_EQ(returned_phase->id, phase_id_2);
  EXPECT_EQ(returned_phase->next, nullopt);
}

// Tests situation where both the current and next phases are specified.
TEST_F(ManualPhaseProviderTest, CurrentAndNextPhases) {
  EXPECT_NO_THROW(
      dut.AddPhaseRing(phase_ring_id, phase_id_1, phase_id_2, kDurationUntil));
  drake::optional<PhaseProvider::Result> returned_phase =
      dut.GetPhase(phase_ring_id);
  EXPECT_EQ(returned_phase->id, phase_id_1);
  EXPECT_NE(returned_phase->next, nullopt);
  EXPECT_EQ(returned_phase->next->id, phase_id_2);
  EXPECT_EQ(*returned_phase->next->duration_until, kDurationUntil);
  EXPECT_NO_THROW(
      dut.SetPhase(phase_ring_id, phase_id_2, phase_id_1, 2 * kDurationUntil));
  returned_phase = dut.GetPhase(phase_ring_id);
  EXPECT_EQ(returned_phase->id, phase_id_2);
  EXPECT_NE(returned_phase->next, nullopt);
  EXPECT_EQ(returned_phase->next->id, phase_id_1);
  EXPECT_EQ(*returned_phase->next->duration_until, 2 * kDurationUntil);
}

// Tests that an exception is thrown if the phases within a PhaseRing cover
// different sets of RightOfWayRules.
GTEST_TEST(PhaseRingTest, InvalidPhases) {
  const RuleStates rule_states_1{
      {RightOfWayRule::Id("a"), RightOfWayRule::State::Id("1")},
      {RightOfWayRule::Id("b"), RightOfWayRule::State::Id("2")}};
  const RuleStates rule_states_2{
      {RightOfWayRule::Id("a"), RightOfWayRule::State::Id("1")}};
  Phase phase_1(Phase::Id("bar"), rule_states_1);
  Phase phase_2(Phase::Id("baz"), rule_states_2);
  const std::vector<Phase> phases{phase_1, phase_2};
  EXPECT_THROW(PhaseRing(PhaseRing::Id("foo"), phases), std::exception);
}

}  // namespace
}  // namespace maliput
}  // namespace drake
