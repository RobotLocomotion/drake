#include "drake/automotive/maliput/base/simple_right_of_way_phase_book.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace {

using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::RightOfWayRule;

struct SimpleRightOfWayPhaseBookTest : public ::testing::Test {
  SimpleRightOfWayPhaseBookTest()
      : rule_id_a("rule a"),
        rule_id_b("rule b"),
        phase(Phase::Id("phase"),
              {{rule_id_a, RightOfWayRule::State::Id("a")},
               {rule_id_b, RightOfWayRule::State::Id("b")}}),
        ring_id("ring"),
        ring(ring_id, {phase}) {}

  const RightOfWayRule::Id rule_id_a;
  const RightOfWayRule::Id rule_id_b;
  const Phase phase;
  const PhaseRing::Id ring_id;
  const PhaseRing ring;
};

TEST_F(SimpleRightOfWayPhaseBookTest, BasicTest) {
  SimpleRightOfWayPhaseBook dut;
  EXPECT_NO_THROW(dut.AddPhaseRing(ring));
  optional<PhaseRing> result = dut.GetPhaseRing(ring_id);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result->id(), ring_id);
  const PhaseRing::Id unknown_ring_id("unknown ring");
  EXPECT_EQ(dut.GetPhaseRing(unknown_ring_id), nullopt);
  for (const auto rule_id : {rule_id_a, rule_id_b}) {
    result = dut.FindPhaseRing(rule_id);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->id(), ring_id);
  }
  const RightOfWayRule::Id unknown_rule_id("unknown rule");
  EXPECT_EQ(dut.FindPhaseRing(unknown_rule_id), nullopt);
  EXPECT_THROW(dut.RemovePhaseRing(unknown_ring_id), std::logic_error);
  EXPECT_NO_THROW(dut.RemovePhaseRing(ring_id));
  EXPECT_EQ(dut.GetPhaseRing(ring_id), nullopt);
  for (const auto rule_id : {rule_id_a, rule_id_b}) {
    EXPECT_EQ(dut.FindPhaseRing(rule_id), nullopt);
  }
}

// Verifies that an exception is thrown when the user attempts to add a
// different PhaseRing that has the same ID as a previously added PhaseRing.
TEST_F(SimpleRightOfWayPhaseBookTest, RingWithSameId) {
  SimpleRightOfWayPhaseBook dut;
  dut.AddPhaseRing(ring);
  const RightOfWayRule::Id rule_id_c("rule c");
  const Phase different_phase(
      Phase::Id("different phase with different rules"),
      {{rule_id_c, RightOfWayRule::State::Id("c")}});
  const PhaseRing ring_with_same_id(ring_id, {different_phase});
  EXPECT_THROW(dut.AddPhaseRing(ring_with_same_id), std::logic_error);
}

// Verifies that an exception is thrown when the user attempts to add a
// PhaseRing with a unique ID but contains a phase with a RightOfWayRule::Id
// that overlaps the rules covered by a previously added PhaseRing.
TEST_F(SimpleRightOfWayPhaseBookTest, RingWithOverlappingRule) {
  SimpleRightOfWayPhaseBook dut;
  dut.AddPhaseRing(ring);
  const Phase phase_with_overlapping_rule(
      Phase::Id("different phase with overlapping rules"),
      {{rule_id_a, RightOfWayRule::State::Id("a")}});
  const PhaseRing ring_with_overlapping_rule(PhaseRing::Id("unique phase ID"),
                                             {phase_with_overlapping_rule});
  EXPECT_THROW(dut.AddPhaseRing(ring_with_overlapping_rule), std::logic_error);
}

}  // namespace
}  // namespace maliput
}  // namespace drake
