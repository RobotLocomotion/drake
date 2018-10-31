#include "drake/automotive/maliput/simple_phase_book/simple_right_of_way_phase_book.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/right_of_way_phase.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace simple_phase_book {
namespace {

using api::rules::RightOfWayRule;
using api::rules::RightOfWayPhase;
using api::rules::RightOfWayPhaseRing;

GTEST_TEST(SimpleRightOfWayPhaseBookTest, BasicTest) {
  const RightOfWayRule::Id rule_id_a("rule a");
  const RightOfWayRule::Id rule_id_b("rule b");
  const RightOfWayRule::Id unknown_rule_id("unknown rule");
  const RightOfWayPhase phase(RightOfWayPhase::Id("phase"),
                              {{rule_id_a, RightOfWayRule::State::Id("a")},
                               {rule_id_b, RightOfWayRule::State::Id("b")}});
  const RightOfWayPhaseRing::Id ring_id("ring");
  const RightOfWayPhaseRing::Id unknown_ring_id("unknown ring");
  const RightOfWayPhaseRing ring(ring_id, {phase});
  SimpleRightOfWayPhaseBook dut;
  EXPECT_NO_THROW(dut.AddPhaseRing(ring));
  EXPECT_THROW(dut.AddPhaseRing(ring), std::logic_error);
  optional<RightOfWayPhaseRing> result = dut.GetPhaseRing(ring_id);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result->id(), ring_id);
  EXPECT_EQ(dut.GetPhaseRing(unknown_ring_id), nullopt);
  for (const auto rule_id : {rule_id_a, rule_id_b}) {
    result = dut.FindPhaseRing(rule_id);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->id(), ring_id);
  }
  EXPECT_EQ(dut.FindPhaseRing(unknown_rule_id), nullopt);
  EXPECT_THROW(dut.RemovePhaseRing(unknown_ring_id), std::logic_error);
  EXPECT_NO_THROW(dut.RemovePhaseRing(ring_id));
  EXPECT_EQ(dut.GetPhaseRing(ring_id), nullopt);
  for (const auto rule_id : {rule_id_a, rule_id_b}) {
    EXPECT_EQ(dut.FindPhaseRing(rule_id), nullopt);
  }
}

}  // namespace
}  // namespace simple_phase_book
}  // namespace maliput
}  // namespace drake
