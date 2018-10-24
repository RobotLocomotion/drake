#include "drake/automotive/maliput/simple_phase_book/simple_right_of_way_phase_book.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"

namespace drake {
namespace maliput {
namespace simple_phase_book {
namespace {

using api::rules::RightOfWayRule;
using api::rules::RightOfWayPhaseRing;

GTEST_TEST(SimpleRightOfWayPhaseBookTest, BasicTest) {
  const RightOfWayRule::Id rule_id("foo");
  const RightOfWayPhaseRing::Id ring_id("bar");
  SimpleRightOfWayPhaseBook dut;
  EXPECT_NO_THROW(dut.AddEntry(rule_id, ring_id));
  EXPECT_EQ(dut.GetPhaseRing(rule_id), ring_id);
  EXPECT_THROW(dut.AddEntry(rule_id, ring_id), std::logic_error);
}

}  // namespace
}  // namespace simple_phase_book
}  // namespace maliput
}  // namespace drake
