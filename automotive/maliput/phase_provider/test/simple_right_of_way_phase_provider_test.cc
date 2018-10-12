#include "drake/automotive/maliput/phase_provider/simple_right_of_way_phase_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/right_of_way_phase_provider.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace phase_provider {
namespace {

using api::rules::RightOfWayRule;
using api::rules::RightOfWayPhase;
using api::rules::RightOfWayPhaseProvider;
using api::rules::RightOfWayPhaseRing;

// Fixture for testing the SimpleRightOfWayPhaseProvider.
struct SimpleRightOfWayPhaseProviderTest : public ::testing::Test {
  SimpleRightOfWayPhaseProviderTest()
      : phase_id_1("foo"), phase_id_2("bar"), phase_ring_id("bar") {
  }

  const RightOfWayPhase::Id phase_id_1;
  const RightOfWayPhase::Id phase_id_2;
  const RightOfWayPhaseRing::Id phase_ring_id;
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

}  // namespace
}  // namespace phase_provider
}  // namespace maliput
}  // namespace drake
