/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/right_of_way_phase.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

class RightOfWayPhaseTest : public ::testing::Test {
 protected:
  RightOfWayPhaseTest()
      : id_("test_id"),
        rule_states_{{RightOfWayRule::Id("northbound-forward"),
                      RightOfWayRule::State::Id("GO")},
                     {RightOfWayRule::Id("southbound-left-turn"),
                      RightOfWayRule::State::Id("STOP")}},
        bulb_states_{{{Bulb::Id("northbound-forward-green"), BulbState::kOn},
                      {Bulb::Id("northbound-forward-red"), BulbState::kOff},
                      {Bulb::Id("southbound-left-turn-green"), BulbState::kOff},
                      {Bulb::Id("southbound-left-turn-red"), BulbState::kOn}}},
        phase_{id_, rule_states_, bulb_states_} {}

  const RightOfWayPhase::Id id_;
  const RuleStates rule_states_;
  const optional<BulbStates> bulb_states_;
  const RightOfWayPhase phase_;
};

TEST_F(RightOfWayPhaseTest, Accessors) {
  for (const optional<BulbStates>& bulb_states :
       std::vector<optional<BulbStates>>{nullopt, bulb_states_}) {
    RightOfWayPhase dut(id_, rule_states_, bulb_states);
    EXPECT_EQ(dut.id(), id_);
    EXPECT_EQ(dut.rule_states(), rule_states_);
    EXPECT_EQ(dut.bulb_states(), bulb_states);
  }
}

TEST_F(RightOfWayPhaseTest, Copying) {
  const RightOfWayPhase dut(phase_);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, phase_));
}

TEST_F(RightOfWayPhaseTest, Assignment) {
  RightOfWayPhase dut(RightOfWayPhase::Id("other_dut_id"), RuleStates());
  dut = phase_;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, phase_));
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
