/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/phase.h"
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

class PhaseTest : public ::testing::Test {
 protected:
  PhaseTest()
      : id_("test_id"),
        rule_states_{{RightOfWayRule::Id("northbound-forward"),
                      RightOfWayRule::State::Id("GO")},
                     {RightOfWayRule::Id("southbound-left-turn"),
                      RightOfWayRule::State::Id("STOP")}},
        bulb_states_{
            {{{TrafficLight::Id("major-intersection"),
               BulbGroup::Id("northbound"), Bulb::Id("forward-green")},
              BulbState::kOn},
             {{TrafficLight::Id("major-intersection"),
               BulbGroup::Id("northbound"), Bulb::Id("forward-red")},
              BulbState::kOff},
             {{TrafficLight::Id("major-intersection"),
               BulbGroup::Id("southbound"), Bulb::Id("left-turn-green")},
              BulbState::kOff},
             {{TrafficLight::Id("major-intersection"),
               BulbGroup::Id("southbound"), Bulb::Id("left-turn-red")},
              BulbState::kOn}}},
        phase_{id_, rule_states_, bulb_states_} {}

  const Phase::Id id_;
  const RuleStates rule_states_;
  const optional<BulbStates> bulb_states_;
  const Phase phase_;
};

TEST_F(PhaseTest, Accessors) {
  for (const optional<BulbStates>& bulb_states :
       std::vector<optional<BulbStates>>{nullopt, bulb_states_}) {
    Phase dut(id_, rule_states_, bulb_states);
    EXPECT_EQ(dut.id(), id_);
    EXPECT_EQ(dut.rule_states(), rule_states_);
    EXPECT_EQ(dut.bulb_states(), bulb_states);
  }
}

TEST_F(PhaseTest, Copying) {
  const Phase dut(phase_);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, phase_));
}

TEST_F(PhaseTest, Assignment) {
  Phase dut(Phase::Id("other_dut_id"), RuleStates());
  dut = phase_;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, phase_));
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
