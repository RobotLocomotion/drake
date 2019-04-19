/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/rule_state_provider.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

class RuleStateProviderTest : public ::testing::Test {
 protected:
  const RightOfWayRule::Id kExistingId{"aye"};
  const RightOfWayRule::Id kNonExistingId{"nay"};

  const RightOfWayRule::State::Id kCurrentStateId{"state"};
  const RightOfWayRule::State::Id kNextStateId{"next"};
  const double kDurationUntil{99.};

  class MockStateProvider : public RuleStateProvider {
   public:
    explicit MockStateProvider(const RuleStateProviderTest* fixture)
        : fixture_(fixture) {}
   private:
    drake::optional<RightOfWayResult> DoGetState(
        const RightOfWayRule::Id& id) const final {
      if (id == fixture_->kExistingId) {
        return RightOfWayResult{fixture_->kCurrentStateId,
                                RightOfWayResult::Next{
                                    fixture_->kNextStateId,
                                    fixture_->kDurationUntil}};
      }
      return nullopt;
    }

    const RuleStateProviderTest* const fixture_;
  };
};


TEST_F(RuleStateProviderTest, ExerciseInterface) {
  using Result = RuleStateProvider::RightOfWayResult;

  const MockStateProvider dut(this);

  EXPECT_TRUE(dut.GetState(kExistingId).has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetState(kExistingId).value(),
      (Result{kCurrentStateId,
              Result::Next{kNextStateId, kDurationUntil}})));

  EXPECT_FALSE(dut.GetState(kNonExistingId).has_value());
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
