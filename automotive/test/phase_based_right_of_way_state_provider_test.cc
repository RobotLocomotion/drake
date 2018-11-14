#include "drake/automotive/phase_based_right_of_way_state_provider.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/simple_phase_book/simple_right_of_way_phase_book.h"
#include "drake/automotive/maliput/simple_phase_provider/simple_right_of_way_phase_provider.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::rules::RightOfWayPhase;
using maliput::api::rules::RightOfWayPhaseRing;
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::RightOfWayStateProvider;
using maliput::simple_phase_book::SimpleRightOfWayPhaseBook;
using maliput::simple_phase_provider::SimpleRightOfWayPhaseProvider;

GTEST_TEST(PhaseBasedRightOfWayStateProviderTest, BasicTest) {
  const RightOfWayRule::Id rule_id_a("rule a");
  const RightOfWayRule::Id rule_id_b("rule b");

  const RightOfWayRule::State::Id state_id_go("GO");
  const RightOfWayRule::State::Id state_id_stop("STOP");

  const RightOfWayPhase::Id phase_id_1("phase1");
  const RightOfWayPhase phase1(phase_id_1,
      {{rule_id_a, state_id_go},
       {rule_id_b, state_id_stop}});
  const RightOfWayPhase::Id phase_id_2("phase2");
  const RightOfWayPhase phase2(phase_id_2,
      {{rule_id_a, state_id_stop},
       {rule_id_b, state_id_go}});

  const RightOfWayPhaseRing::Id ring_id("ring");
  const RightOfWayPhaseRing ring(ring_id, {phase1, phase2});

  SimpleRightOfWayPhaseBook phase_book;
  phase_book.AddPhaseRing(ring);

  SimpleRightOfWayPhaseProvider phase_provider;
  phase_provider.AddPhaseRing(ring_id, phase_id_1);

  PhaseBasedRightOfWayStateProvider dut(&phase_book, &phase_provider);

  EXPECT_EQ(&dut.phase_book(), &phase_book);
  EXPECT_EQ(&dut.phase_provider(), &phase_provider);

  struct ExpectedState {
    const RightOfWayRule::Id rule;
    const RightOfWayStateProvider::Result result;
  };

  auto compare_expected = [&](const std::vector<ExpectedState>& test_cases) {
    for (const auto& test : test_cases) {
      optional<RightOfWayStateProvider::Result> result =
          dut.GetState(test.rule);
      EXPECT_TRUE(result.has_value());
      EXPECT_EQ(result->current_id, test.result.current_id);
      if (test.result.next) {
        EXPECT_EQ(result->next->id, test.result.next->id);
        if (test.result.next->duration_until) {
          EXPECT_EQ(*result->next->duration_until,
                    *test.result.next->duration_until);
        } else {
          EXPECT_EQ(result->next->duration_until, nullopt);
        }
      } else {
        EXPECT_EQ(result->next, nullopt);
      }
    }
  };

  // TODO(liang.fok) Add tests for "next state" in returned results once #9993
  // is resolved.
  const std::vector<ExpectedState> phase_1_test_cases{
      {rule_id_a, {state_id_go, nullopt}},
      {rule_id_b, {state_id_stop, nullopt}}};
  compare_expected(phase_1_test_cases);
  phase_provider.SetPhase(ring_id, phase_id_2);
  const std::vector<ExpectedState> phase_2_test_cases{
      {rule_id_a, {state_id_stop}},
      {rule_id_b, {state_id_go}}};
  compare_expected(phase_2_test_cases);
  EXPECT_FALSE(dut.GetState(RightOfWayRule::Id("unknown rule")).has_value());
}

}  // namespace
}  // namespace automotive
}  // namespace drake
