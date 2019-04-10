#include "drake/automotive/maliput/base/phase_based_rule_state_provider.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/base/manual_phase_provider.h"
#include "drake/automotive/maliput/base/manual_phase_ring_book.h"

namespace drake {
namespace maliput {
namespace {

using maliput::api::rules::Phase;
using maliput::api::rules::PhaseRing;
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::RuleStateProvider;

GTEST_TEST(PhaseBasedRuleStateProviderTest, BasicTest) {
  const RightOfWayRule::Id rule_id_a("rule a");
  const RightOfWayRule::Id rule_id_b("rule b");

  const RightOfWayRule::State::Id state_id_go("GO");
  const RightOfWayRule::State::Id state_id_stop("STOP");

  const Phase::Id phase_id_1("phase1");
  const Phase phase1(phase_id_1, {{rule_id_a, state_id_go},
                                  {rule_id_b, state_id_stop}});
  const Phase::Id phase_id_2("phase2");
  const Phase phase2(phase_id_2, {{rule_id_a, state_id_stop},
                                  {rule_id_b, state_id_go}});

  const PhaseRing::Id ring_id("ring");
  const PhaseRing ring(ring_id, {phase1, phase2});

  ManualPhaseRingBook phase_ring_book;
  phase_ring_book.AddPhaseRing(ring);

  ManualPhaseProvider phase_provider;
  phase_provider.AddPhaseRing(ring_id, phase_id_1);

  PhaseBasedRuleStateProvider dut(&phase_ring_book, &phase_provider);

  EXPECT_EQ(&dut.phase_ring_book(), &phase_ring_book);
  EXPECT_EQ(&dut.phase_provider(), &phase_provider);

  struct ExpectedState {
    const RightOfWayRule::Id rule;
    const RuleStateProvider::RightOfWayResult result;
  };

  auto compare_expected = [&](const std::vector<ExpectedState>& test_cases) {
    for (const auto& test : test_cases) {
      optional<RuleStateProvider::RightOfWayResult> result =
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
}  // namespace maliput
}  // namespace drake
