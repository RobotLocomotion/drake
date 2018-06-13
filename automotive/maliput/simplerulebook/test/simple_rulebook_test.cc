#include "drake/automotive/maliput/simplerulebook/simple_rulebook.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_speed_limit_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace simplerulebook {
namespace {

using api::LaneId;
using api::rules::LaneSRange;
using api::rules::LaneSRoute;
using api::rules::RightOfWayRule;
using api::rules::RoadRulebook;
using api::rules::SpeedLimitRule;

const LaneSRange kZone(LaneId("a"), {10., 20.});

const RightOfWayRule kRightOfWay(
    RightOfWayRule::Id("rowr_id"),
    LaneSRoute({kZone}),
    RightOfWayRule::ZoneType::kStopExcluded,
    {RightOfWayRule::State{RightOfWayRule::State::Id("rowr_state_id"),
                           RightOfWayRule::State::Type::kStopThenGo,
                           {}}});

const SpeedLimitRule kSpeedLimit(SpeedLimitRule::Id("slr_id"),
                                 kZone,
                                 SpeedLimitRule::Severity::kStrict,
                                 0., 44.);


GTEST_TEST(SimpleRulebookTest, DefaultConstructor) {
  SimpleRulebook dut;
}


GTEST_TEST(SimpleRulebookTest, AddGetRemoveRightOfWay) {
  SimpleRulebook dut;

  EXPECT_THROW(dut.GetRule(kRightOfWay.id()), std::out_of_range);
  dut.AddRule(kRightOfWay);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kRightOfWay.id()), kRightOfWay));
  EXPECT_THROW(dut.AddRule(kRightOfWay), std::runtime_error);
  dut.RemoveRule(kRightOfWay.id());
  EXPECT_THROW(dut.GetRule(kRightOfWay.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kRightOfWay.id()), std::runtime_error);
}


GTEST_TEST(SimpleRulebookTest, AddGetRemoveSpeedLimit) {
  SimpleRulebook dut;

  EXPECT_THROW(dut.GetRule(kSpeedLimit.id()), std::out_of_range);
  dut.AddRule(kSpeedLimit);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kSpeedLimit.id()), kSpeedLimit));
  EXPECT_THROW(dut.AddRule(kSpeedLimit), std::runtime_error);
  dut.RemoveRule(kSpeedLimit.id());
  EXPECT_THROW(dut.GetRule(kSpeedLimit.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kSpeedLimit.id()), std::runtime_error);
}


GTEST_TEST(SimpleRulebookTest, RemoveAll) {
  SimpleRulebook dut;
  dut.RemoveAll();  // I.e., should work on empty rulebook.
  dut.AddRule(kRightOfWay);
  dut.AddRule(kSpeedLimit);
  dut.RemoveAll();
  EXPECT_THROW(dut.GetRule(kRightOfWay.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kRightOfWay.id()), std::runtime_error);
  EXPECT_THROW(dut.GetRule(kSpeedLimit.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kSpeedLimit.id()), std::runtime_error);
  // Since the original rules are gone, it should be possible to re-add them.
  dut.AddRule(kRightOfWay);
  dut.AddRule(kSpeedLimit);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kRightOfWay.id()), kRightOfWay));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kSpeedLimit.id()), kSpeedLimit));
}


GTEST_TEST(SimpleRulebookTest, FindRules) {
  SimpleRulebook dut;
  dut.AddRule(kSpeedLimit);
  dut.AddRule(kRightOfWay);

  const double kZeroTolerance = 0.;

  const RoadRulebook::QueryResults empty = dut.FindRules({}, kZeroTolerance);
  EXPECT_EQ(empty.right_of_way.size(), 0);
  EXPECT_EQ(empty.speed_limit.size(), 0);

  const RoadRulebook::QueryResults nonempty = dut.FindRules({kZone},
                                                            kZeroTolerance);
  EXPECT_EQ(nonempty.right_of_way.size(), 1);
  EXPECT_EQ(nonempty.speed_limit.size(), 1);

  const LaneSRange kReversedZone(kZone.lane_id(),
                                 {kZone.s_range().s1(), kZone.s_range().s0()});
  const RoadRulebook::QueryResults reversed = dut.FindRules({kReversedZone},
                                                            kZeroTolerance);
  EXPECT_EQ(reversed.right_of_way.size(), 1);
  EXPECT_EQ(reversed.speed_limit.size(), 1);

  const double kNonzeroTolerance = 0.1;

  ASSERT_LT(kZone.s_range().s0(), kZone.s_range().s1());
  // Construct a range that just barely overlaps the kNonzeroTolerance band
  // of kZone.s1.
  const LaneSRange kNearbyRange(
      kZone.lane_id(),
      {kZone.s_range().s1() + (0.9 * kNonzeroTolerance),
       kZone.s_range().s1() + 50.});
  const RoadRulebook::QueryResults nearby = dut.FindRules({kNearbyRange},
                                                          kNonzeroTolerance);
  EXPECT_EQ(nearby.right_of_way.size(), 1);
  EXPECT_EQ(nearby.speed_limit.size(), 1);

  // Construct a range that sits just outside of the kNonzeroTolerance band
  // of kZone.s1.
  const LaneSRange kTooFarRange(
      kZone.lane_id(),
      {kZone.s_range().s1() + (1.1 * kNonzeroTolerance),
       kZone.s_range().s1() + 50.});
  const RoadRulebook::QueryResults toofar = dut.FindRules({kTooFarRange},
                                                          kNonzeroTolerance);
  EXPECT_EQ(toofar.right_of_way.size(), 0);
  EXPECT_EQ(toofar.speed_limit.size(), 0);
}


}  // namespace
}  // namespace simplerulebook
}  // namespace maliput
}  // namespace drake
