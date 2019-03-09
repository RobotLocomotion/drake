/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/test/mock.h"
#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

// Tests for RightOfWayRule::State

GTEST_TEST(RightOfWayRuleStateTest, Construction) {
  EXPECT_NO_THROW(RightOfWayRule::State(RightOfWayRule::State::Id("some_id"),
                                        RightOfWayRule::State::Type::kStop,
                                        {}));
  EXPECT_NO_THROW(RightOfWayRule::State(RightOfWayRule::State::Id("some_id"),
                                        RightOfWayRule::State::Type::kGo,
                                        Mock::YieldGroup2()));
}

GTEST_TEST(RightOfWayRuleStateTest, Accessors) {
  const RightOfWayRule::State dut(RightOfWayRule::State::Id("dut_id"),
                                  RightOfWayRule::State::Type::kStop,
                                  Mock::YieldGroup2());
  EXPECT_EQ(dut.id(), RightOfWayRule::State::Id("dut_id"));
  EXPECT_EQ(dut.type(), RightOfWayRule::State::Type::kStop);
  EXPECT_EQ(dut.yield_to(), Mock::YieldGroup2());
}

GTEST_TEST(RightOfWayRuleStateTest, Copying) {
  const RightOfWayRule::State source(RightOfWayRule::State::Id("dut_id"),
                                     RightOfWayRule::State::Type::kStopThenGo,
                                     Mock::YieldGroup2());
  const RightOfWayRule::State dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(RightOfWayRuleStateTest, Assignment) {
  const RightOfWayRule::State source(RightOfWayRule::State::Id("source_id"),
                                     RightOfWayRule::State::Type::kStopThenGo,
                                     {});
  RightOfWayRule::State dut(RightOfWayRule::State::Id("dut_id"),
                            RightOfWayRule::State::Type::kGo,
                            Mock::YieldGroup2());
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

// Tests for RightOfWayRule itself

GTEST_TEST(RightOfWayRuleTest, Construction) {
  EXPECT_NO_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"),
                                 Mock::LaneSRoute(),
                                 RightOfWayRule::ZoneType::kStopExcluded,
                                 {Mock::NoYieldState(), Mock::YieldState()}));
  EXPECT_NO_THROW(RightOfWayRule(
      RightOfWayRule::Id("some_id"), Mock::LaneSRoute(),
      RightOfWayRule::ZoneType::kStopExcluded, {Mock::NoYieldState()}));

  // At least one State must be provided.
  EXPECT_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"), Mock::LaneSRoute(),
                              RightOfWayRule::ZoneType::kStopExcluded, {}),
               std::exception);
  // At least one State::Id's must be unique.
  const RightOfWayRule::State kDupIdState(RightOfWayRule::State::Id("s1"),
                                          RightOfWayRule::State::Type::kGo, {});
  EXPECT_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"), Mock::LaneSRoute(),
                              RightOfWayRule::ZoneType::kStopExcluded,
                              {Mock::NoYieldState(), kDupIdState}),
               std::exception);
}

GTEST_TEST(RightOfWayRuleTest, Accessors) {
  const RightOfWayRule dut(RightOfWayRule::Id("dut_id"), Mock::LaneSRoute(),
                           RightOfWayRule::ZoneType::kStopExcluded,
                           {Mock::NoYieldState(), Mock::YieldState()});
  EXPECT_EQ(dut.id(), RightOfWayRule::Id("dut_id"));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.zone(), Mock::LaneSRoute()));
  EXPECT_EQ(dut.zone_type(), RightOfWayRule::ZoneType::kStopExcluded);
  EXPECT_EQ(dut.states().size(), 2);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.states().at(RightOfWayRule::State::Id("s1")),
                               Mock::NoYieldState()));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.states().at(RightOfWayRule::State::Id("s2")),
                               Mock::YieldState()));
  EXPECT_FALSE(dut.is_static());
  EXPECT_THROW(dut.static_state(), std::exception);
}

// Specifically test the different results for a static (single state) rule.
GTEST_TEST(RightOfWayRuleTest, StaticRuleOnlyAccessors) {
  const RightOfWayRule dut(RightOfWayRule::Id("dut_id"), Mock::LaneSRoute(),
                           RightOfWayRule::ZoneType::kStopExcluded,
                           {Mock::YieldState()});
  EXPECT_TRUE(dut.is_static());
  EXPECT_NO_THROW(dut.static_state());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.static_state(), Mock::YieldState()));
}

GTEST_TEST(RightOfWayRuleTest, Copying) {
  const RightOfWayRule source(RightOfWayRule::Id("source_id"),
                              Mock::LaneSRoute(),
                              RightOfWayRule::ZoneType::kStopExcluded,
                              {Mock::NoYieldState(), Mock::YieldState()});

  const RightOfWayRule dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(RightOfWayRuleTest, Assignment) {
  const RightOfWayRule source(RightOfWayRule::Id("source_id"),
                              Mock::LaneSRoute(),
                              RightOfWayRule::ZoneType::kStopExcluded,
                              {Mock::NoYieldState(), Mock::YieldState()});

  RightOfWayRule dut(RightOfWayRule::Id("some_id"), Mock::LaneSRoute(),
                     RightOfWayRule::ZoneType::kStopAllowed,
                     {Mock::NoYieldState()});

  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
