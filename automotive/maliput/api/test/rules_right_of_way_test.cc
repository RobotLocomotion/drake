/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {

// Tests for RightOfWayRule::State

const LaneSRoute kZone({LaneSRange(LaneId("a"), {0., 9.}),
                        LaneSRange(LaneId("b"), {17., 12.})});

const RightOfWayRule::State::YieldGroup kYieldGroupSize2{
  RightOfWayRule::Id("other_rule_a"),
  RightOfWayRule::Id("other_rule_b")};

GTEST_TEST(RightOfWayRuleStateTest, Construction) {
  EXPECT_NO_THROW(RightOfWayRule::State(
      RightOfWayRule::State::Id("some_id"),
      RightOfWayRule::State::Type::kStop,
      {}));
  EXPECT_NO_THROW(RightOfWayRule::State(
      RightOfWayRule::State::Id("some_id"),
      RightOfWayRule::State::Type::kGo,
      kYieldGroupSize2));
}


GTEST_TEST(RightOfWayRuleStateTest, Accessors) {
  const RightOfWayRule::State dut(RightOfWayRule::State::Id("dut_id"),
                                  RightOfWayRule::State::Type::kStop,
                                  kYieldGroupSize2);
  EXPECT_EQ(dut.id(), RightOfWayRule::State::Id("dut_id"));
  EXPECT_EQ(dut.type(), RightOfWayRule::State::Type::kStop);
  EXPECT_EQ(dut.yield_to(), kYieldGroupSize2);
}


GTEST_TEST(RightOfWayRuleStateTest, Copying) {
  const RightOfWayRule::State source(RightOfWayRule::State::Id("dut_id"),
                                     RightOfWayRule::State::Type::kStopThenGo,
                                     kYieldGroupSize2);
  const RightOfWayRule::State dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}


GTEST_TEST(RightOfWayRuleStateTest, Assignment) {
  const RightOfWayRule::State source(RightOfWayRule::State::Id("source_id"),
                                     RightOfWayRule::State::Type::kStopThenGo,
                                     {});
  RightOfWayRule::State dut(RightOfWayRule::State::Id("dut_id"),
                            RightOfWayRule::State::Type::kGo,
                            kYieldGroupSize2);
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}


// Tests for RightOfWayRule itself

const RightOfWayRule::State kNoYieldState(RightOfWayRule::State::Id("s1"),
                                          RightOfWayRule::State::Type::kStop,
                                          {});

const RightOfWayRule::State kYieldingState(RightOfWayRule::State::Id("s2"),
                                           RightOfWayRule::State::Type::kGo,
                                           kYieldGroupSize2);


GTEST_TEST(RightOfWayRuleTest, Construction) {
  EXPECT_NO_THROW(RightOfWayRule(
      RightOfWayRule::Id("some_id"),
      kZone, RightOfWayRule::ZoneType::kStopExcluded,
      {kNoYieldState, kYieldingState}));
  EXPECT_NO_THROW(RightOfWayRule(
      RightOfWayRule::Id("some_id"),
      kZone, RightOfWayRule::ZoneType::kStopExcluded,
      {kNoYieldState}));

  // At least one State must be provided.
  EXPECT_THROW(RightOfWayRule(
      RightOfWayRule::Id("some_id"),
      kZone, RightOfWayRule::ZoneType::kStopExcluded,
      {}),
      std::exception);
  // At least one State::Id's must be unique.
  const RightOfWayRule::State kDupIdState(RightOfWayRule::State::Id("s1"),
                                          RightOfWayRule::State::Type::kGo,
                                          {});
  EXPECT_THROW(RightOfWayRule(
      RightOfWayRule::Id("some_id"),
      kZone, RightOfWayRule::ZoneType::kStopExcluded,
      {kNoYieldState, kDupIdState}),
      std::exception);
}


GTEST_TEST(RightOfWayRuleTest, Accessors) {
  const RightOfWayRule dut(
      RightOfWayRule::Id("dut_id"),
      kZone,
      RightOfWayRule::ZoneType::kStopExcluded,
      {kNoYieldState, kYieldingState});
  EXPECT_EQ(dut.id(), RightOfWayRule::Id("dut_id"));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.zone(), kZone));
  EXPECT_EQ(dut.zone_type(), RightOfWayRule::ZoneType::kStopExcluded);
  EXPECT_EQ(dut.states().size(), 2);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.states().at(RightOfWayRule::State::Id("s1")),
                               kNoYieldState));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.states().at(RightOfWayRule::State::Id("s2")),
                               kYieldingState));
  EXPECT_FALSE(dut.is_static());
  EXPECT_THROW(dut.static_state(), std::exception);
}


// Specifically test the different results for a static (single state) rule.
GTEST_TEST(RightOfWayRuleTest, StaticRuleOnlyAccessors) {
  const RightOfWayRule dut(
      RightOfWayRule::Id("dut_id"),
      kZone,
      RightOfWayRule::ZoneType::kStopExcluded,
      {kYieldingState});
  EXPECT_TRUE(dut.is_static());
  EXPECT_NO_THROW(dut.static_state());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.static_state(), kYieldingState));
}


GTEST_TEST(RightOfWayRuleTest, Copying) {
  const RightOfWayRule source(
      RightOfWayRule::Id("source_id"),
      kZone, RightOfWayRule::ZoneType::kStopExcluded,
      {kNoYieldState, kYieldingState});

  const RightOfWayRule dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}


GTEST_TEST(RightOfWayRuleTest, Assignment) {
  const RightOfWayRule source(
      RightOfWayRule::Id("source_id"),
      kZone, RightOfWayRule::ZoneType::kStopExcluded,
      {kNoYieldState, kYieldingState});

  RightOfWayRule dut(RightOfWayRule::Id("some_id"),
                     kZone, RightOfWayRule::ZoneType::kStopAllowed,
                     {kNoYieldState});

  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}


class RightOfWayStateProviderTest : public ::testing::Test {
 protected:
  const RightOfWayRule::Id kExistingId{"aye"};
  const RightOfWayRule::Id kNonExistingId{"nay"};

  const RightOfWayRule::State::Id kCurrentStateId{"state"};
  const RightOfWayRule::State::Id kNextStateId{"next"};
  const double kDurationUntil{99.};

  class MockStateProvider : public RightOfWayStateProvider {
   public:
    explicit MockStateProvider(const RightOfWayStateProviderTest* fixture)
        : fixture_(fixture) {}
   private:
    drake::optional<Result> DoGetState(
        const RightOfWayRule::Id& id) const final {
      if (id == fixture_->kExistingId) {
        return Result{fixture_->kCurrentStateId,
                      Result::Next{fixture_->kNextStateId,
                                   fixture_->kDurationUntil}};
      }
      return nullopt;
    }

    const RightOfWayStateProviderTest* const fixture_;
  };
};


TEST_F(RightOfWayStateProviderTest, ExerciseInterface) {
  using Result = RightOfWayStateProvider::Result;

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
