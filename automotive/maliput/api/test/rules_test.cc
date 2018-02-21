#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {


GTEST_TEST(RightOfWayRuleTest, Construction) {
  EXPECT_NO_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"),
                                 RightOfWayRule::Type::kStopThenGo));
  EXPECT_NO_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"),
                                 RightOfWayRule::Type::kDynamic));
}

GTEST_TEST(RightOfWayRuleTest, Accessors) {
  const RightOfWayRule dut(RightOfWayRule::Id("dut_id"),
                           RightOfWayRule::Type::kStopThenGo);
  EXPECT_EQ(dut.id(), RightOfWayRule::Id("dut_id"));
  EXPECT_EQ(dut.type(), RightOfWayRule::Type::kStopThenGo);
}

GTEST_TEST(RightOfWayRuleTest, Copying) {
  const RightOfWayRule source(RightOfWayRule::Id("dut_id"),
                              RightOfWayRule::Type::kStopThenGo);

  const RightOfWayRule dut(source);
  EXPECT_EQ(dut.id(), source.id());
  EXPECT_EQ(dut.type(), source.type());
}

GTEST_TEST(RightOfWayRuleTest, Assignment) {
  const RightOfWayRule source(RightOfWayRule::Id("dut_id"),
                              RightOfWayRule::Type::kStopThenGo);
  RightOfWayRule dut(RightOfWayRule::Id("other_id"),
                      RightOfWayRule::Type::kYield);
  dut = source;
  EXPECT_EQ(dut.id(), source.id());
  EXPECT_EQ(dut.type(), source.type());
}


}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
