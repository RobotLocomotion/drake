/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/regions.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {


const LaneSRange kZone(LaneId("the_lane"), SRange(13., 15.));


GTEST_TEST(SpeedLimitRuleTest, Construction) {
  EXPECT_NO_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                                 SpeedLimitRule::Severity::kStrict,
                                 33., 77.));
  EXPECT_NO_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                                 SpeedLimitRule::Severity::kStrict,
                                 0., 77.));
  EXPECT_NO_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                                 SpeedLimitRule::Severity::kStrict,
                                 0., 0.));

  // Min must not be greater than max.
  EXPECT_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                              SpeedLimitRule::Severity::kStrict,
                              90., 77.),
               std::runtime_error);
  // Negative limits are not allowed.
  EXPECT_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                              SpeedLimitRule::Severity::kStrict,
                              -8., 77.),
               std::runtime_error);
  EXPECT_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone,
                              SpeedLimitRule::Severity::kStrict,
                              -8., -77.),
               std::runtime_error);
}


GTEST_TEST(SpeedLimitRuleTest, Accessors) {
  const SpeedLimitRule dut(SpeedLimitRule::Id("dut_id"), kZone,
                           SpeedLimitRule::Severity::kStrict,
                           5., 8.);
  EXPECT_EQ(dut.id(), SpeedLimitRule::Id("dut_id"));
  EXPECT_EQ(dut.zone().lane_id(), kZone.lane_id());
  EXPECT_EQ(dut.zone().s_range(), kZone.s_range());
  EXPECT_EQ(dut.severity(), SpeedLimitRule::Severity::kStrict);
  EXPECT_EQ(dut.min(), 5.);
  EXPECT_EQ(dut.max(), 8.);

  // Test another severity and the effectively-no-minimum case as well.
  const SpeedLimitRule dut2(SpeedLimitRule::Id("dut_id"), kZone,
                            SpeedLimitRule::Severity::kAdvisory,
                            0., 8.);
  EXPECT_EQ(dut2.severity(), SpeedLimitRule::Severity::kAdvisory);
  EXPECT_EQ(dut2.min(), 0.);
}


GTEST_TEST(SpeedLimitRuleTest, Copying) {
  const SpeedLimitRule source(SpeedLimitRule::Id("dut_id"), kZone,
                              SpeedLimitRule::Severity::kStrict,
                              5., 8.);

  const SpeedLimitRule dut(source);
  EXPECT_EQ(dut.id(), source.id());
  EXPECT_EQ(dut.zone().lane_id(), source.zone().lane_id());
  EXPECT_EQ(dut.zone().s_range(), source.zone().s_range());
  EXPECT_EQ(dut.severity(), source.severity());
  EXPECT_EQ(dut.max(), source.max());
  EXPECT_EQ(dut.min(), source.min());
}


GTEST_TEST(SpeedLimitRuleTest, Assignment) {
  const SpeedLimitRule source(SpeedLimitRule::Id("dut_id"), kZone,
                              SpeedLimitRule::Severity::kStrict,
                              0., 8.);
  SpeedLimitRule dut(SpeedLimitRule::Id("other_id"), kZone,
                     SpeedLimitRule::Severity::kAdvisory,
                     70., 90.);

  dut = source;
  EXPECT_EQ(dut.id(), source.id());
  EXPECT_EQ(dut.zone().lane_id(), source.zone().lane_id());
  EXPECT_EQ(dut.zone().s_range(), source.zone().s_range());
  EXPECT_EQ(dut.severity(), source.severity());
  EXPECT_EQ(dut.max(), source.max());
  EXPECT_EQ(dut.min(), source.min());
}


}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
