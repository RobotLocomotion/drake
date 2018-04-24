#include "drake/automotive/maliput/simplerulebook/yaml_io.h"

#include <sstream>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_speed_limit_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"
#include "drake/automotive/maliput/simplerulebook/simple_rulebook.h"
#include "drake/common/drake_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace maliput {
namespace simplerulebook {
namespace {

using api::LaneId;
using api::rules::LaneSRange;
using api::rules::LaneSRoute;
using api::rules::RightOfWayRule;
using api::rules::SpeedLimitRule;
using api::rules::SRange;

GTEST_TEST(YamlIoTest, LoadYaml) {
  SimpleRulebook dut;
  std::istringstream yaml(R"R(# -*- yaml -*-
---
something_before: other-stuff
maliput_simple_rulebook_v1:
  speed_limit:
    sl1:
      zone: [jibby, 1.5, 10.2]
      severity: Strict
      limit:  [23., 92.]
    sl2:
      zone: [jabby, 0., 1000.]
      severity: Advisory
      limit:  34.

  right_of_way:
    rw1:
      controlled_zone:
        - [l0, 0., 90.]
        - [l1, 23., 7.]
        - [l2, 5., 10.]
      type: StopThenGo
something_after: other-stuff
)R");

  LoadYaml(&yaml, &dut);

  const SpeedLimitRule::Id sl1_id("sl1");
  const SpeedLimitRule::Id sl2_id("sl2");

  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetRule(sl1_id),
      SpeedLimitRule(sl1_id,
                     LaneSRange(LaneId("jibby"), SRange(1.5, 10.2)),
                     SpeedLimitRule::Severity::kStrict,
                     23., 92.)));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetRule(SpeedLimitRule::Id("sl2")),
      SpeedLimitRule(SpeedLimitRule::Id("sl2"),
                     LaneSRange(LaneId("jabby"), SRange(0., 1000.)),
                     SpeedLimitRule::Severity::kAdvisory,
                     0., 34.)));

  const RightOfWayRule::Id rw1_id("rw1");

  EXPECT_TRUE(MALIPUT_IS_EQUAL(
      dut.GetRule(rw1_id),
      RightOfWayRule(rw1_id,
                     LaneSRoute({
                         LaneSRange(LaneId("l0"), {0., 90.}),
                         LaneSRange(LaneId("l1"), {23., 7.}),
                         LaneSRange(LaneId("l2"), {5., 10.}),
                             }),
                     RightOfWayRule::Type::kStopThenGo)));
}


// Exercise a non-exhaustive variety of parse errors.
GTEST_TEST(YamlIoTest, LoadYamlParseErrors) {
  SimpleRulebook dut;
  std::istringstream yaml;

  // No simple-rulebook YAML to be found.
  yaml = std::istringstream(R"R(# -*- yaml -*-
---
something_before: other-stuff
something_after: other-stuff
)R");
  DRAKE_EXPECT_THROWS_MESSAGE(LoadYaml(&yaml, &dut), std::runtime_error,
                              ".*invalid node.*");

  // Bad SpeedLimitRule::Severity.
  yaml = std::istringstream(R"R(# -*- yaml -*-
---
maliput_simple_rulebook_v1:
  speed_limit:
    sl1:
      zone: [jibby, 1.5, 10.2]
      severity: StrictXXX
      limit:  [23., 92.]
)R");
  dut.RemoveAll();
  DRAKE_EXPECT_THROWS_MESSAGE(LoadYaml(&yaml, &dut), std::runtime_error,
                              "Unknown SpeedLimitRule::Severity: StrictXXX");

  // Bad LaneSRange.
  yaml = std::istringstream(R"R(# -*- yaml -*-
---
maliput_simple_rulebook_v1:
  speed_limit:
    sl1:
      zone: [jibby, 1.5, 1XXX0.2]
      severity: Strict
      limit:  [23., 92.]
)R");
  dut.RemoveAll();
  DRAKE_EXPECT_THROWS_MESSAGE(LoadYaml(&yaml, &dut), std::runtime_error,
                              ".*bad conversion.*");

  // Bad SpeedLimitRule limit.
  yaml = std::istringstream(R"R(# -*- yaml -*-
---
maliput_simple_rulebook_v1:
  speed_limit:
    sl1:
      zone: [jibby, 1.5, 10.2]
      severity: Strict
      limit:  [23.]
)R");
  dut.RemoveAll();
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadYaml(&yaml, &dut), std::runtime_error,
      "Failure .* condition 'node.size\\(\\) == 2' failed.");

  // Duplicate id.
  yaml = std::istringstream(R"R(# -*- yaml -*-
---
maliput_simple_rulebook_v1:
  speed_limit:
    sl1:
      zone: [jibby, 1.5, 10.2]
      severity: Strict
      limit:  [23., 25.]
    sl1:
      zone: [jibby, 1.5, 10.2]
      severity: Strict
      limit:  [23., 25.]
)R");
  dut.RemoveAll();
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadYaml(&yaml, &dut), std::runtime_error,
      "Failure .* condition 'map_result.second' failed.");

  // Bad RightOfWayRule::Type
  yaml = std::istringstream(R"R(# -*- yaml -*-
---
maliput_simple_rulebook_v1:
  right_of_way:
    rw1:
      controlled_zone:
        - [l0, 0., 90.]
      type: StopThenGoXXX
)R");
  dut.RemoveAll();
  DRAKE_EXPECT_THROWS_MESSAGE(LoadYaml(&yaml, &dut), std::runtime_error,
                              "Unknown RightOfWayRule::Type: StopThenGoXXX");
}


}  // namespace
}  // namespace simplerulebook
}  // namespace maliput
}  // namespace drake
