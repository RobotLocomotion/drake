#include "drake/automotive/maliput/base/road_rulebook_loader.h"

#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/test_utilities/rules_direction_usage_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_right_of_way_compare.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/loader.h"
#include "drake/common/drake_optional.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace maliput {
namespace {

using drake::maliput::api::LaneId;
using drake::maliput::api::rules::DirectionUsageRule;
using drake::maliput::api::rules::LaneSRange;
using drake::maliput::api::rules::LaneSRoute;
using drake::maliput::api::rules::RightOfWayRule;
using drake::maliput::api::rules::SRange;

class TestLoading2x2IntersectionRules : public ::testing::Test {
 protected:
  TestLoading2x2IntersectionRules()
      : filepath_(FindResourceOrThrow(
            "drake/automotive/maliput/multilane/2x2_intersection.yaml")),
        road_geometry_(
            multilane::LoadFile(multilane::BuilderFactory(), filepath_)) {}

  std::vector<RightOfWayRule> CreateStraightThroughRightOfWayRules() const {
    struct TestCase {
      std::string rule_name;
      std::string lane_name;
    };
    const std::vector<TestCase> test_cases = {
        {"NorthStraight", "l:ns_intersection_segment_0"},
        {"SouthStraight", "l:ns_intersection_segment_1"},
        {"EastStraight", "l:ew_intersection_segment_0"},
        {"WestStraight", "l:ew_intersection_segment_1"},
    };
    std::vector<RightOfWayRule> result;
    for (const auto& test_case : test_cases) {
      result.push_back(RightOfWayRule(
          RightOfWayRule::Id(test_case.rule_name),
          LaneSRoute(
              {LaneSRange(LaneId(test_case.lane_name), SRange(0, 18.75))}),
          RightOfWayRule::ZoneType::kStopExcluded,
          {RightOfWayRule::State(RightOfWayRule::State::Id("Go"),
                                 RightOfWayRule::State::Type::kGo, {}),
           RightOfWayRule::State(RightOfWayRule::State::Id("Stop"),
                                 RightOfWayRule::State::Type::kStop, {})}));
    }
    return result;
  }

  std::vector<RightOfWayRule> CreateRightTurnRightOfWayRules() const {
    struct TestCase {
      std::string rule_name;
      std::string lane_name;
      std::string yield_1;
      std::string yield_2;
    };
    const std::vector<TestCase> test_cases = {
        {"NorthRightTurn", "l:north_right_turn_segment_0", "EastStraight",
         "SouthLeftTurn"},
        {"SouthRightTurn", "l:south_right_turn_segment_0", "WestStraight",
         "NorthLeftTurn"},
        {"EastRightTurn", "l:east_right_turn_segment_0", "SouthStraight",
         "WestLeftTurn"},
        {"WestRightTurn", "l:west_right_turn_segment_0", "NorthStraight",
         "EastLeftTurn"},
    };
    std::vector<RightOfWayRule> result;
    for (const auto& test_case : test_cases) {
      result.push_back(RightOfWayRule(
          RightOfWayRule::Id(test_case.rule_name),
          LaneSRoute({LaneSRange(LaneId(test_case.lane_name),
                                 SRange(0, M_PI_2 * 7.5))}),
          RightOfWayRule::ZoneType::kStopExcluded,
          {RightOfWayRule::State(RightOfWayRule::State::Id("Go"),
                                 RightOfWayRule::State::Type::kGo,
                                 {RightOfWayRule::Id(test_case.yield_2)}),
           RightOfWayRule::State(RightOfWayRule::State::Id("StopThenGo"),
                                 RightOfWayRule::State::Type::kStopThenGo,
                                 {RightOfWayRule::Id(test_case.yield_1),
                                  RightOfWayRule::Id(test_case.yield_2)})}));
    }
    return result;
  }

  std::vector<RightOfWayRule> CreateLeftTurnRightOfWayRules() const {
    struct TestCase {
      std::string rule_name;
      std::string lane_name;
      std::string yield;
    };
    std::vector<TestCase> test_cases = {
        {"NorthLeftTurn", "l:north_left_turn_segment_0", "SouthStraight"},
        {"SouthLeftTurn", "l:south_left_turn_segment_0", "NorthStraight"},
        {"EastLeftTurn", "l:east_left_turn_segment_0", "WestStraight"},
        {"WestLeftTurn", "l:west_left_turn_segment_0", "EastStraight"},
    };
    std::vector<RightOfWayRule> result;
    for (const auto& test_case : test_cases) {
      result.push_back(RightOfWayRule(
          RightOfWayRule::Id(test_case.rule_name),
          LaneSRoute({LaneSRange(LaneId(test_case.lane_name),
                                 SRange(0, M_PI_2 * 11.25))}),
          RightOfWayRule::ZoneType::kStopExcluded,
          {RightOfWayRule::State(RightOfWayRule::State::Id("Go"),
                                 RightOfWayRule::State::Type::kGo,
                                 {RightOfWayRule::Id(test_case.yield)}),
           RightOfWayRule::State(RightOfWayRule::State::Id("Stop"),
                                 RightOfWayRule::State::Type::kStop, {})}));
    }
    return result;
  }

  std::vector<DirectionUsageRule> CreateDirectionUsageRules() const {
    struct TestCase {
      std::string rule_name;
      std::string lane_name;
      DirectionUsageRule::State::Type rule_type;
      DirectionUsageRule::State::Severity rule_severity;
    };
    const std::vector<TestCase> test_cases = {
        // Turn cases.
        {"NorthRightTurn", "l:north_right_turn_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"NorthLeftTurn", "l:north_left_turn_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"SouthRightTurn", "l:south_right_turn_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"SouthLeftTurn", "l:south_left_turn_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"EastRightTurn", "l:east_right_turn_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"EastLeftTurn", "l:east_left_turn_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"WestRightTurn", "l:west_right_turn_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"WestLeftTurn", "l:west_left_turn_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        // Straight segments.
        {"NorthApproach", "l:s_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"NorthStraight", "l:ns_intersection_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"NorthExit", "l:n_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"SouthApproach", "l:n_segment_1",
         DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"SouthStraight", "l:ns_intersection_segment_1",
         DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kStrict},
        {"SouthExit", "l:s_segment_1",
         DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"EastApproach", "l:w_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"EastStraight", "l:ew_intersection_segment_0",
         DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"EastExit", "l:e_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"WestEntrance", "l:e_segment_1",
         DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"WestStraight", "l:ew_intersection_segment_1",
         DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kStrict},
        {"WestExit", "l:w_segment_1",
         DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kPreferred},
    };
    std::vector<DirectionUsageRule> result;
    for (const auto& test_case : test_cases) {
      const auto lane =
          road_geometry_->ById().GetLane(LaneId(test_case.lane_name));
      result.push_back(DirectionUsageRule(
          DirectionUsageRule::Id(test_case.rule_name),
          LaneSRange(LaneId(test_case.lane_name), SRange(0, lane->length())),
          {DirectionUsageRule::State(DirectionUsageRule::State::Id("default"),
                                     test_case.rule_type,
                                     test_case.rule_severity)}));
    }
    return result;
  }

  const std::string filepath_;
  const std::unique_ptr<const api::RoadGeometry> road_geometry_;
};

TEST_F(TestLoading2x2IntersectionRules, LoadFromFile) {
  const std::unique_ptr<api::rules::RoadRulebook> rulebook =
      LoadRoadRulebookFromFile(road_geometry_.get(), filepath_);
  EXPECT_NE(rulebook, nullptr);

  // RightOfWayRules testing.
  {
    const auto straight_cases = CreateStraightThroughRightOfWayRules();
    const auto right_turn_cases = CreateRightTurnRightOfWayRules();
    const auto left_turn_cases = CreateLeftTurnRightOfWayRules();

    std::vector<RightOfWayRule> test_cases;
    test_cases.insert(test_cases.end(), std::begin(straight_cases),
                      std::end(straight_cases));
    test_cases.insert(test_cases.end(), std::begin(right_turn_cases),
                      std::end(right_turn_cases));
    test_cases.insert(test_cases.end(), std::begin(left_turn_cases),
                      std::end(left_turn_cases));

    for (const auto& test_case : test_cases) {
      const RightOfWayRule rule = rulebook->GetRule(test_case.id());
      EXPECT_TRUE(MALIPUT_IS_EQUAL(rule, test_case));
    }
  }

  // DirectionUsageRules testing.
  {
    std::vector<DirectionUsageRule> direction_usage_cases =
        CreateDirectionUsageRules();
    for (const auto& test_case : direction_usage_cases) {
      const DirectionUsageRule rule = rulebook->GetRule(test_case.id());
      EXPECT_TRUE(MALIPUT_IS_EQUAL(rule, test_case));
    }
  }
}

}  // namespace
}  // namespace maliput
}  // namespace drake
