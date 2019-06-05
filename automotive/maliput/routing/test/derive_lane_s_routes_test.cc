#include "drake/automotive/maliput/routing/derive_lane_s_routes.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/dragway/test_utilities/fixtures.h"
#include "drake/automotive/maliput/multilane/test_utilities/fixtures.h"

namespace drake {
namespace maliput {
namespace routing {

using api::Lane;
using api::LaneId;
using api::LanePosition;
using api::RoadPosition;

using api::rules::LaneSRange;
using api::rules::LaneSRoute;
using api::rules::SRange;

using dragway::DragwayBasedTest;
using multilane::BranchAndMergeBasedTest;
using multilane::LoopBasedTest;
using multilane::MultiBranchBasedTest;

namespace {

class DragwayBasedRouteTest : public DragwayBasedTest {
 protected:
  void CheckRoutes(const std::vector<LaneSRoute>& routes,
                   const LaneSRange& expected_range) {
    ASSERT_EQ(routes.size(), 1);
    ASSERT_EQ(routes.at(0).ranges().size(), 1);
    const LaneSRange& range = routes.at(0).ranges().at(0);
    ASSERT_EQ(range.lane_id(), expected_range.lane_id());
    ASSERT_EQ(range.s_range().s0(), expected_range.s_range().s0());
    ASSERT_EQ(range.s_range().s1(), expected_range.s_range().s1());
  }
};

}  // namespace

TEST_F(DragwayBasedRouteTest, SingleLaneTests) {
  struct TestCase {
    double s_start{};
    double s_end{};
  };
  const std::vector<const Lane*> lanes = {left_lane_, center_lane_,
                                          right_lane_};
  // clang-format off
  const std::vector<TestCase> test_cases = {{0, 0},
                                            {0, kLength},
                                            {kLength, kLength},
                                            {0.25 * kLength, 0.75 * kLength}};
  // clang-format on
  for (const auto& lane : lanes) {
    for (const auto& test_case : test_cases) {
      const RoadPosition start(lane, LanePosition(test_case.s_start, 0, 0));
      const RoadPosition end(lane, LanePosition(test_case.s_end, 0, 0));
      CheckRoutes(
          DeriveLaneSRoutes(start, end, kLength),
          LaneSRange(lane->id(), SRange(test_case.s_start, test_case.s_end)));
    }
  }
}

TEST_F(DragwayBasedRouteTest, CrossLaneTests) {
  struct TestCase {
    const Lane* start_lane{};
    const Lane* end_lane{};
  };
  // clang-format off
  const std::vector<TestCase> test_cases = {{left_lane_, center_lane_},
                                            {left_lane_, right_lane_},
                                            {center_lane_, left_lane_},
                                            {center_lane_, right_lane_},
                                            {right_lane_, center_lane_},
                                            {right_lane_, left_lane_}};
  // clang-format on
  for (const auto& test_case : test_cases) {
    const RoadPosition start(test_case.start_lane, LanePosition(0, 0, 0));
    const RoadPosition end(test_case.end_lane, LanePosition(kLength, 0, 0));
    ASSERT_EQ(DeriveLaneSRoutes(start, end, kLength).size(), 0);
  }
}

namespace {

void CheckRoutes(const std::vector<LaneSRoute>& routes,
                 const std::vector<LaneSRange>& expected_route) {
  ASSERT_EQ(routes.size(), 1);
  const std::vector<LaneSRange>& ranges = routes.at(0).ranges();
  ASSERT_EQ(ranges.size(), expected_route.size());
  for (size_t i = 0; i < ranges.size(); ++i) {
    const LaneSRange& range = ranges.at(i);
    const LaneSRange& expected_range = expected_route.at(i);
    ASSERT_EQ(range.lane_id(), expected_range.lane_id());
    const SRange s_range = range.s_range();
    const SRange expected_s_range = expected_range.s_range();
    ASSERT_EQ(s_range.s0(), expected_s_range.s0());
    ASSERT_EQ(s_range.s1(), expected_s_range.s1());
  }
}

}  // namespace

TEST_F(BranchAndMergeBasedTest, StartLeftLaneToEndLeftLane) {
  const Lane* start_lane = index_.GetLane(LaneId("l:0_1"));
  const Lane* end_lane = index_.GetLane(LaneId("l:3_1"));
  const double start_s = 0;
  const double end_s = end_lane->length();
  const RoadPosition start(start_lane, LanePosition(start_s, 0, 0));
  const RoadPosition end(end_lane, LanePosition(end_s, 0, 0));
  CheckRoutes(
      DeriveLaneSRoutes(start, end, total_length_),
      {LaneSRange(LaneId("l:0_1"), SRange(start_s, start_lane->length())),
       LaneSRange(LaneId("l:1.1_0"),
                  SRange(0, index_.GetLane(LaneId("l:1.1_0"))->length())),
       LaneSRange(LaneId("l:1.2_0"),
                  SRange(0, index_.GetLane(LaneId("l:1.2_0"))->length())),
       LaneSRange(LaneId("l:1.3_0"),
                  SRange(0, index_.GetLane(LaneId("l:1.3_0"))->length())),
       LaneSRange(LaneId("l:3_1"), SRange(0, end_s))});
}

TEST_F(BranchAndMergeBasedTest, EndLeftLaneToStartLeftLane) {
  const Lane* start_lane = index_.GetLane(LaneId("l:3_1"));
  const Lane* end_lane = index_.GetLane(LaneId("l:0_1"));
  // Checks route that ends mid-lane.
  const double start_s = start_lane->length() / 2;
  const double end_s = 0;
  const RoadPosition start(start_lane, LanePosition(start_s, 0, 0));
  const RoadPosition end(end_lane, LanePosition(end_s, 0, 0));
  CheckRoutes(
      DeriveLaneSRoutes(start, end, total_length_),
      {LaneSRange(LaneId("l:3_1"), SRange(start_s, 0)),
       LaneSRange(LaneId("l:1.3_0"),
                  SRange(index_.GetLane(LaneId("l:1.3_0"))->length(), 0)),
       LaneSRange(LaneId("l:1.2_0"),
                  SRange(index_.GetLane(LaneId("l:1.2_0"))->length(), 0)),
       LaneSRange(LaneId("l:1.1_0"),
                  SRange(index_.GetLane(LaneId("l:1.1_0"))->length(), 0)),
       LaneSRange(LaneId("l:0_1"), SRange(end_lane->length(), end_s))});
}

// Verifies DeriveLaneSRoutes() can handle situations when the starting and
// ending road positions result in no continuous S curve. This happens when the
// path must jump to a lane to the left or right.
TEST_F(BranchAndMergeBasedTest, NoContinuousSPath) {
  const Lane* start_lane = index_.GetLane(LaneId("l:0_0"));
  const Lane* end_lane = index_.GetLane(LaneId("l:3_1"));
  const RoadPosition start(start_lane, LanePosition(0, 0, 0));
  const RoadPosition end(end_lane, LanePosition(end_lane->length(), 0, 0));
  ASSERT_EQ(DeriveLaneSRoutes(start, end, total_length_).size(), 0);
}

// Verifies DeriveLaneSRoutes() doesn't enter an infinite loop.
TEST_F(LoopBasedTest, DeriveLaneSRoutesNoInfiniteLoop) {
  // Arbitrary large number to avoid hitting limit.
  constexpr double kMaxLength{1e6};
  const Lane* start_lane = index_.GetLane(LaneId("l:0_0"));
  const Lane* end_lane = index_.GetLane(LaneId("l:4_0"));
  const RoadPosition start(start_lane, LanePosition(0, 0, 0));
  const RoadPosition end(end_lane, LanePosition(end_lane->length(), 0, 0));
  const std::vector<LaneSRoute> routes =
      DeriveLaneSRoutes(start, end, kMaxLength);
  ASSERT_EQ(routes.size(), 5);
}

// Verifies DeriveLaneSRoutes() can handle start / end positions that are in the
// same lane where there is a loop from one end of the lane to the other end.
TEST_F(LoopBasedTest, LoopBackLane) {
  struct TestCase {
    const double start_s_fraction;
    const double end_s_fraction;
  };
  const std::vector<TestCase> test_cases{{0, 1}, {0.25, 0.75}};
  constexpr double kMaxLength{
      1e6};  // Arbitrary large number to avoid hitting limit.
  const Lane* start_and_end_lane = index_.GetLane(LaneId("l:2_0"));
  for (const auto& test_case : test_cases) {
    const double start_s =
        test_case.start_s_fraction * start_and_end_lane->length();
    const double end_s =
        test_case.end_s_fraction * start_and_end_lane->length();
    const RoadPosition start(start_and_end_lane, LanePosition(start_s, 0, 0));
    const RoadPosition end(start_and_end_lane, LanePosition(end_s, 0, 0));
    CheckRoutes(DeriveLaneSRoutes(start, end, kMaxLength),
                {LaneSRange(LaneId("l:2_0"), SRange(start_s, end_s))});
    CheckRoutes(DeriveLaneSRoutes(end, start, kMaxLength),
                {LaneSRange(LaneId("l:2_0"), SRange(end_s, start_s))});
  }
}

// Verifies DeriveLaneSRoutes() can handle multi-branch intersections.
TEST_F(MultiBranchBasedTest, DeriveLaneSRoutesTest) {
  // Arbitrary large number to avoid hitting limit.
  constexpr double kMaxLength{1e6};
  const std::vector<std::pair<const Lane*, const Lane*>> test_cases = {
      std::make_pair(index_.GetLane(LaneId("l:0_0")),
                     index_.GetLane(LaneId("l:1.1_0"))),
      std::make_pair(index_.GetLane(LaneId("l:0_0")),
                     index_.GetLane(LaneId("l:2.1_0"))),
      std::make_pair(index_.GetLane(LaneId("l:0_0")),
                     index_.GetLane(LaneId("l:3.1_0")))};
  for (const auto& test_case : test_cases) {
    const RoadPosition start(test_case.first, LanePosition(0, 0, 0));
    const RoadPosition end(test_case.second,
                           LanePosition(test_case.second->length(), 0, 0));
    CheckRoutes(DeriveLaneSRoutes(start, end, kMaxLength),
                {LaneSRange(LaneId(test_case.first->id().string()),
                            SRange(0, test_case.first->length())),
                 LaneSRange(LaneId(test_case.second->id().string()),
                            SRange(0, test_case.second->length()))});
  }
}

}  // namespace routing
}  // namespace maliput
}  // namespace drake
