#include "drake/automotive/maliput/utility/derive_lane_s_route.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/loader.h"

namespace drake {
namespace maliput {
namespace utility {

using api::GeoPosition;
using api::LanePosition;
using api::RoadPosition;
using api::rules::LaneSRange;
using api::rules::LaneSRoute;
using api::rules::SRange;

class DragwayBasedTest : public ::testing::Test {
 protected:
  DragwayBasedTest()
      : dragway_(
            api::RoadGeometryId("my_dragway"), kNumLanes, kLength, kLaneWidth,
            kShoulderWidth, kMaxHeight,
            std::numeric_limits<double>::epsilon() /* linear_tolerance */,
            std::numeric_limits<double>::epsilon() /* angular_tolerance */),
        right_lane_(*dragway_.ToRoadPosition(GeoPosition(0, -kLaneWidth, 0),
                                             nullptr, nullptr, nullptr).lane),
        center_lane_(*dragway_.ToRoadPosition(GeoPosition(0, 0, 0),
                                              nullptr, nullptr, nullptr).lane),
        left_lane_(*dragway_.ToRoadPosition(GeoPosition(0, kLaneWidth, 0),
                                            nullptr, nullptr, nullptr).lane) {
  }

  void SetUp() override {
    ASSERT_TRUE(right_lane_.to_left());
    ASSERT_TRUE(center_lane_.to_left());
    ASSERT_TRUE(center_lane_.to_right());
    ASSERT_TRUE(left_lane_.to_right());
  }

  // The number of lanes was intentionally chosen to evaluate all combinations
  // of adjacent lanes, i.e., it includes a lane with just a lane to the left, a
  // lane with just a lane to the right, and a lane with lanes to both the left
  // and right. The rest of the parameters were arbitrarily chosen.
  const int kNumLanes{3};
  const double kLength{100};
  const double kLaneWidth{6};
  const double kShoulderWidth{1};
  const double kMaxHeight{5};
  const dragway::RoadGeometry dragway_;
  const api::Lane& right_lane_;
  const api::Lane& center_lane_;
  const api::Lane& left_lane_;
};

class DragwayBasedSequenceTest : public DragwayBasedTest {
 protected:
  void CheckSequences(
      const std::vector<std::vector<const api::Lane*>>& sequences,
      std::vector<api::LaneId> expected_ids) {
    ASSERT_EQ(sequences.size(), 1);
    const std::vector<const api::Lane*> sequence = sequences.at(0);
    ASSERT_EQ(sequence.size(), expected_ids.size());
    for (int i = 0; i < static_cast<int>(expected_ids.size()); ++i) {
      ASSERT_EQ(sequence.at(i)->id(), expected_ids.at(i));
    }
  }
};

TEST_F(DragwayBasedSequenceTest, CenterToLeft) {
  CheckSequences(FindLaneSequences(center_lane_, left_lane_),
                 {center_lane_.id(), left_lane_.id()});
}

TEST_F(DragwayBasedSequenceTest, CenterToRight) {
  CheckSequences(FindLaneSequences(center_lane_, right_lane_),
                 {center_lane_.id(), right_lane_.id()});
}

TEST_F(DragwayBasedSequenceTest, RightToLeft) {
  CheckSequences(FindLaneSequences(right_lane_, left_lane_),
                 {right_lane_.id(), center_lane_.id(), left_lane_.id()});
}

TEST_F(DragwayBasedSequenceTest, LeftToRight) {
  CheckSequences(FindLaneSequences(left_lane_, right_lane_),
                 {left_lane_.id(), center_lane_.id(), right_lane_.id()});
}

class MultilaneBasedTest : public ::testing::Test {
 protected:
  MultilaneBasedTest()
      : road_geometry_(multilane::LoadFile(multilane::BuilderFactory(),
          "automotive/maliput/multilane/branch_and_merge.yaml")) {
  }

  void SetUp() override {
    for (int i = 0; i < road_geometry_->num_junctions(); ++i) {
      const api::Junction* junction = road_geometry_->junction(i);
      for (int j = 0; j < junction->num_segments(); ++j) {
        const api::Segment* segment = junction->segment(j);
        for (int k = 0; k < segment->num_lanes(); ++k) {
          const api::Lane* lane = segment->lane(k);
          lanes_[lane->id().string()] = lane;
        }
      }
    }
    ASSERT_EQ(lanes_.size(), 14);
  }

  std::unique_ptr<const api::RoadGeometry> road_geometry_;
  std::map<std::string, const api::Lane*> lanes_;
};

class MultilaneBasedSequenceTest : public MultilaneBasedTest {
 protected:
  void CheckSequences(
      const std::vector<std::vector<const api::Lane*>>& sequences,
      std::vector<std::vector<std::string>> expected_ids) {
    ASSERT_TRUE(sequences.size() == expected_ids.size());
    for (const auto& sequence : sequences) {
      bool found = false;
      for (int j = 0; !found && j < static_cast<int>(expected_ids.size());
          ++j) {
        const std::vector<std::string>& expected_seq = expected_ids.at(j);
        bool match = true;
        if (sequence.size() == expected_seq.size()) {
          for (int i = 0; match && i < static_cast<int>(sequence.size()); ++i) {
            if (sequence.at(i)->id().string() != expected_seq.at(i)) {
              match = false;
            }
          }
        } else {
          match = false;
        }
        if (match) {
          found = true;
        }
      }
      ASSERT_TRUE(found);
    }
  }
};

TEST_F(MultilaneBasedSequenceTest, StartLeftLaneToEndLeftLane) {
  const api::Lane& start_lane = *lanes_["l:0_1"];
  const api::Lane& end_lane = *lanes_["l:3_1"];
  CheckSequences(FindLaneSequences(start_lane, end_lane),
                 {{"l:0_1", "l:1.1_0", "l:1.2_0", "l:1.3_0", "l:1.4_0",
                   "l:1.5_0", "l:3_1"},
                  {"l:0_1", "l:0_0", "l:2.1_0", "l:2.2_0", "l:2.3_0", "l:2.4_0",
                   "l:2.5_0", "l:3_0", "l:3_1"}});
}

TEST_F(MultilaneBasedSequenceTest, StartRightLaneToEndRightLane) {
  const api::Lane& start_lane = *lanes_["l:0_0"];
  const api::Lane& end_lane = *lanes_["l:3_0"];
  CheckSequences(FindLaneSequences(start_lane, end_lane),
                 {{"l:0_0", "l:2.1_0", "l:2.2_0", "l:2.3_0", "l:2.4_0",
                   "l:2.5_0", "l:3_0"},
                  {"l:0_0", "l:0_1", "l:1.1_0", "l:1.2_0", "l:1.3_0", "l:1.4_0",
                   "l:1.5_0", "l:3_1", "l:3_0"}});
}

TEST_F(MultilaneBasedSequenceTest, StartLeftLaneToEndRightLane) {
  const api::Lane& start_lane = *lanes_["l:0_1"];
  const api::Lane& end_lane = *lanes_["l:3_0"];
  CheckSequences(FindLaneSequences(start_lane, end_lane),
                 {{"l:0_1", "l:1.1_0", "l:1.2_0", "l:1.3_0", "l:1.4_0",
                   "l:1.5_0", "l:3_1", "l:3_0"},
                  {"l:0_1", "l:0_0", "l:2.1_0", "l:2.2_0", "l:2.3_0", "l:2.4_0",
                   "l:2.5_0", "l:3_0"}});
}

TEST_F(MultilaneBasedSequenceTest, StartRightLaneToEndLeftLane) {
  const api::Lane& start_lane = *lanes_["l:0_0"];
  const api::Lane& end_lane = *lanes_["l:3_1"];
  CheckSequences(FindLaneSequences(start_lane, end_lane),
                 {{"l:0_0", "l:2.1_0", "l:2.2_0", "l:2.3_0", "l:2.4_0",
                   "l:2.5_0", "l:3_0", "l:3_1"},
                  {"l:0_0", "l:0_1", "l:1.1_0", "l:1.2_0", "l:1.3_0", "l:1.4_0",
                   "l:1.5_0", "l:3_1"}});
}

TEST_F(MultilaneBasedSequenceTest, StartRightLaneToMiddleLeftLane) {
  const api::Lane& start_lane = *lanes_["l:0_0"];
  const api::Lane& end_lane = *lanes_["l:1.3_0"];
  CheckSequences(FindLaneSequences(start_lane, end_lane),
                 {{"l:0_0", "l:0_1", "l:1.1_0", "l:1.2_0", "l:1.3_0"},
                  {"l:0_0", "l:2.1_0", "l:2.2_0", "l:2.3_0", "l:2.4_0",
                   "l:2.5_0", "l:3_0", "l:3_1", "l:1.5_0", "l:1.4_0", "l:1.3_0"}
                 });
}

TEST_F(MultilaneBasedSequenceTest, StartRightLaneToMiddleRightLane) {
  const api::Lane& start_lane = *lanes_["l:0_0"];
  const api::Lane& end_lane = *lanes_["l:2.3_0"];
  CheckSequences(FindLaneSequences(start_lane, end_lane),
                 {{"l:0_0", "l:2.1_0", "l:2.2_0", "l:2.3_0"},
                  {"l:0_0", "l:0_1", "l:1.1_0", "l:1.2_0", "l:1.3_0", "l:1.4_0",
                   "l:1.5_0", "l:3_1", "l:3_0", "l:2.5_0", "l:2.4_0", "l:2.3_0"}
                 });
}

TEST_F(MultilaneBasedSequenceTest, StartLeftLaneToMiddleLeftLane) {
  const api::Lane& start_lane = *lanes_["l:0_1"];
  const api::Lane& end_lane = *lanes_["l:1.3_0"];
  CheckSequences(FindLaneSequences(start_lane, end_lane),
                 {{"l:0_1", "l:1.1_0", "l:1.2_0", "l:1.3_0"},
                  {"l:0_1", "l:0_0", "l:2.1_0", "l:2.2_0", "l:2.3_0", "l:2.4_0",
                   "l:2.5_0", "l:3_0", "l:3_1", "l:1.5_0", "l:1.4_0", "l:1.3_0"}
                 });
}

TEST_F(MultilaneBasedSequenceTest, StartLeftLaneToMiddleRightLane) {
  const api::Lane& start_lane = *lanes_["l:0_1"];
  const api::Lane& end_lane = *lanes_["l:2.3_0"];
  CheckSequences(FindLaneSequences(start_lane, end_lane),
                 {{"l:0_1", "l:0_0", "l:2.1_0", "l:2.2_0", "l:2.3_0"},
                  {"l:0_1", "l:1.1_0", "l:1.2_0", "l:1.3_0", "l:1.4_0",
                   "l:1.5_0", "l:3_1", "l:3_0", "l:2.5_0", "l:2.4_0", "l:2.3_0"}
                 });
}

TEST_F(MultilaneBasedSequenceTest, StartAndStopInSameLane) {
  const std::string lane_id = "l:0_1";
  const api::Lane& lane = *lanes_[lane_id];
  CheckSequences(FindLaneSequences(lane, lane), {{lane_id}});
}

GTEST_TEST(DeriveLaneSRouteTest, NoRouteToEndLane) {
  std::unique_ptr<const api::RoadGeometry> road = multilane::LoadFile(
        multilane::BuilderFactory(),
        "automotive/maliput/multilane/dual_non_intersecting_lanes.yaml");
  const api::Lane& start_lane = *road->junction(0)->segment(0)->lane(0);
  const api::Lane& end_lane = *road->junction(1)->segment(0)->lane(0);
  ASSERT_EQ(FindLaneSequences(start_lane, end_lane).size(), 0);
}

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

TEST_F(DragwayBasedRouteTest, SingleLaneTests) {
  struct TestCase {
    double s_start;
    double s_end;
  };
  const std::vector<const api::Lane*> lanes =
      {&left_lane_, &center_lane_, &right_lane_};
  const std::vector<TestCase> test_cases = {{0, 0},
                                            {0, kLength},
                                            {kLength, kLength},
                                            {0.25 * kLength, 0.75 * kLength}};
  for (const auto& lane : lanes) {
    for (const auto& test_case : test_cases) {
      const RoadPosition start(lane, LanePosition(test_case.s_start, 0, 0));
      const RoadPosition end(lane, LanePosition(test_case.s_end, 0, 0));
      CheckRoutes(DeriveLaneSRoutes(start, end),
                  LaneSRange(lane->id(), SRange(test_case.s_start,
                                                test_case.s_end)));
    }
  }
}

TEST_F(DragwayBasedRouteTest, CrossLaneTests) {
  struct TestCase {
    const api::Lane* start_lane;
    const api::Lane* end_lane;
  };
  const std::vector<TestCase> test_cases = {{&left_lane_, &center_lane_},
                                            {&left_lane_, &right_lane_},
                                            {&center_lane_, &left_lane_},
                                            {&center_lane_, &right_lane_},
                                            {&right_lane_, &center_lane_},
                                            {&right_lane_, &left_lane_}};
  for (const auto& test_case : test_cases) {
    const RoadPosition start(test_case.start_lane, LanePosition(0, 0, 0));
    const RoadPosition end(test_case.end_lane, LanePosition(kLength, 0, 0));
    ASSERT_EQ(DeriveLaneSRoutes(start, end).size(), 0);
  }
}

class MultilaneBasedRouteTest : public MultilaneBasedTest {
 protected:
  // Holds the information needed to check a LaneSRange.
  struct RouteElement {
    std::string id;
    double s0;
    double s1;
  };

  void CheckRoutes(const std::vector<LaneSRoute>& routes,
                   const std::vector<RouteElement>& expected_route) {
    ASSERT_EQ(routes.size(), 1);
    const std::vector<LaneSRange>& ranges = routes.at(0).ranges();
    ASSERT_EQ(ranges.size(), expected_route.size());
    for (int i = 0; i < static_cast<int>(ranges.size()); ++i) {
      const LaneSRange& range = ranges.at(i);
      const RouteElement& expected = expected_route.at(i);
      ASSERT_EQ(range.lane_id().string(), expected.id);
      const SRange s_range = range.s_range();
      ASSERT_EQ(s_range.s0(), expected.s0);
      ASSERT_EQ(s_range.s1(), expected.s1);
    }
  }
};

TEST_F(MultilaneBasedRouteTest, StartLeftLaneToEndLeftLane) {
  const api::Lane& start_lane = *lanes_["l:0_1"];
  const api::Lane& end_lane = *lanes_["l:3_1"];
  const double start_s = 0;
  const double end_s = end_lane.length();
  const RoadPosition start(&start_lane, LanePosition(start_s, 0, 0));
  const RoadPosition end(&end_lane, LanePosition(end_s, 0, 0));
  CheckRoutes(DeriveLaneSRoutes(start, end),
              {{"l:0_1", start_s, start_lane.length()},
               {"l:1.1_0", 0, lanes_["l:1.1_0"]->length()},
               {"l:1.2_0", 0, lanes_["l:1.2_0"]->length()},
               {"l:1.3_0", 0, lanes_["l:1.3_0"]->length()},
               {"l:1.4_0", 0, lanes_["l:1.4_0"]->length()},
               {"l:1.5_0", 0, lanes_["l:1.5_0"]->length()},
               {"l:3_1", 0, end_s}});
}

TEST_F(MultilaneBasedRouteTest, EndLeftLaneToStartLeftLane) {
  const api::Lane& start_lane = *lanes_["l:3_1"];
  const api::Lane& end_lane = *lanes_["l:0_1"];
  const double start_s = start_lane.length();
  const double end_s = 0;
  const RoadPosition start(&start_lane, LanePosition(start_s, 0, 0));
  const RoadPosition end(&end_lane, LanePosition(end_s, 0, 0));
  CheckRoutes(DeriveLaneSRoutes(start, end),
              {{"l:3_1", start_s, 0},
               {"l:1.5_0", lanes_["l:1.5_0"]->length(), 0},
               {"l:1.4_0", lanes_["l:1.4_0"]->length(), 0},
               {"l:1.3_0", lanes_["l:1.3_0"]->length(), 0},
               {"l:1.2_0", lanes_["l:1.2_0"]->length(), 0},
               {"l:1.1_0", lanes_["l:1.1_0"]->length(), 0},
               {"l:0_1", end_lane.length(), end_s}});
}

TEST_F(MultilaneBasedRouteTest, StartRightLaneToEndRightLane) {
  const api::Lane& start_lane = *lanes_["l:0_0"];
  const api::Lane& end_lane = *lanes_["l:3_0"];
  const double start_s = 0;
  const double end_s = end_lane.length();
  const RoadPosition start(&start_lane, LanePosition(start_s, 0, 0));
  const RoadPosition end(&end_lane, LanePosition(end_s, 0, 0));
  CheckRoutes(DeriveLaneSRoutes(start, end),
              {{"l:0_0", start_s, start_lane.length()},
               {"l:2.1_0", 0, lanes_["l:2.1_0"]->length()},
               {"l:2.2_0", 0, lanes_["l:2.2_0"]->length()},
               {"l:2.3_0", 0, lanes_["l:2.3_0"]->length()},
               {"l:2.4_0", 0, lanes_["l:2.4_0"]->length()},
               {"l:2.5_0", 0, lanes_["l:2.5_0"]->length()},
               {"l:3_0", 0, end_s}});
}

TEST_F(MultilaneBasedRouteTest, EndRightLaneToStartRightLane) {
  const api::Lane& start_lane = *lanes_["l:3_0"];
  const api::Lane& end_lane = *lanes_["l:0_0"];
  const double start_s = start_lane.length();
  const double end_s = 0;
  const RoadPosition start(&start_lane, LanePosition(start_s, 0, 0));
  const RoadPosition end(&end_lane, LanePosition(end_s, 0, 0));
  CheckRoutes(DeriveLaneSRoutes(start, end),
              {{"l:3_0", start_s, 0},
               {"l:2.5_0", lanes_["l:2.5_0"]->length(), 0},
               {"l:2.4_0", lanes_["l:2.4_0"]->length(), 0},
               {"l:2.3_0", lanes_["l:2.3_0"]->length(), 0},
               {"l:2.2_0", lanes_["l:2.2_0"]->length(), 0},
               {"l:2.1_0", lanes_["l:2.1_0"]->length(), 0},
               {"l:0_0", end_lane.length(), end_s}});
}

// Tests what happens when the starting and ending road positions result in no
// continuous S curve. This happens when the path must jump to a lane to the
// left or right.
TEST_F(MultilaneBasedRouteTest, NoContinuousSPath) {
  const api::Lane& start_lane = *lanes_["l:0_0"];
  const api::Lane& end_lane = *lanes_["l:3_1"];
  const double start_s = 0;
  const double end_s = end_lane.length();
  const RoadPosition start(&start_lane, LanePosition(start_s, 0, 0));
  const RoadPosition end(&end_lane, LanePosition(end_s, 0, 0));
  ASSERT_EQ(DeriveLaneSRoutes(start, end).size(), 0);
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
