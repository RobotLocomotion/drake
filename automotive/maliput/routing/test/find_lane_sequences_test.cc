#include "drake/automotive/maliput/routing/find_lane_sequences.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/test_utilities/fixtures.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/loader.h"
#include "drake/automotive/maliput/multilane/test_utilities/fixtures.h"

namespace drake {
namespace maliput {
namespace routing {

using drake::maliput::api::Lane;
using drake::maliput::api::LaneId;
using drake::maliput::api::RoadGeometry;
using drake::maliput::dragway::DragwayBasedTest;
using drake::maliput::multilane::BranchAndMergeBasedTest;
using drake::maliput::multilane::LoopBasedTest;
using drake::maliput::multilane::MultiBranchBasedTest;

namespace {

void CheckSequences(const std::vector<std::vector<const Lane*>>& sequences,
                    const std::vector<std::vector<std::string>>& expected_ids) {
  ASSERT_TRUE(sequences.size() == expected_ids.size());
  for (const auto& sequence : sequences) {
    bool found = false;
    for (int j = 0; !found && j < static_cast<int>(expected_ids.size()); ++j) {
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

}  // namespace

TEST_F(DragwayBasedTest, FindLaneSequencesChangeLanes) {
  CheckSequences(FindLaneSequences(center_lane_, left_lane_, kLength), {});
  CheckSequences(FindLaneSequences(center_lane_, right_lane_, kLength), {});
  CheckSequences(FindLaneSequences(right_lane_, left_lane_, kLength), {});
  CheckSequences(FindLaneSequences(left_lane_, right_lane_, kLength), {});
}

TEST_F(DragwayBasedTest, FindLaneSequencesSameLane) {
  for (double length : std::vector<double>{kLength, 0.}) {
    CheckSequences(FindLaneSequences(center_lane_, center_lane_, length),
                   {{center_lane_->id().string()}});
    CheckSequences(FindLaneSequences(right_lane_, right_lane_, length),
                   {{right_lane_->id().string()}});
    CheckSequences(FindLaneSequences(left_lane_, left_lane_, length),
                   {{left_lane_->id().string()}});
  }
}

TEST_F(BranchAndMergeBasedTest, FindLaneSequencesStartLeftLaneToEndLeftLane) {
  const Lane* start_lane = index_.GetLane(LaneId("l:0_1"));
  const Lane* end_lane = index_.GetLane(LaneId("l:3_1"));
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_),
                 {{"l:0_1", "l:1.1_0", "l:1.2_0", "l:1.3_0", "l:3_1"}});
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_ / 2),
                 {});
}

TEST_F(BranchAndMergeBasedTest, FindLaneSequencesStartRightLaneToEndRightLane) {
  const Lane* start_lane = index_.GetLane(LaneId("l:0_0"));
  const Lane* end_lane = index_.GetLane(LaneId("l:3_0"));
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_),
                 {{"l:0_0", "l:2.1_0", "l:2.2_0", "l:2.3_0", "l:3_0"}});
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_ / 2),
                 {});
}

TEST_F(BranchAndMergeBasedTest, FindLaneSequencesStartLeftLaneToEndRightLane) {
  const Lane* start_lane = index_.GetLane(LaneId("l:0_1"));
  const Lane* end_lane = index_.GetLane(LaneId("l:3_0"));
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_), {});
}

TEST_F(BranchAndMergeBasedTest, FindLaneSequencesStartRightLaneToEndLeftLane) {
  const Lane* start_lane = index_.GetLane(LaneId("l:0_0"));
  const Lane* end_lane = index_.GetLane(LaneId("l:3_1"));
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_), {});
}

TEST_F(BranchAndMergeBasedTest, FindLaneSequencesStartAndStopInSameLane) {
  const std::string lane_id = "l:0_1";
  const Lane* lane = index_.GetLane(LaneId(lane_id));
  for (double length :
       std::vector<double>{total_length_, total_length_ / 2, 0.}) {
    CheckSequences(FindLaneSequences(lane, lane, length), {{lane_id}});
  }
}

TEST_F(LoopBasedTest, FindLaneSequencesTest) {
  // Arbitrary large number to avoid hitting limit.
  constexpr double kMaxLength{1e6};
  // Verifies the DUT doesn't enter an infinite loop.
  const Lane* start_lane = index_.GetLane(LaneId("l:0_0"));
  const Lane* end_lane = index_.GetLane(LaneId("l:4_0"));
  const std::vector<std::vector<const Lane*>> sequences =
      FindLaneSequences(start_lane, end_lane, kMaxLength);
  ASSERT_EQ(sequences.size(), 5);
}

TEST_F(MultiBranchBasedTest, FindLaneSequencesTest) {
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
    CheckSequences(
        FindLaneSequences(test_case.first, test_case.second, kMaxLength),
        {{test_case.first->id().string(), test_case.second->id().string()}});
  }
}

GTEST_TEST(FindLaneSequencesTest, NoRouteToEndLane) {
  std::unique_ptr<const RoadGeometry> road =
      drake::maliput::multilane::LoadFile(
          drake::maliput::multilane::BuilderFactory(),
          "automotive/maliput/multilane/dual_non_intersecting_lanes.yaml");
  const Lane* start_lane = road->junction(0)->segment(0)->lane(0);
  const Lane* end_lane = road->junction(1)->segment(0)->lane(0);
  ASSERT_EQ(FindLaneSequences(start_lane, end_lane,
                              start_lane->length() + end_lane->length())
                .size(),
            0);
}

GTEST_TEST(FindLaneSequencesTest, MaxLengthOmitsStartAndEndLanes) {
  std::unique_ptr<const RoadGeometry> road =
      drake::maliput::multilane::LoadFile(
          drake::maliput::multilane::BuilderFactory(),
          "automotive/maliput/multilane/long_start_and_end_lanes.yaml");
  const RoadGeometry::IdIndex& index = road->ById();
  const Lane* start_lane = index.GetLane(LaneId("l:0_0"));
  const Lane* middle_lane = index.GetLane(LaneId("l:1_0"));
  const Lane* end_lane = index.GetLane(LaneId("l:2_0"));
  ASSERT_EQ(
      FindLaneSequences(start_lane, end_lane, middle_lane->length() / 2).size(),
      0);
  ASSERT_EQ(
      FindLaneSequences(start_lane, end_lane, middle_lane->length()).size(), 1);
  const double total_length =
      start_lane->length() + middle_lane->length() + end_lane->length();
  ASSERT_EQ(FindLaneSequences(start_lane, end_lane, total_length).size(), 1);
}

}  // namespace routing
}  // namespace maliput
}  // namespace drake
