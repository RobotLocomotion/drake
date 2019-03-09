#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/loader.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace {

using api::GeoPosition;
using api::Lane;
using api::LaneId;
using api::LanePosition;
using api::RBounds;
using api::RoadGeometry;

constexpr double kStartDistance{59.375};
constexpr double kEntryDistance{9.375};
constexpr double kMidDistance{kStartDistance / 2.0 + kEntryDistance};
constexpr double kLaneWidth{3.75};
constexpr double kHalfLaneWidth{kLaneWidth / 2.0};
constexpr double kApproachLength{50.0};
constexpr double kIntersectionWidth{kEntryDistance * 2.0};

class Test2x2Intersection : public ::testing::Test {
 protected:
  virtual void SetUp() {
    static const char* const kFilePath =
        "drake/automotive/maliput/multilane/2x2_intersection.yaml";
    const std::string file_path = FindResourceOrThrow(kFilePath);
    dut_ = LoadFile(multilane::BuilderFactory(), file_path);
  }

  std::unique_ptr<const RoadGeometry> dut_;
};

TEST_F(Test2x2Intersection, CheckRoadGeometryProperties) {
  EXPECT_NE(dut_, nullptr);
  EXPECT_EQ(dut_->id().string(), "basic_two_lane_x_intersection");
  EXPECT_EQ(dut_->num_junctions(), 14);
  EXPECT_EQ(dut_->num_branch_points(), 16);
  EXPECT_NEAR(dut_->linear_tolerance(), 0.01, 1e-6);
  EXPECT_NEAR(dut_->angular_tolerance(), 0.5 * M_PI / 180., 1e-6);
  EXPECT_NEAR(dut_->scale_length(), 1, 1e-6);
  EXPECT_EQ(dut_->CheckInvariants().size(), 0);
}

TEST_F(Test2x2Intersection, CheckLaneNamesAndPositions) {
  struct ExpectedLane {
    LaneId id;
    double length;
    // These are positions that should fall within the lane.
    std::vector<GeoPosition> geo_positions;
  };
  const std::vector<ExpectedLane> test_cases{
      // Segment west of intersection.
      // East-bound lane.
      {LaneId("l:w_segment_0"),
       kApproachLength,
       {GeoPosition(-kStartDistance, -kHalfLaneWidth, 0),
        GeoPosition(-kMidDistance, -kHalfLaneWidth, 0),
        GeoPosition(-kEntryDistance, -kLaneWidth, 0)}},
      // West-bound lane.
      {LaneId("l:w_segment_1"),
       kApproachLength,
       {GeoPosition(-kStartDistance, kHalfLaneWidth, 0),
        GeoPosition(-kMidDistance, kHalfLaneWidth, 0),
        GeoPosition(-kEntryDistance, kHalfLaneWidth, 0)}},

      // Segment east of intersection.
      // East-bound lane.
      {LaneId("l:e_segment_0"),
       kApproachLength,
       {GeoPosition(kEntryDistance, -kHalfLaneWidth, 0),
        GeoPosition(kMidDistance, -kHalfLaneWidth, 0),
        GeoPosition(kStartDistance, -kHalfLaneWidth, 0)}},
      // West-bound lane.
      {LaneId("l:e_segment_1"),
       kApproachLength,
       {GeoPosition(kEntryDistance, kHalfLaneWidth, 0),
        GeoPosition(kMidDistance, kHalfLaneWidth, 0),
        GeoPosition(kStartDistance, kHalfLaneWidth, 0)}},

      // Segment north of intersection.
      // North-bound lane.
      {LaneId("l:n_segment_0"),
       kApproachLength,
       {GeoPosition(kHalfLaneWidth, kEntryDistance, 0),
        GeoPosition(kHalfLaneWidth, kMidDistance, 0),
        GeoPosition(kHalfLaneWidth, kStartDistance, 0)}},
      // South-bound lane.
      {LaneId("l:n_segment_1"),
       kApproachLength,
       {GeoPosition(-kHalfLaneWidth, kEntryDistance, 0),
        GeoPosition(-kHalfLaneWidth, kMidDistance, 0),
        GeoPosition(-kHalfLaneWidth, kStartDistance, 0)}},

      // Segment south of intersection.
      // North-bound lane.
      {LaneId("l:s_segment_0"),
       kApproachLength,
       {GeoPosition(kHalfLaneWidth, -kEntryDistance, 0),
        GeoPosition(kHalfLaneWidth, -kMidDistance, 0),
        GeoPosition(kHalfLaneWidth, -kStartDistance, 0)}},
      // South-bound lane.
      {LaneId("l:s_segment_1"),
       kApproachLength,
       {GeoPosition(-kHalfLaneWidth, -kEntryDistance, 0),
        GeoPosition(-kHalfLaneWidth, -kMidDistance, 0),
        GeoPosition(-kHalfLaneWidth, -kStartDistance, 0)}},

      // East/West segment inside the intersection.
      // East-bound lane.
      {LaneId("l:ew_intersection_segment_0"),
       kIntersectionWidth,
       {GeoPosition(0, -kHalfLaneWidth, 0), GeoPosition(8, -kHalfLaneWidth, 0),
        GeoPosition(-2.33, -2.33, 0), GeoPosition(2.33, -2.33, 0)}},
      // West-bound lane.
      {LaneId("l:ew_intersection_segment_1"),
       kIntersectionWidth,
       {GeoPosition(0, kHalfLaneWidth, 0), GeoPosition(8, kHalfLaneWidth, 0),
        GeoPosition(2.33, 2.33, 0), GeoPosition(-2.33, 2.33, 0)}},

      // North/South segment inside the intersection.
      // North-bound lane.
      {LaneId("l:ns_intersection_segment_0"),
       kIntersectionWidth,
       {GeoPosition(kHalfLaneWidth, 0, 0), GeoPosition(kHalfLaneWidth, -8, 0),
        GeoPosition(2.33, -2.33, 0), GeoPosition(2.33, 2.33, 0)}},
      // South-bound lane.
      {LaneId("l:ns_intersection_segment_1"),
       kIntersectionWidth,
       {GeoPosition(0, -kHalfLaneWidth, 0), GeoPosition(-kHalfLaneWidth, 8, 0),
        GeoPosition(-2.33, -2.33, 0), GeoPosition(-2.33, 2.33, 0)}},

      // Right turn lanes.
      {LaneId("l:east_right_turn_segment_0"),
       M_PI_2 * 7.5,
       {GeoPosition(-kEntryDistance, -kHalfLaneWidth, 0),
        GeoPosition(-4.8, -4.8, 0),
        GeoPosition(-kHalfLaneWidth, -kEntryDistance, 0)}},
      {LaneId("l:west_right_turn_segment_0"),
       M_PI_2 * 7.5,
       {GeoPosition(kEntryDistance, kHalfLaneWidth, 0),
        GeoPosition(4.8, 4.8, 0),
        GeoPosition(kHalfLaneWidth, kEntryDistance, 0)}},
      {LaneId("l:north_right_turn_segment_0"),
       M_PI_2 * 7.5,
       {GeoPosition(kHalfLaneWidth, -kEntryDistance, 0),
        GeoPosition(4.8, -4.8, 0),
        GeoPosition(kEntryDistance, -kHalfLaneWidth, 0)}},
      {LaneId("l:south_right_turn_segment_0"),
       M_PI_2 * 7.5,
       {GeoPosition(-kHalfLaneWidth, kEntryDistance, 0),
        GeoPosition(-4.8, 4.8, 0),
        GeoPosition(-kEntryDistance, kHalfLaneWidth, 0)}},

      // Left turn lanes.
      {LaneId("l:east_left_turn_segment_0"),
       M_PI_2 * 11.25,
       {GeoPosition(-kEntryDistance, -kHalfLaneWidth, 0),
        GeoPosition(-1.66, 0.64, 0),
        GeoPosition(kHalfLaneWidth, kEntryDistance, 0)}},
      {LaneId("l:west_left_turn_segment_0"),
       M_PI_2 * 11.25,
       {GeoPosition(kEntryDistance, kHalfLaneWidth, 0),
        GeoPosition(1.66, -0.64, 0),
        GeoPosition(-kHalfLaneWidth, -kEntryDistance, 0)}},
      {LaneId("l:north_left_turn_segment_0"),
       M_PI_2 * 11.25,
       {GeoPosition(kHalfLaneWidth, -kEntryDistance, 0),
        GeoPosition(0.64, -1.66, 0),
        GeoPosition(-kEntryDistance, kHalfLaneWidth, 0)}},
      {LaneId("l:south_left_turn_segment_0"),
       M_PI_2 * 11.25,
       {GeoPosition(-kHalfLaneWidth, kEntryDistance, 0),
        GeoPosition(-0.64, 1.66, 0),
        GeoPosition(kEntryDistance, -kHalfLaneWidth, 0)}},
  };
  double distance{};
  GeoPosition closest_point;
  for (const auto& test_case : test_cases) {
    const Lane* lane = dut_->ById().GetLane(test_case.id);
    EXPECT_NE(lane, nullptr);
    EXPECT_NEAR(lane->length(), test_case.length, dut_->linear_tolerance());
    for (const auto& geo_position : test_case.geo_positions) {
      const LanePosition lane_position =
          lane->ToLanePosition(geo_position, &closest_point, &distance);
      EXPECT_NEAR(distance, 0, dut_->linear_tolerance());
      const RBounds lane_bounds = lane->lane_bounds(lane_position.s());
      EXPECT_TRUE(lane_bounds.min() <= lane_position.r());
      EXPECT_TRUE(lane_bounds.max() >= lane_position.r());
    }
  }
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
