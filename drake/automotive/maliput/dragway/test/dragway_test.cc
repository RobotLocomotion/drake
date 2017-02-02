#include "drake/automotive/maliput/dragway/branch_point.h"
#include "drake/automotive/maliput/dragway/junction.h"
#include "drake/automotive/maliput/dragway/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"

#include <cmath>

#include "gtest/gtest.h"

namespace drake {
namespace maliput {
namespace dragway {
namespace {

class MaliputDragwayLaneTest : public ::testing::Test {
 public:
  MaliputDragwayLaneTest()
      : kLength_(1365.34),
        kLaneWidth_(5.0058),
        kShoulderWidth_(4.2319) {
  }

  // Verifies the correctness of the provided `lane`.
  void VerifyLaneCorrectness(const api::Lane* lane, int num_lanes) {
    const double road_width = num_lanes * kLaneWidth_ + 2 * kShoulderWidth_;
    const double y_min = -road_width / 2;
    const double y_max = road_width / 2;

    const int lane_index = lane->index();
    const double expected_y_offset =
        y_min + kShoulderWidth_ + lane_index * kLaneWidth_ + kLaneWidth_ / 2;

    EXPECT_EQ(lane->id().id, "Dragway_Lane_" + std::to_string(lane_index));
    EXPECT_EQ(lane->length(), kLength_);

    // Tests Lane::lane_bounds().
    for (double s = 0 ; s < kLength_; s += kLength_ / 100) {
      const api::RBounds lane_bounds = lane->lane_bounds(s);
      EXPECT_EQ(lane_bounds.r_min, -kLaneWidth_ / 2);
      EXPECT_EQ(lane_bounds.r_max, kLaneWidth_ / 2);
    }

    // Tests Lane::driveable_bounds().
    const double driveable_r_min = y_min - expected_y_offset;
    const double driveable_r_max = y_max - expected_y_offset;

    for (double s = 0 ; s < kLength_; s += kLength_ / 100) {
      const api::RBounds driveable_bounds = lane->driveable_bounds(s);
      EXPECT_EQ(driveable_bounds.r_min, driveable_r_min);
      EXPECT_EQ(driveable_bounds.r_max, driveable_r_max);
    }

    // Tests Lane::ToGeoPosition().
    const double driveable_width = driveable_r_max - driveable_r_min;
    for (double s = 0; s < kLength_; s += kLength_ / 100) {
      for (double r = driveable_r_min; r < driveable_r_max;
          r += driveable_width / 100) {
        for (double h = 0; h < 1; h += 0.1) {
          const api::LanePosition lane_position(s, r, h);
          const api::GeoPosition geo_position =
              lane->ToGeoPosition(lane_position);
          const double linear_tolerance =
              lane->segment()->junction()->road_geometry()->linear_tolerance();
          EXPECT_DOUBLE_EQ(geo_position.x, s);
          EXPECT_NEAR(geo_position.y, expected_y_offset + r, linear_tolerance);
          EXPECT_DOUBLE_EQ(geo_position.z, h);
        }
      }
    }

    // Tests Lane::GetOrientation().
    for (double s = 0; s < kLength_; s += kLength_ / 100) {
      for (double r = driveable_r_min; r < driveable_r_max;
          r += driveable_width / 100) {
        for (double h = 0; h < 1; h += 0.1) {
          const api::LanePosition lane_position(s, r, h);
          const api::Rotation rotation =
              lane->GetOrientation(lane_position);
          EXPECT_DOUBLE_EQ(rotation.roll, 0);
          EXPECT_DOUBLE_EQ(rotation.pitch, 0);
          EXPECT_DOUBLE_EQ(rotation.yaw, 0);
        }
      }
    }

    // Tests Lane::EvalMotionDerivatives().
    for (double s = 0; s < kLength_; s += kLength_ / 100) {
      for (double r = driveable_r_min; r < driveable_r_max;
          r += driveable_width / 100) {
        for (double h = 0; h < 1; h += 0.2) {
          const api::LanePosition lane_position(s, r, h);

          // The following translational velocities can be any value. We just
          // want to verify that the same values are returned from
          // Lane::EvalMotionDerivatives().
          const double kSigma_v = 1.1;
          const double kRho_v = 2.2;
          const double kEta_v = 3.3;

          const api::LanePosition motion_derivatives =
              lane->EvalMotionDerivatives(lane_position,
                  api::IsoLaneVelocity(kSigma_v, kRho_v, kEta_v));
          EXPECT_DOUBLE_EQ(motion_derivatives.s, kSigma_v);
          EXPECT_DOUBLE_EQ(motion_derivatives.r, kRho_v);
          EXPECT_DOUBLE_EQ(motion_derivatives.h, kEta_v);
        }
      }
    }
  }

  // Verifies that the branches within the provided `lane` are correct. The
  // provided `lane` must be in the provided `road_geometry`.
  void VerifyBranches(const api::Lane* lane,
      const RoadGeometry* road_geometry) const {
    // Verifies that the same BranchPoint covers both ends of the dragway lane.
    EXPECT_EQ(lane->GetBranchPoint(api::LaneEnd::kStart),
              lane->GetBranchPoint(api::LaneEnd::kFinish));

    const api::BranchPoint* branch_point =
        lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_NE(branch_point, nullptr);
    EXPECT_EQ(branch_point->id().id, lane->id().id + "_Branch_Point");
    EXPECT_EQ(branch_point->road_geometry(), road_geometry);

    // Verifies correctness of the confluent branches.
    {
      const api::LaneEndSet* lane_end_set_start =
          lane->GetConfluentBranches(api::LaneEnd::kStart);
      const api::LaneEndSet* lane_end_set_finish =
          lane->GetConfluentBranches(api::LaneEnd::kFinish);
      EXPECT_EQ(lane_end_set_start->size(), 1);
      EXPECT_EQ(lane_end_set_finish->size(), 1);

      const api::LaneEnd& lane_end_start =
        lane->GetConfluentBranches(api::LaneEnd::kStart)->get(0);
      EXPECT_EQ(lane_end_start.lane, lane);
      EXPECT_EQ(lane_end_start.end, api::LaneEnd::kStart);
      const api::LaneEnd& lane_end_finish =
        lane->GetConfluentBranches(api::LaneEnd::kFinish)->get(0);
      EXPECT_EQ(lane_end_finish.lane, lane);
      EXPECT_EQ(lane_end_finish.end, api::LaneEnd::kFinish);
    }

    // Verifies correctness of the ongoing branches.
    {
      const api::LaneEndSet* lane_end_set_start =
          lane->GetOngoingBranches(api::LaneEnd::kStart);
      const api::LaneEndSet* lane_end_set_finish =
          lane->GetOngoingBranches(api::LaneEnd::kFinish);
      EXPECT_EQ(lane_end_set_start->size(), 1);
      EXPECT_EQ(lane_end_set_finish->size(), 1);

      const api::LaneEnd& lane_end_start =
        lane->GetOngoingBranches(api::LaneEnd::kStart)->get(0);
      EXPECT_EQ(lane_end_start.lane, lane);
      EXPECT_EQ(lane_end_start.end, api::LaneEnd::kFinish);
      const api::LaneEnd& lane_end_finish =
        lane->GetOngoingBranches(api::LaneEnd::kFinish)->get(0);
      EXPECT_EQ(lane_end_finish.lane, lane);
      EXPECT_EQ(lane_end_finish.end, api::LaneEnd::kStart);
    }

    // Verifies correctness of the default branches.
    {
      std::unique_ptr<api::LaneEnd> default_start_lane_end =
          lane->GetDefaultBranch(api::LaneEnd::kStart);
      EXPECT_NE(default_start_lane_end.get(), nullptr);
      EXPECT_EQ(default_start_lane_end->end, api::LaneEnd::kFinish);
      EXPECT_EQ(default_start_lane_end->lane, lane);

      std::unique_ptr<api::LaneEnd> default_finish_lane_end =
          lane->GetDefaultBranch(api::LaneEnd::kFinish);
      EXPECT_NE(default_finish_lane_end.get(), nullptr);
      EXPECT_EQ(default_finish_lane_end->end, api::LaneEnd::kStart);
      EXPECT_EQ(default_finish_lane_end->lane, lane);
    }
  }

  const double kLength_{};
  const double kLaneWidth_{};
  const double kShoulderWidth_{};
};

/*
 Tests a dragway containing one lane. It is arranged as shown below in the world
 frame:

              x
              ^
        r_max | r_min
       ---------------     s = kLength_
       |      ^      |
       |      :      |
       |      ^      |
       |      :      |
  y <---------o---------> s = 0
              |
              V
 */
TEST_F(MaliputDragwayLaneTest, SingleLane) {
  const api::RoadGeometryId road_geometry_id({"OneLaneDragwayRoadGeometry"});
  const int kNumLanes = 1;

  RoadGeometry road_geometry(road_geometry_id, kNumLanes, kLength_, kLaneWidth_,
      kShoulderWidth_);

  const api::Junction* junction = road_geometry.junction(0);
  ASSERT_NE(junction, nullptr);
  EXPECT_EQ(junction->segment(0)->lane(0)->length(), kLength_);

  const api::Segment* segment = junction->segment(0);
  ASSERT_NE(segment, nullptr);
  EXPECT_EQ(segment->num_lanes(), 1);
  EXPECT_EQ(segment->id().id, "Dragway_Segment_ID");

  const int kLaneIndex = 0;
  const api::Lane* lane = segment->lane(kLaneIndex);
  ASSERT_NE(lane, nullptr);
  EXPECT_EQ(lane->segment(), segment);

  VerifyLaneCorrectness(lane, kNumLanes);
  VerifyBranches(lane, &road_geometry);
}


/*
 Tests a dragway containing two lanes. The two lanes are arranged as shown below
 in the world frame:

                              x
                              ^
                  r_max r_min | r_max  r_min
                --------------|--------------  s = kLength
                |      ^      |      ^      |
                |      :      |      :      |
                |      ^      |      ^      |
                |      :      |      :      |  s = 0
      y <--+------------------o------------------------------>
                              |
                              V
 */
TEST_F(MaliputDragwayLaneTest, TwoLaneDragway) {
  const api::RoadGeometryId road_geometry_id({"TwoLaneDragwayRoadGeometry"});
  const int kNumLanes = 2;

  RoadGeometry road_geometry(road_geometry_id, kNumLanes, kLength_,
      kLaneWidth_, kShoulderWidth_);

  const api::Junction* junction = road_geometry.junction(0);
  ASSERT_NE(junction, nullptr);
  EXPECT_EQ(junction->segment(0)->lane(0)->length(), kLength_);

  const api::Segment* segment = junction->segment(0);
  ASSERT_NE(segment, nullptr);
  EXPECT_EQ(segment->num_lanes(), kNumLanes);
  EXPECT_EQ(segment->id().id, "Dragway_Segment_ID");

  for (int i = 0; i < kNumLanes; ++i) {
    const api::Lane* lane = segment->lane(i);
    ASSERT_NE(lane, nullptr);
    VerifyLaneCorrectness(lane, kNumLanes);
    VerifyBranches(lane, &road_geometry);
  }
}

}  // namespace
}  // namespace dragway
}  // namespace maliput
}  // namespace drake
