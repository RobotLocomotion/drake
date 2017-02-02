#include "drake/automotive/maliput/dragway/branch_point.h"
#include "drake/automotive/maliput/dragway/junction.h"
#include "drake/automotive/maliput/dragway/lane.h"
#include "drake/automotive/maliput/dragway/northbound_lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/dragway/southbound_lane.h"

#include <cmath>

#include "gtest/gtest.h"

namespace drake {
namespace maliput {
namespace dragway {

class MaliputDragwayLaneTest : public ::testing::Test {
 public:
  MaliputDragwayLaneTest()
      : kLength_(1365.34),
        kLaneBounds_({-2.5029, 2.5029}),
        kDriveableBounds_({-4.6692, 4.6692}),
        kDriveableWidth_(kDriveableBounds_.r_max - kDriveableBounds_.r_min) {
  }

  // Verifies the correctness of the provided `northbound_lane`.
  void VerifyNorthboundLaneCorrectness(const NorthboundLane* lane,
      int lane_index) {
    // TODO(liang.fok) Support non-zero lane indices when the need arises.
    // Currently, this test only supports RoadGeometries containing at most
    // one northbound land and one southbound lane.
    if (lane_index != 0) {
      throw std::runtime_error(
          "MaliputDragwayLaneTest::VerifyNorthboundLaneCorrectness: "
          "Non-zero lane indices currently not supported.");
    }
    EXPECT_EQ(lane->y_offset(), -kDriveableBounds_.r_max);
    EXPECT_EQ(lane->id().id, "Dragway_Northbound_Lane_" +
        std::to_string(lane_index));
    EXPECT_EQ(lane->length(), kLength_);

    // Tests NorthboundLane::lane_bounds().
    for (double s = 0 ; s < kLength_; s += kLength_ / 100) {
      const api::RBounds lane_bounds = lane->lane_bounds(s);
      EXPECT_EQ(lane_bounds.r_min, kLaneBounds_.r_min);
      EXPECT_EQ(lane_bounds.r_max, kLaneBounds_.r_max);
    }

    // Tests NorthboundLane::driveable_bounds().
    for (double s = 0 ; s < kLength_; s += kLength_ / 100) {
      const api::RBounds driveable_bounds = lane->driveable_bounds(s);
      EXPECT_EQ(driveable_bounds.r_min, kDriveableBounds_.r_min);
      EXPECT_EQ(driveable_bounds.r_max, kDriveableBounds_.r_max);
    }

    // Tests NorthboundLane::ToGeoPosition().
    for (double s = 0; s < kLength_; s += kLength_ / 100) {
      for (double r = kDriveableBounds_.r_min; r < kDriveableBounds_.r_max;
          r += kDriveableWidth_ / 100) {
        for (double h = 0; h < 1; h += 0.1) {
          const api::LanePosition lane_position(s, r, h);
          const api::GeoPosition geo_position =
              lane->ToGeoPosition(lane_position);
          EXPECT_DOUBLE_EQ(geo_position.x, s);
          EXPECT_DOUBLE_EQ(geo_position.y, -kDriveableBounds_.r_max + r);
          EXPECT_DOUBLE_EQ(geo_position.z, h);
        }
      }
    }

    // Tests NorthboundLane::GetOrientation().
    for (double s = 0; s < kLength_; s += kLength_ / 100) {
      for (double r = kDriveableBounds_.r_min; r < kDriveableBounds_.r_max;
          r += kDriveableWidth_ / 100) {
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

    // Tests NorthboundLane::EvalMotionDerivatives().
    for (double s = 0; s < kLength_; s += kLength_ / 100) {
      for (double r = kDriveableBounds_.r_min; r < kDriveableBounds_.r_max;
          r += kDriveableWidth_ / 100) {
        for (double h = 0; h < 1; h += 0.2) {
          const api::LanePosition lane_position(s, r, h);
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

  void VerifySouthboundLaneCorrectness(const SouthboundLane* lane,
      int lane_index) {
    // TODO(liang.fok) Support non-zero lane indices when the need arises.
    // Currently, this test only supports RoadGeometries containing at most
    // one northbound land and one southbound lane.
    if (lane_index != 0) {
      throw std::runtime_error(
          "MaliputDragwayLaneTest::VerifySouthboundLaneCorrectness: "
          "Non-zero lane indices currently not supported.");
    }
    EXPECT_EQ(lane->y_offset(), kDriveableBounds_.r_max);
    EXPECT_EQ(lane->id().id, "Dragway_Southbound_Lane_" +
        std::to_string(lane_index));
    EXPECT_EQ(lane->length(), kLength_);

    // Tests SouthboundLane::lane_bounds().
    for (double s = 0 ; s < kLength_; s += kLength_ / 100) {
      const api::RBounds lane_bounds = lane->lane_bounds(s);
      EXPECT_EQ(lane_bounds.r_min, kLaneBounds_.r_min);
      EXPECT_EQ(lane_bounds.r_max, kLaneBounds_.r_max);
    }

    // Tests SouthboundLane::driveable_bounds().
    for (double s = 0 ; s < kLength_; s += kLength_ / 100) {
      const api::RBounds driveable_bounds = lane->driveable_bounds(s);
      EXPECT_EQ(driveable_bounds.r_min, kDriveableBounds_.r_min);
      EXPECT_EQ(driveable_bounds.r_max, kDriveableBounds_.r_max);
    }

    // Tests SouthboundLane::ToGeoPosition().
    for (double s = 0; s < kLength_; s += kLength_ / 100) {
      for (double r = kDriveableBounds_.r_min; r < kDriveableBounds_.r_max;
          r += kDriveableWidth_ / 100) {
        for (double h = 0; h < 1; h += 0.2) {
          const api::LanePosition lane_position(s, r, h);
          const api::GeoPosition geo_position =
              lane->ToGeoPosition(lane_position);
          EXPECT_DOUBLE_EQ(geo_position.x, kLength_ - s);
          EXPECT_DOUBLE_EQ(geo_position.y, kDriveableBounds_.r_max - r);
          EXPECT_DOUBLE_EQ(geo_position.z, h);
        }
      }
    }

    // Tests SouthboundLane::GetOrientation().
    for (double s = 0; s < kLength_; s += kLength_ / 100) {
      for (double r = kDriveableBounds_.r_min; r < kDriveableBounds_.r_max;
          r += kDriveableWidth_ / 100) {
        for (double h = 0; h < 1; h += 0.1) {
          const api::LanePosition lane_position(s, r, h);
          const api::Rotation rotation =
              lane->GetOrientation(lane_position);
          EXPECT_DOUBLE_EQ(rotation.roll, 0);
          EXPECT_DOUBLE_EQ(rotation.pitch, 0);
          EXPECT_DOUBLE_EQ(rotation.yaw, M_PI);
        }
      }
    }

    // Tests SouthboundLane::EvalMotionDerivatives().
    for (double s = 0; s < kLength_; s += kLength_ / 100) {
      for (double r = kDriveableBounds_.r_min; r < kDriveableBounds_.r_max;
          r += kDriveableWidth_ / 100) {
        for (double h = 0; h < 1; h += 0.1) {
          const api::LanePosition lane_position(s, r, h);
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
  void VerifyBranches(const Lane* lane,
      const RoadGeometry* road_geometry) const {
    // Verifies that the same BranchPoint covers both ends of the dragway lane.
    EXPECT_EQ(lane->GetBranchPoint(api::LaneEnd::kStart),
              lane->GetBranchPoint(api::LaneEnd::kFinish));

    const api::BranchPoint* api_branch_point =
        lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_NE(api_branch_point, nullptr);
    const BranchPoint* branch_point =
        dynamic_cast<const BranchPoint*>(api_branch_point);
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
      const api::LaneEnd& lane_end_finish =
        lane->GetConfluentBranches(api::LaneEnd::kStart)->get(0);
      EXPECT_EQ(lane_end_finish.lane, lane);
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
      const api::LaneEnd& lane_end_finish =
        lane->GetOngoingBranches(api::LaneEnd::kStart)->get(0);
      EXPECT_EQ(lane_end_finish.lane, lane);
    }

    // Verifies correctness of the default branches.
    {
      std::unique_ptr<api::LaneEnd> default_start_lane_end =
          lane->GetDefaultBranch(api::LaneEnd::kStart);
      EXPECT_NE(default_start_lane_end.get(), nullptr);
      EXPECT_EQ(default_start_lane_end->end, api::LaneEnd::kStart);
      EXPECT_EQ(default_start_lane_end->lane, lane);

      std::unique_ptr<api::LaneEnd> default_finish_lane_end =
          lane->GetDefaultBranch(api::LaneEnd::kFinish);
      EXPECT_NE(default_finish_lane_end.get(), nullptr);
      EXPECT_EQ(default_finish_lane_end->end, api::LaneEnd::kFinish);
      EXPECT_EQ(default_finish_lane_end->lane, lane);
    }
  }

  const double kLength_{};
  const api::RBounds kLaneBounds_;
  const api::RBounds kDriveableBounds_;
  const double kDriveableWidth_{};
};

/*
 Tests a dragway RoadGeometry containing one northbound lane. It is arranged as
 shown below in the world frame:

       x
       ^
       | r_max  r_min
       |--------------     s = kLength
       |      ^      |
       |      :      |
       |      ^      |
       |      :      |
  y <--o----------------> s = 0
       |
       V
 */
TEST_F(MaliputDragwayLaneTest, SingleNorthboundLane) {
  const api::RoadGeometryId road_geometry_id(
      {"SingleNorthboundLaneRoadGeometry"});
  const int kNumSouthboundLanes = 0;
  const int kNumNorthboundLanes = 1;

  RoadGeometry road_geometry(road_geometry_id, kNumSouthboundLanes,
      kNumNorthboundLanes, kLength_, kLaneBounds_, kDriveableBounds_);

  const Junction* junction = road_geometry.junction();
  ASSERT_NE(junction, nullptr);
  EXPECT_EQ(junction->length(), kLength_);

  const Segment* segment = junction->segment();
  ASSERT_NE(segment, nullptr);
  EXPECT_EQ(segment->num_lanes(), 1);
  EXPECT_EQ(segment->num_southbound_lanes(), kNumSouthboundLanes);
  EXPECT_EQ(segment->num_northbound_lanes(), kNumNorthboundLanes);
  EXPECT_EQ(segment->id().id, std::string(Segment::kDragwaySegmentId));

  const int kLaneIndex = 0;
  const api::Lane* api_lane = segment->lane(kLaneIndex);
  ASSERT_NE(api_lane, nullptr);
  const NorthboundLane* northbound_lane =
      dynamic_cast<const NorthboundLane*>(api_lane);
  ASSERT_NE(northbound_lane, nullptr);
  EXPECT_EQ(northbound_lane->segment(), segment);

  VerifyNorthboundLaneCorrectness(northbound_lane, kLaneIndex);
  VerifyBranches(northbound_lane, &road_geometry);
}

/*
 Tests a dragway RoadGeometry containing a single southbound lane.
 It is arranged as shown below in the world frame:

                     x
                     ^
         r_min  r_max|
       --------------|     s = 0
       |      :      |
       |      V      |
       |      :      |
       |      V      |
  y <--+-------------o--> s = kLength
                     |
                     V
 */
TEST_F(MaliputDragwayLaneTest, SingleSouthboundLane) {
  const api::RoadGeometryId road_geometry_id(
      {"SingleSouthboundLaneRoadGeometry"});
  const int kNumSouthboundLanes = 1;
  const int kNumNorthboundLanes = 0;

  RoadGeometry road_geometry(road_geometry_id, kNumSouthboundLanes,
      kNumNorthboundLanes, kLength_, kLaneBounds_, kDriveableBounds_);

  const Junction* junction = road_geometry.junction();
  ASSERT_NE(junction, nullptr);
  EXPECT_EQ(junction->length(), kLength_);

  const Segment* segment = junction->segment();
  ASSERT_NE(segment, nullptr);
  EXPECT_EQ(segment->num_lanes(), 1);
  EXPECT_EQ(segment->num_southbound_lanes(), kNumSouthboundLanes);
  EXPECT_EQ(segment->num_northbound_lanes(), kNumNorthboundLanes);
  EXPECT_EQ(segment->id().id, std::string(Segment::kDragwaySegmentId));

  const api::Lane* api_lane = segment->lane(0);
  ASSERT_NE(api_lane, nullptr);
  const SouthboundLane* southbound_lane =
      dynamic_cast<const SouthboundLane*>(api_lane);
  ASSERT_NE(southbound_lane, nullptr);
  EXPECT_EQ(southbound_lane->segment(), segment);

  const int kLaneIndex = 0;
  VerifySouthboundLaneCorrectness(southbound_lane, kLaneIndex);
  VerifyBranches(southbound_lane, &road_geometry);
}

/*
 Tests a dragway containing one southbound lane and one northbound lane. The two
 lanes are arranged as shown below in the world frame:

                              x
                              ^
                  r_min r_max | r_max  r_min
    s = 0       --------------|--------------  s = kLength
                |      :      |      ^      |
                |      V      |      :      |
                |      :      |      ^      |
    s = kLength |      V      |      :      |  s = 0
      y <--+------------------o------------------------------>
                 (southbound) | (northbound)
                              V
 */
TEST_F(MaliputDragwayLaneTest, TwoLaneDragway) {
  const api::RoadGeometryId road_geometry_id(
      {"TwoLaneRoadGeometry"});
  const int kNumSouthboundLanes = 1;
  const int kNumNorthboundLanes = 1;

  RoadGeometry road_geometry(road_geometry_id, kNumSouthboundLanes,
      kNumNorthboundLanes, kLength_, kLaneBounds_, kDriveableBounds_);

  const Junction* junction = road_geometry.junction();
  ASSERT_NE(junction, nullptr);
  EXPECT_EQ(junction->length(), kLength_);

  const Segment* segment = junction->segment();
  ASSERT_NE(segment, nullptr);
  EXPECT_EQ(segment->num_lanes(), 2);
  EXPECT_EQ(segment->num_southbound_lanes(), kNumSouthboundLanes);
  EXPECT_EQ(segment->num_northbound_lanes(), kNumNorthboundLanes);
  EXPECT_EQ(segment->id().id, std::string(Segment::kDragwaySegmentId));

  const api::Lane* api_lane_0 = segment->lane(0);
  const api::Lane* api_lane_1 = segment->lane(1);
  ASSERT_NE(api_lane_0, nullptr);
  ASSERT_NE(api_lane_1, nullptr);

  const SouthboundLane* southbound_lane =
      dynamic_cast<const SouthboundLane*>(api_lane_0);
  const NorthboundLane* northbound_lane =
      dynamic_cast<const NorthboundLane*>(api_lane_1);
  ASSERT_NE(southbound_lane, nullptr);
  ASSERT_NE(northbound_lane, nullptr);

  // There is only one northbound and one southbound lane. Thus, both indices
  // are zero.
  const int kSouthboundLaneIndex = 0;
  const int kNorthboundLaneIndex = 0;

  VerifySouthboundLaneCorrectness(southbound_lane, kSouthboundLaneIndex);
  VerifyNorthboundLaneCorrectness(northbound_lane, kNorthboundLaneIndex);

  // Verifies the left and right lanes are correctly set.
  EXPECT_EQ(southbound_lane->to_left(), northbound_lane);
  EXPECT_EQ(southbound_lane->to_right(), nullptr);
  EXPECT_EQ(northbound_lane->to_left(), southbound_lane);
  EXPECT_EQ(northbound_lane->to_right(), nullptr);
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
