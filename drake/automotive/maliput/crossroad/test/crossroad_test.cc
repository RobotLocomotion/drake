#include "gtest/gtest.h"

#include "drake/automotive/maliput/crossroad/branch_point.h"
#include "drake/automotive/maliput/crossroad/junction.h"
#include "drake/automotive/maliput/crossroad/lane.h"
#include "drake/automotive/maliput/crossroad/road_geometry.h"

namespace drake {
namespace maliput {
namespace crossroad {
namespace {

// To understand the characteristics of the geometry, consult the
// crossroad::Segment and crossroad::Lane detailed class overview docs.
class MaliputCrossroadLaneTest : public ::testing::Test {
 public:
  MaliputCrossroadLaneTest()
      : length_(100.0), lane_width_(6.0), shoulder_width_(0.5) {}

  // Contains expected driveable r_min, driveable r_max, and r_offset parameters
  // of a Crossroad Lane.
  struct ExpectedLaneParameters {
    double r_offset{};
    double driveable_r_min{};
    double driveable_r_max{};
  };

  ExpectedLaneParameters GetExpectedLaneParameters(int num_lanes,
                                                   int lane_index) const {
    ExpectedLaneParameters result{};
    if (num_lanes == 1) {
      result.r_offset = 0.0;
      result.driveable_r_min = -3.5;
      result.driveable_r_max = 3.5;
    } else if ((num_lanes == 2) && (lane_index == 0)) {
      result.r_offset = -3.0;
      result.driveable_r_min = -3.5;
      result.driveable_r_max = 9.5;
    } else if ((num_lanes == 2) && (lane_index == 1)) {
      result.r_offset = 3.0;
      result.driveable_r_min = -9.5;
      result.driveable_r_max = 3.5;
    } else {
      throw std::runtime_error("GetExpectedLaneParameters: bad input");
    }
    return result;
  }

  // Verifies the correctness of the provided `lane`.
  void VerifyLaneCorrectness(const api::Lane* lane, int num_lanes) {
    const ExpectedLaneParameters expected =
        GetExpectedLaneParameters(num_lanes, lane->index());

    // Tests Lane::lane_bounds().
    for (double s = 0; s < length_; s += length_ / 100) {
      const api::RBounds lane_bounds = lane->lane_bounds(s);
      EXPECT_EQ(lane_bounds.r_min, -lane_width_ / 2);
      EXPECT_EQ(lane_bounds.r_max, lane_width_ / 2);
    }

    EXPECT_EQ(lane->length(), length_);

    // Tests Lane::driveable_bounds().
    for (double s = 0; s < length_; s += length_ / 100) {
      const api::RBounds driveable_bounds = lane->driveable_bounds(s);
      EXPECT_EQ(driveable_bounds.r_min, expected.driveable_r_min);
      EXPECT_EQ(driveable_bounds.r_max, expected.driveable_r_max);
    }

    // The following block of test code evaluates methods that take as input a
    // (s, r, h) lane position. Ideally, we want to check that the
    // method-under-test is correct for all combinations of (s, r, h). This,
    // unfortunately, it not possible since there are too many combinations of
    // (s, r, h). Instead we pick points in a grid that spans the (s, r, h)
    // state space and only check those points in the hopes that they are
    // representative of the entire state space.
    const double driveable_width =
        expected.driveable_r_max - expected.driveable_r_min;
    for (double s = 0; s < length_; s += length_ / 100) {
      for (double r = expected.driveable_r_min; r <= expected.driveable_r_max;
           r += driveable_width / 100) {
        for (double h = 0; h < 1; h += 0.1) {
          const api::LanePosition lane_position(s, r, h);

          // Tests Lane::ToGeoPosition().
          const api::GeoPosition geo_position =
              lane->ToGeoPosition(lane_position);
          const double linear_tolerance =
              lane->segment()->junction()->road_geometry()->linear_tolerance();
          EXPECT_DOUBLE_EQ(geo_position.x, s);
          EXPECT_NEAR(geo_position.y, expected.r_offset + r, linear_tolerance);
          EXPECT_DOUBLE_EQ(geo_position.z, h);

          // Tests Lane::GetOrientation().
          const api::Rotation rotation = lane->GetOrientation(lane_position);
          EXPECT_DOUBLE_EQ(rotation.roll, 0);
          EXPECT_DOUBLE_EQ(rotation.pitch, 0);
          EXPECT_DOUBLE_EQ(rotation.yaw, 0);

          // Tests Lane::EvalMotionDerivatives().
          //
          // The following translational velocities can be any value. We just
          // want to verify that the same values are returned from
          // Lane::EvalMotionDerivatives().
          const double kSigma_v = 1.1;
          const double kRho_v = 2.2;
          const double kEta_v = 3.3;

          const api::LanePosition motion_derivatives =
              lane->EvalMotionDerivatives(
                  lane_position,
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
    // Verifies that the same BranchPoint covers both ends of the crossroad
    // lane.
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

  const double length_{};
  const double lane_width_{};
  const double shoulder_width_{};
};

/*
 Tests a crossroad containing one lane. It is arranged as shown below in the
 world frame:

               x
               ^
      |<-------|------->|    driveable r_max / r_min
      | |<-----|----->| |    lane      r_max / r_min
      -------------------    s = length_
      | |      ^      | |
      | |      :      | |
      | |      ^      | |
      | |      :      | |
  y <----------o---------->  s = 0
               |
               V
 */
TEST_F(MaliputCrossroadLaneTest, SingleLane) {
  const api::RoadGeometryId road_geometry_id({"OneLaneCrossroadRoadGeometry"});
  const int kNumLanes = 1;
  // The following linear tolerance was empirically derived on a 64-bit Ubuntu
  // system. It is necessary due to inaccuracies in floating point calculations
  // and different ways of computing the driveable r_min and r_max.
  const double kLinearTolerance = 1e-15;

  RoadGeometry road_geometry(road_geometry_id, kNumLanes, length_, lane_width_,
                             shoulder_width_, kLinearTolerance);

  const api::Junction* junction = road_geometry.junction(0);
  ASSERT_NE(junction, nullptr);
  const api::Segment* segment = junction->segment(0);
  ASSERT_NE(segment, nullptr);
  EXPECT_EQ(segment->lane(0)->length(), length_);
  EXPECT_EQ(segment->num_lanes(), 1);
  EXPECT_EQ(segment->id().id, "Crossroad_Segment_ID");

  const int kLaneIndex = 0;
  const api::Lane* lane = segment->lane(kLaneIndex);
  ASSERT_NE(lane, nullptr);
  EXPECT_EQ(lane->segment(), segment);

  VerifyLaneCorrectness(lane, kNumLanes);
  VerifyBranches(lane, &road_geometry);
}

/*
 Tests a crossroad containing two lanes. The two lanes are arranged as shown
 below
 in the world frame:

                              x
                              ^
                              |
              |<-------|--------------------->|  lane 1 driveable r_max / r_min
              |<---------------------|------->|  lane 0 driveable r_max / r_min
              | |<-----|----->|<-----|----->| |  lane             r_max / r_min
              ----------------|----------------  s = length_
              | |      ^      |      ^      | |
              | |      :      |      :      | |
              | |      ^      |      ^      | |
              | |      :      |      :      | |  s = 0
      y <---------------------o------------------------------>
                    index 1   |    index 0
                              V
 */
TEST_F(MaliputCrossroadLaneTest, TwoLaneCrossroad) {
  const api::RoadGeometryId road_geometry_id({"TwoLaneCrossroadRoadGeometry"});
  const int kNumLanes = 2;
  // The following linear tolerance was empirically derived on a 64-bit Ubuntu
  // system. It is necessary due to inaccuracies in floating point calculations
  // and different ways of computing the driveable r_min and r_max.
  const double kLinearTolerance = 1e-15;

  RoadGeometry road_geometry(road_geometry_id, kNumLanes, length_, lane_width_,
                             shoulder_width_, kLinearTolerance);

  const api::Junction* junction = road_geometry.junction(0);
  ASSERT_NE(junction, nullptr);
  const api::Segment* segment = junction->segment(0);
  ASSERT_NE(segment, nullptr);
  const api::Lane* lane_zero = segment->lane(0);
  ASSERT_NE(lane_zero, nullptr);
  EXPECT_EQ(lane_zero->length(), length_);
  EXPECT_EQ(segment->num_lanes(), kNumLanes);
  EXPECT_EQ(segment->id().id, "Crossroad_Segment_ID");

  for (int i = 0; i < kNumLanes; ++i) {
    const api::Lane* lane = segment->lane(i);
    ASSERT_NE(lane, nullptr);
    VerifyLaneCorrectness(lane, kNumLanes);
    VerifyBranches(lane, &road_geometry);
  }
}

}  // namespace
}  // namespace crossroad
}  // namespace maliput
}  // namespace drake
