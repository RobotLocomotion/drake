#include <cmath>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/dragway/branch_point.h"
#include "drake/automotive/maliput/dragway/junction.h"
#include "drake/automotive/maliput/dragway/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"

namespace drake {
namespace maliput {
namespace dragway {
namespace {

// To understand the characteristics of the geometry, consult the
// dragway::Segment and dragway::Lane detailed class overview docs.
class MaliputDragwayLaneTest : public ::testing::Test {
 public:
  MaliputDragwayLaneTest()
      : length_(100.0),
        lane_width_(6.0),
        shoulder_width_(0.5) {
  }

  // Contains expected driveable r_min, driveable r_max, and y_offset parameters
  // of a Dragway Lane.
  struct ExpectedLaneParameters {
    double y_offset{};
    double driveable_r_min{};
    double driveable_r_max{};
  };

  ExpectedLaneParameters GetExpectedLaneParameters(
      int num_lanes, int lane_index) const {
    ExpectedLaneParameters result{};
    if (num_lanes == 1) {
      result.y_offset = 0.0;
      result.driveable_r_min = -3.5;
      result.driveable_r_max = 3.5;
    } else if ((num_lanes == 2) && (lane_index == 0)) {
      result.y_offset = -3.0;
      result.driveable_r_min = -3.5;
      result.driveable_r_max = 9.5;
    } else if ((num_lanes == 2) && (lane_index == 1)) {
      result.y_offset = 3.0;
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
    for (double s = 0 ; s < length_; s += length_ / 100) {
      const api::RBounds lane_bounds = lane->lane_bounds(s);
      EXPECT_EQ(lane_bounds.r_min, -lane_width_ / 2);
      EXPECT_EQ(lane_bounds.r_max, lane_width_ / 2);
    }

    EXPECT_EQ(lane->length(), length_);

    // Tests Lane::driveable_bounds().
    for (double s = 0 ; s < length_; s += length_ / 100) {
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
          EXPECT_NEAR(geo_position.y, expected.y_offset + r, linear_tolerance);
          EXPECT_DOUBLE_EQ(geo_position.z, h);

          // Tests Lane::GetOrientation().
          const api::Rotation rotation =
              lane->GetOrientation(lane_position);
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

  const double length_{};
  const double lane_width_{};
  const double shoulder_width_{};

  // The following linear tolerance was empirically derived on a 64-bit Ubuntu
  // system. It is necessary due to inaccuracies in floating point calculations
  // and different ways of computing the driveable r_min and r_max.
  const double kLinearTolerance = 1e-15;
};

/*
 Tests a dragway containing one lane. It is arranged as shown below in the world
 frame:

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
TEST_F(MaliputDragwayLaneTest, SingleLane) {
  const api::RoadGeometryId road_geometry_id({"OneLaneDragwayRoadGeometry"});
  const int kNumLanes = 1;

  RoadGeometry road_geometry(road_geometry_id, kNumLanes, length_, lane_width_,
      shoulder_width_, kLinearTolerance);

  const api::Junction* junction = road_geometry.junction(0);
  ASSERT_NE(junction, nullptr);
  const api::Segment* segment = junction->segment(0);
  ASSERT_NE(segment, nullptr);
  EXPECT_EQ(segment->lane(0)->length(), length_);
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
TEST_F(MaliputDragwayLaneTest, TwoLaneDragway) {
  const api::RoadGeometryId road_geometry_id({"TwoLaneDragwayRoadGeometry"});
  const int kNumLanes = 2;

  RoadGeometry road_geometry(road_geometry_id, kNumLanes, length_,
      lane_width_, shoulder_width_, kLinearTolerance);

  const api::Junction* junction = road_geometry.junction(0);
  ASSERT_NE(junction, nullptr);
  const api::Segment* segment = junction->segment(0);
  ASSERT_NE(segment, nullptr);
  const api::Lane* lane_zero = segment->lane(0);
  ASSERT_NE(lane_zero, nullptr);
  EXPECT_EQ(lane_zero->length(), length_);
  EXPECT_EQ(segment->num_lanes(), kNumLanes);
  EXPECT_EQ(segment->id().id, "Dragway_Segment_ID");

  for (int i = 0; i < kNumLanes; ++i) {
    const api::Lane* lane = segment->lane(i);
    ASSERT_NE(lane, nullptr);
    VerifyLaneCorrectness(lane, kNumLanes);
    VerifyBranches(lane, &road_geometry);
  }
}

// Tests dragway::RoadGeometry::ToRoadPosition() using a two-lane dragway where
// the x,y projection of all geographic positions provided to it are in the
// road's driveable region. This unit test also verifies that
// dragway::RoadGeometry::IsGeoPositionOnDragway() does not incorrectly return
// false.
TEST_F(MaliputDragwayLaneTest, TestToRoadPositionOnRoad) {
  const api::RoadGeometryId road_geometry_id({"TwoLaneDragwayRoadGeometry"});
  const int kNumLanes = 2;

  RoadGeometry road_geometry(road_geometry_id, kNumLanes, length_,
      lane_width_, shoulder_width_, kLinearTolerance);

  // Spot checks geographic positions on lane 0 and the right shoulder with a
  // focus on edge cases.
  for (double x = 0; x <= length_; x += length_ / 2) {
    for (double y = -lane_width_ - shoulder_width_; y <= 0;
        y += (lane_width_ + shoulder_width_) / 2) {
      for (double z = 0; z <= 10; z += 5) {
        api::GeoPosition nearest_position;
        double distance;
        const api::RoadPosition road_position = road_geometry.ToRoadPosition(
            api::GeoPosition(x, y, z), nullptr /* hint */, &nearest_position,
            &distance);
        const api::Lane* expected_lane =
            road_geometry.junction(0)->segment(0)->lane(0);
        EXPECT_DOUBLE_EQ(nearest_position.x, x);
        EXPECT_DOUBLE_EQ(nearest_position.y, y);
        EXPECT_DOUBLE_EQ(nearest_position.z, z);
        EXPECT_DOUBLE_EQ(distance, 0);
        EXPECT_EQ(road_position.lane, expected_lane);
        EXPECT_EQ(road_position.pos.s, x);
        EXPECT_EQ(road_position.pos.r, y + lane_width_ / 2);
        EXPECT_EQ(road_position.pos.h, z);
      }
    }
  }

  // Spot checks geographic positions on lane 1 and the left shoulder with a
  // focus on edge cases.
  for (double x = 0; x <= length_; x += length_ / 2) {
    for (double y = 0; y <= lane_width_ + shoulder_width_;
        y += (lane_width_ + shoulder_width_) / 2) {
      for (double z = 0; z <= 10; z += 5) {
        api::GeoPosition nearest_position;
        double distance;
        const api::RoadPosition road_position = road_geometry.ToRoadPosition(
            api::GeoPosition(x, y, z), nullptr /* hint */, &nearest_position,
            &distance);
        const int lane_index = (y == 0 ? 0 : 1);
        const api::Lane* expected_lane =
            road_geometry.junction(0)->segment(0)->lane(lane_index);
        EXPECT_DOUBLE_EQ(nearest_position.x, x);
        EXPECT_DOUBLE_EQ(nearest_position.y, y);
        EXPECT_DOUBLE_EQ(nearest_position.z, z);
        EXPECT_DOUBLE_EQ(distance, 0);
        EXPECT_EQ(road_position.lane, expected_lane);
        EXPECT_EQ(road_position.pos.s, x);
        if (y == 0) {
          EXPECT_EQ(road_position.pos.r, y + lane_width_ / 2);
        } else {
          EXPECT_EQ(road_position.pos.r, y - lane_width_ / 2);
        }
        EXPECT_EQ(road_position.pos.h, z);
      }
    }
  }
}

// Tests dragway::RoadGeometry::ToRoadPosition() using a two-lane dragway where
// the x,y projection of all geographic positions provided to it may be outside
// of the road's driveable region. This unit test also verifies that
// dragway::RoadGeometry::IsGeoPositionOnDragway() does not incorrectly return
// false.
TEST_F(MaliputDragwayLaneTest, TestToRoadPositionOffRoad) {
  const api::RoadGeometryId road_geometry_id({"TwoLaneDragwayRoadGeometry"});
  const int kNumLanes = 2;

  RoadGeometry road_geometry(road_geometry_id, kNumLanes, length_,
      lane_width_, shoulder_width_, kLinearTolerance);

  // Computes the bounds of the road's driveable region.
  const double x_max = length_;
  const double x_min = 0;
  const double y_max = lane_width_ + shoulder_width_;
  const double y_min = -y_max;

  // Spot checks a region that is a superset of the road's driveable bounds with
  // a focus on edge cases.
  const double test_x_min = x_min - 10;
  const double test_x_max = x_max + 10;
  const double test_y_min = y_min - 10;
  const double test_y_max = y_max + 10;

  // Defines the test case values for x and y. Points both far away and close to
  // the driveable area are evaluated.
  const std::vector<double> x_test_cases{
    test_x_min, x_min - 1e-10, x_max + 1e-10, test_x_max};
  const std::vector<double> y_test_cases{
    test_y_min, y_min - 1e-10, y_max + 1e-10, test_y_max};

  for (const auto x : x_test_cases) {
    for (const auto y : y_test_cases) {
      // Provide a non-zero `z` value for one test case just to ensure that the
      // results of dragway::RoadGeometry::ToRoadPosition() are reasonable for
      // this scenario.
      const double z = (x == test_x_min && y == test_y_min) ? 0.01 : 0;
      api::GeoPosition nearest_position;
      double distance;
      const api::RoadPosition road_position = road_geometry.ToRoadPosition(
          api::GeoPosition(x, y, z), nullptr /* hint */, &nearest_position,
          &distance);
      EXPECT_LE(nearest_position.x, x_max);
      EXPECT_GE(nearest_position.x, x_min);
      EXPECT_LE(nearest_position.y, y_max);
      EXPECT_GE(nearest_position.y, y_min);

      api::GeoPosition expected_nearest_position;
      expected_nearest_position.x = x;
      expected_nearest_position.y = y;
      expected_nearest_position.z = z;
      if (x < x_min) {
        expected_nearest_position.x = x_min;
      }
      if (x > x_max) {
        expected_nearest_position.x = x_max;
      }
      if (y < y_min) {
        expected_nearest_position.y = y_min;
      }
      if (y > y_max) {
        expected_nearest_position.y = y_max;
      }

      EXPECT_DOUBLE_EQ(nearest_position.x, expected_nearest_position.x);
      EXPECT_DOUBLE_EQ(nearest_position.y, expected_nearest_position.y);
      EXPECT_DOUBLE_EQ(nearest_position.z, expected_nearest_position.z);
      EXPECT_LT(0, distance);
      const int expected_lane_index = (y > 0 ? 1 : 0);
      const Lane* expected_lane = dynamic_cast<const Lane*>(
          road_geometry.junction(0)->segment(0)->lane(expected_lane_index));
      EXPECT_EQ(road_position.lane, expected_lane);
      EXPECT_EQ(road_position.pos.s, expected_nearest_position.x);
      EXPECT_EQ(road_position.pos.r,
          expected_nearest_position.y - expected_lane->y_offset());
      EXPECT_EQ(road_position.pos.h, z);
    }
  }
}

// Tests that dragway::RoadGeometry::ToRoadPosition() can be called with
// parameters `nearest_position` and `distance` set to `nullptr`.
TEST_F(MaliputDragwayLaneTest, TestToRoadPositionNullptr) {
  const api::RoadGeometryId road_geometry_id({"TwoLaneDragwayRoadGeometry"});
  const int kNumLanes = 2;

  RoadGeometry road_geometry(road_geometry_id, kNumLanes, length_,
      lane_width_, shoulder_width_, kLinearTolerance);

  // Case 1: The provided geo_pos is in the driveable region.
  EXPECT_NO_THROW(road_geometry.ToRoadPosition(
            api::GeoPosition(length_ / 2 /* x */, 0 /* y */, 0 /* z */),
            nullptr /* hint */,
            nullptr /* nearest_position */,
            nullptr /* distance */));

  // Case 2: The provided geo_pos is beyond the driveable region.
  EXPECT_NO_THROW(road_geometry.ToRoadPosition(
            api::GeoPosition(length_ + 1e-10 /* x */, 0 /* y */, 0 /* z */),
            nullptr /* hint */,
            nullptr /* nearest_position */,
            nullptr /* distance */));
}

// Tests dragway::Lane::ToLanePosition() using geographic positions whose
// projections onto the XY plane reside within the lane's driveable region.
TEST_F(MaliputDragwayLaneTest, TestToLanePosition) {
  const api::RoadGeometryId road_geometry_id({"OneLaneDragwayRoadGeometry"});
  const int kNumLanes = 1;

  RoadGeometry road_geometry(road_geometry_id, kNumLanes, length_,
      lane_width_, shoulder_width_, kLinearTolerance);

  const api::Junction* junction = road_geometry.junction(0);
  ASSERT_NE(junction, nullptr);
  const api::Segment* segment = junction->segment(0);
  ASSERT_NE(segment, nullptr);
  const Lane* lane = dynamic_cast<const Lane*>(segment->lane(0));
  ASSERT_NE(lane, nullptr);
  EXPECT_EQ(lane->length(), length_);

  /*
    A figure of the one-lane dragway is shown below. The minimum and maximum
    values of the dragway's driveable region are demarcated.
 
                        X
              Y = max_y ^  Y = min_y
                        :
                  |     :     |
                  |     :     |
          --------+-----+-----+---------  X = max_x
                  | .   :   . |
                  | .   :   . |
                  | .   :   . |
                  | .  The  . |
                  | .Dragway. |
                  | .   :   . |
                  | .   :   . |
                  | .   :   . |
     Y <----------+-----o-----+---------  X = min_x
                  |     :     |
                        :
                ->|-|<- : ->|-|<-  shoulder_width
                        :
                    |<--:-->|      lane_width
                        V
  */
  const double min_x = 0;
  const double max_x = length_;
  const double min_y = -lane_width_ / 2 - shoulder_width_;
  const double max_y = lane_width_ / 2 + shoulder_width_;

  // Defines the test case values for x and y. Points both far away and close to
  // the driveable area are evaluated.
  const std::vector<double> x_test_cases{
    min_x - 10, min_x - 1e-10, min_x, max_x, max_x + 1e-10, max_x + 10};
  const std::vector<double> y_test_cases{
    min_y - 10, min_y - 1e-10, min_y, max_y, max_y + 1e-10, max_y + 10};

  // Spot checks geographic positions on and beyond the lane's driveable region
  // with a focus on edge cases.
  for (const auto x : x_test_cases) {
    for (const auto y : y_test_cases) {
      // Test one case where z is not equal to zero.
      const double z = (x == min_x && y == min_y) ? 0.1 : 0;
      api::GeoPosition nearest_position;
      double distance;
      const api::LanePosition lane_position = lane->ToLanePosition(
          api::GeoPosition(x, y, z), &nearest_position, &distance);
      api::GeoPosition expected_nearest_position(x, y, z);
      if (x < min_x) {
        expected_nearest_position.x = min_x;
      }
      if (x > max_x) {
        expected_nearest_position.x = max_x;
      }
      if (y < min_y) {
        expected_nearest_position.y = min_y;
      }
      if (y > max_y) {
        expected_nearest_position.y = max_y;
      }
      EXPECT_DOUBLE_EQ(nearest_position.x, expected_nearest_position.x);
      EXPECT_DOUBLE_EQ(nearest_position.y, expected_nearest_position.y);
      EXPECT_DOUBLE_EQ(nearest_position.z, expected_nearest_position.z);
      EXPECT_GE(distance, 0);
      EXPECT_EQ(lane_position.s, expected_nearest_position.x);
      EXPECT_EQ(lane_position.r,
          expected_nearest_position.y - lane->y_offset());
      EXPECT_EQ(lane_position.h, expected_nearest_position.z);
    }
  }
}

}  // namespace
}  // namespace dragway
}  // namespace maliput
}  // namespace drake
