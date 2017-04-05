#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"

namespace drake {
namespace maliput {
namespace rndf {

const double kLinearTolerance = 1e-6;
const double kAngularTolerance = 1e-6;


#define EXPECT_RBOUNDS_EQ(actual, expected)                       \
  do {                                                            \
    const api::RBounds _actual(actual.r_min, actual.r_max);       \
    const api::RBounds _expected(expected.r_min, expected.r_max); \
    EXPECT_EQ(_actual.r_min, _expected.r_min);                    \
    EXPECT_EQ(_actual.r_max, _expected.r_max);                    \
  } while (0)

// Check the metadata of a lane and see if it behaves OK
GTEST_TEST(RNDFSplineLanesTest, MetadataLane) {
  const double width = 5.;
  const api::RBounds lane_bounds(-width / 2., width / 2.);
  const api::RBounds driveable_bounds(-width / 2., width / 2.);
  RoadGeometry rg({"FlatLineLane"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  Lane *l1 = s1->NewSplineLane(
    {"l1"},
    control_points,
    width);

  // Check road geometry invariants
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  // Check meta properties
  {
    EXPECT_EQ(l1->id().id, "l1");
    EXPECT_EQ(l1->segment(), s1);
    EXPECT_EQ(l1->index(), 0);
    EXPECT_EQ(l1->to_left(), nullptr);
    EXPECT_EQ(l1->to_right(), nullptr);
    EXPECT_RBOUNDS_EQ(l1->lane_bounds(0.), lane_bounds);
    EXPECT_RBOUNDS_EQ(l1->driveable_bounds(0.), driveable_bounds);
  }

  {
    EXPECT_NO_THROW(s1->NewSplineLane(
        {"l2"},
        control_points,
        width));
  }
}

// Check if left and right sides are resolved OK
// 1.1.1 ----->1.1.2
// 1.2.1 ----->1.2.2
// 1.3.1 ----->1.3.2
GTEST_TEST(RNDFSplineLanesTest, RigthLeftTest) {
  const double width = 5.;
  const api::RBounds lane_bounds(-width / 2., width / 2.);
  const api::RBounds driveable_bounds(-width / 2., width / 2.);
  RoadGeometry rg({"FlatLineLane"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  // Lane 1.1
  {
    std::vector<
      std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
        control_points;
    control_points.push_back(
      std::make_tuple(
        ignition::math::Vector3d(0.0, 0.0, 0.0),
        ignition::math::Vector3d(10.0, 0.0, 0.0)));
    control_points.push_back(
      std::make_tuple(
        ignition::math::Vector3d(20.0, 0.0, 0.0),
        ignition::math::Vector3d(10.0, 0.0, 0.0)));
    s1->NewSplineLane(
      {"l1"},
      control_points,
      width);
  }
  // Lane 1.2
  {
    std::vector<
      std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
        control_points;
    control_points.push_back(
      std::make_tuple(
        ignition::math::Vector3d(0.0, 5.0, 0.0),
        ignition::math::Vector3d(10.0, 0.0, 0.0)));
    control_points.push_back(
      std::make_tuple(
        ignition::math::Vector3d(20.0, 5.0, 0.0),
        ignition::math::Vector3d(10.0, 0.0, 0.0)));
    s1->NewSplineLane(
      {"l2"},
      control_points,
      width);
  }
  // Lane 1.3
  {
    std::vector<
      std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
        control_points;
    control_points.push_back(
      std::make_tuple(
        ignition::math::Vector3d(0.0, 10.0, 0.0),
        ignition::math::Vector3d(10.0, 0.0, 0.0)));
    control_points.push_back(
      std::make_tuple(
        ignition::math::Vector3d(20.0, 10.0, 0.0),
        ignition::math::Vector3d(10.0, 0.0, 0.0)));
    s1->NewSplineLane(
      {"l3"},
      control_points,
      width);
  }
  // Check road geometry invariants
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  // Check meta properties
  {
    EXPECT_EQ(s1->lane(0)->to_left(), s1->lane(1));
    EXPECT_EQ(s1->lane(0)->to_right(), nullptr);

    EXPECT_EQ(s1->lane(1)->to_left(), s1->lane(2));
    EXPECT_EQ(s1->lane(1)->to_right(), s1->lane(0));

    EXPECT_EQ(s1->lane(2)->to_left(), nullptr);
    EXPECT_EQ(s1->lane(2)->to_right(), s1->lane(1));
  }
}
// We create a -.- connection and check its creation and correct invariants from
// the road geometry. It will look like:

// 1.1.1 ----->1.1.2------>1.1.3

// This simple example is useful for checking several Lane and BranchPoint
// functions.
GTEST_TEST(RNDFSplineLanesTest, BranchpointsLane) {
  const double width = 5.;
  RoadGeometry rg({"BranchpointsLane"}, kLinearTolerance, kAngularTolerance);
  Lane *l1, *l2;
  {
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  l1 = s1->NewSplineLane(
    {"l1"},
    control_points,
    width);
  }
  {
  Segment* s2 = rg.NewJunction({"j2"})->NewSegment({"s2"});
  std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(40.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  l2 = s2->NewSplineLane(
    {"l2"},
    control_points,
    width);
  }
  BranchPoint *bp1, *bp2, *bp3;

  bp1 = rg.NewBranchPoint({"bp:" + std::to_string(1)});
  bp2 = rg.NewBranchPoint({"bp:" + std::to_string(2)});
  bp3 = rg.NewBranchPoint({"bp:" + std::to_string(3)});


  l1->SetStartBp(bp1);
  l1->SetEndBp(bp2);
  l2->SetStartBp(bp2);
  l2->SetEndBp(bp3);

  bp1->AddABranch({l1, api::LaneEnd::kStart});
  bp2->AddABranch({l1, api::LaneEnd::kFinish});
  bp2->AddBBranch({l2, api::LaneEnd::kStart});
  bp3->AddABranch({l2, api::LaneEnd::kFinish});

  // Check road geometry invariants
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());


  EXPECT_EQ(l1->start_bp(), bp1);
  EXPECT_EQ(l1->end_bp(), bp2);
  EXPECT_EQ(l2->start_bp(), bp2);
  EXPECT_EQ(l2->end_bp(), bp3);
}


}  // namespace rndf
}  // namespace maliput
}  // namespace drake
