/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/builder.h"
/* clang-format on */

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace {

// TODO(maddog-tri)  Test use of Endpoint::reverse() with non-zero theta and
//                   theta_dot, checking that the orientation of the resulting
//                   road surface is continuous, off the centerline, at the
//                   branch-point where two connections connect

GTEST_TEST(MultilaneBuilderTest, Fig8) {
  const double kLaneWidth = 4.;
  const double kLeftShoulder = 2.;
  const double kRightShoulder = 2.;
  const api::HBounds kElevationBounds(0., 5.);
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance);

  const EndpointZ kLowFlatZ(0., 0., 0., 0.);
  const EndpointZ kMidFlatZ(3., 0., 0., 0.);
  const EndpointZ kMidTiltLeftZ(3., 0., -0.4, 0.);
  const EndpointZ kMidTiltRightZ(3., 0., 0.4, 0.);
  const EndpointZ kHighFlatZ(6., 0., 0., 0.);

  const ArcOffset kCounterClockwiseArc(50., 0.75 * M_PI);  // 135deg, 50m radius
  const ArcOffset kClockwiseArc(50., -0.75 * M_PI);  // 135deg, 50m radius

  const int kOneLane = 1;
  const double kZeroR0 = 0.;

  Endpoint start {{0., 0., -M_PI / 4.}, kLowFlatZ};
  auto c0 = b.Connect("0", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                      start, 50., kMidFlatZ);

  auto c1 = b.Connect("1", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                      c0->end(), kCounterClockwiseArc, kMidTiltLeftZ);
  auto c2 = b.Connect("2", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                      c1->end(), kCounterClockwiseArc, kMidFlatZ);

  auto c3 = b.Connect("3", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                      c2->end(), 50., kHighFlatZ);
  auto c4 = b.Connect("4", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                      c3->end(), 50., kMidFlatZ);

  auto c5 = b.Connect("5", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                      c4->end(), kClockwiseArc, kMidTiltRightZ);
  auto c6 = b.Connect("6", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                      c5->end(), kClockwiseArc, kMidFlatZ);

  // Tweak ends to check if fuzzy-matching is working.
  Endpoint c6end = c6->end();
  c6end = Endpoint(EndpointXy(c6end.xy().x() + kLinearTolerance * 0.5,
                              c6end.xy().y() - kLinearTolerance * 0.5,
                              c6end.xy().heading()),
                   EndpointZ(c6end.z().z() + kLinearTolerance * 0.5,
                             c6end.z().z_dot(),
                             c6end.z().theta(), c6end.z().theta_dot()));
  EndpointZ c0start_z = c0->start().z();
  c0start_z = EndpointZ(c0start_z.z() - kLinearTolerance * 0.5,
                        c0start_z.z_dot(),
                        c0start_z.theta(), c0start_z.theta_dot());

  b.Connect("7", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder, c6end, 50.,
            c0start_z);

  std::unique_ptr<const api::RoadGeometry> rg =
      b.Build(api::RoadGeometryId{"figure-eight"});

  EXPECT_EQ(rg->id(), api::RoadGeometryId("figure-eight"));

  EXPECT_EQ(rg->num_junctions(), 8);
  for (int j = 0; j < rg->num_junctions(); ++j) {
    const api::Junction* jnx = rg->junction(j);
    EXPECT_EQ(jnx->road_geometry(), rg.get());
    EXPECT_EQ(jnx->num_segments(), 1);

    const api::Segment* seg = jnx->segment(0);
    EXPECT_EQ(seg->junction(), jnx);
    EXPECT_EQ(seg->num_lanes(), kOneLane);

    const api::Lane* lane = seg->lane(0);
    EXPECT_EQ(lane->segment(), seg);
    EXPECT_EQ(lane->index(), 0);
    EXPECT_EQ(lane->to_left(), nullptr);
    EXPECT_EQ(lane->to_right(), nullptr);

    EXPECT_TRUE(lane->GetBranchPoint(api::LaneEnd::kStart) != nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 1);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);

    EXPECT_TRUE(lane->GetBranchPoint(api::LaneEnd::kFinish) != nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 1);
  }

  EXPECT_EQ(rg->num_branch_points(), 8);
  for (int bpi = 0; bpi < rg->num_branch_points(); ++bpi) {
    const api::BranchPoint* bp = rg->branch_point(bpi);
    EXPECT_EQ(bp->GetASide()->size(), 1);
    EXPECT_EQ(bp->GetBSide()->size(), 1);
  }
};


GTEST_TEST(MultilaneBuilderTest, QuadRing) {
  const double kLaneWidth = 4.;
  const double kLeftShoulder = 2.;
  const double kRightShoulder = 2.;
  const api::HBounds kElevationBounds(0., 5.);
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance);

  const EndpointZ kFlatZ(0., 0., 0., 0.);
  const ArcOffset kLargeClockwiseLoop(150., -2. * M_PI);
  const ArcOffset kSmallClockwiseLoop(50., -2. * M_PI);
  const ArcOffset kLargeCounterClockwiseLoop(150., 2. * M_PI);
  const ArcOffset kSmallCounterClockwiseLoop(50., 2. * M_PI);

  const int kOneLane = 1;
  const double kZeroR0 = 0.;

  Endpoint northbound {{0., 0., M_PI / 2.}, kFlatZ};

  // This heads -y, loops to -x, clockwise, back to origin.
  auto left1 =
      b.Connect("left1", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                northbound.reverse(), kLargeClockwiseLoop, kFlatZ);
  // This heads +y, loops to -x, counterclockwise, back to origin.
  auto left0 =
      b.Connect("left0", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                northbound, kSmallCounterClockwiseLoop, kFlatZ);
  // This heads +y, loops to +x, clockwise, back to origin.
  auto right0 =
      b.Connect("right0", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                northbound, kSmallClockwiseLoop, kFlatZ);
  // This heads -y, loops to +x, counterclockwise, back to origin.
  auto right1 =
      b.Connect("right1", kOneLane, kZeroR0, kLeftShoulder, kRightShoulder,
                northbound.reverse(), kLargeCounterClockwiseLoop, kFlatZ);

  // There is only one branch-point in this topology, since every lane is
  // a ring connecting back to itself and all other lanes.
  //
  //                    -y <-------+-------> +y
  //
  //  #  -----< left1/start   o--      ---o  left1/finish <------  #
  //  #                          \    /                            #
  //  #  -----> left0/finish  o---\  /----o   left0/start >------  #
  //  #                            BP                              #
  //  #  -----> right0/finish o---/  \----o  right0/start >------  #
  //  #                          /    \                            #
  //  #  -----< right1/start  o--      ---o right1/finish <------  #
  //
  // Make up some combination of default routes:
  const int kLaneIndex{0};
  b.SetDefaultBranch(left1, kLaneIndex, api::LaneEnd::kStart, left1, kLaneIndex,
                     api::LaneEnd::kFinish);
  b.SetDefaultBranch(left0, kLaneIndex, api::LaneEnd::kFinish, right0,
                     kLaneIndex, api::LaneEnd::kStart);
  b.SetDefaultBranch(right0, kLaneIndex, api::LaneEnd::kFinish, right1,
                     kLaneIndex, api::LaneEnd::kFinish);
  b.SetDefaultBranch(right1, kLaneIndex, api::LaneEnd::kStart, left1,
                     kLaneIndex, api::LaneEnd::kFinish);

  b.SetDefaultBranch(left1, kLaneIndex, api::LaneEnd::kFinish, left1,
                     kLaneIndex, api::LaneEnd::kStart);
  b.SetDefaultBranch(left0, kLaneIndex, api::LaneEnd::kStart, right1,
                     kLaneIndex, api::LaneEnd::kStart);
  b.SetDefaultBranch(right0, kLaneIndex, api::LaneEnd::kStart, right1,
                     kLaneIndex, api::LaneEnd::kStart);
  // And, leave right1/finish without a default branch.

  b.MakeGroup("all", {right1, right0, left0, left1});

  std::unique_ptr<const api::RoadGeometry> rg =
      b.Build(api::RoadGeometryId{"figure-eight"});

  EXPECT_EQ(rg->num_branch_points(), 1);
  EXPECT_EQ(rg->branch_point(0)->GetASide()->size(), 4);
  EXPECT_EQ(rg->branch_point(0)->GetBSide()->size(), 4);

  EXPECT_EQ(rg->num_junctions(), 1);
  const api::Junction* junction = rg->junction(0);
  EXPECT_EQ(junction->id(), api::JunctionId("j:all"));

  EXPECT_EQ(junction->num_segments(), 4);
  for (int si = 0; si < junction->num_segments(); ++si) {
    const api::Segment* segment = junction->segment(si);
    EXPECT_EQ(segment->num_lanes(), kOneLane);
    const api::Lane* lane = segment->lane(0);

    if (lane->id() == api::LaneId("l:left1_0")) {
      EXPECT_EQ(lane->segment()->id(), api::SegmentId("s:left1"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->lane->id(),
                api::LaneId("l:left1_0"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->end,
                api::LaneEnd::kFinish);
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->lane->id(),
                api::LaneId("l:left1_0"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->end,
                api::LaneEnd::kStart);
    } else if (lane->id() == api::LaneId("l:left0_0")) {
      EXPECT_EQ(lane->segment()->id(), api::SegmentId("s:left0"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->lane->id(),
                api::LaneId("l:right1_0"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->end,
                api::LaneEnd::kStart);
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->lane->id(),
                api::LaneId("l:right0_0"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->end,
                api::LaneEnd::kStart);
    } else if (lane->id() == api::LaneId("l:right0_0")) {
      EXPECT_EQ(lane->segment()->id(), api::SegmentId("s:right0"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->lane->id(),
                api::LaneId("l:right1_0"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->end,
                api::LaneEnd::kStart);
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->lane->id(),
                api::LaneId("l:right1_0"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->end,
                api::LaneEnd::kFinish);
    } else if (lane->id() == api::LaneId("l:right1_0")) {
      EXPECT_EQ(lane->segment()->id(), api::SegmentId("s:right1"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->lane->id(),
                api::LaneId("l:left1_0"));
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->end,
                api::LaneEnd::kFinish);
      EXPECT_FALSE(lane->GetDefaultBranch(api::LaneEnd::kFinish));
    } else {
      GTEST_FAIL();
    }
  }
};

// Holds Lane IDs for each Lane, both at the start and end of the Lane.
struct BranchPointLaneIds {
  std::vector<std::string> start_a_side;
  std::vector<std::string> start_b_side;
  std::vector<std::string> finish_a_side;
  std::vector<std::string> finish_b_side;
};

class MultilaneBuilderPrimitivesTest : public ::testing::Test {
 protected:
  const double kR0{10.};
  const double kLaneWidth{4.};
  const double kLeftShoulder{2.};
  const double kRightShoulder{2.};
  const api::HBounds kElevationBounds{0., 5.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.01 * M_PI};
  const int kNumLanes{3};
  const EndpointZ kLowFlatZ{0., 0., 0., 0.};
  const double kStartHeading{-M_PI / 4.};
  const Endpoint start{{0., 0., kStartHeading}, kLowFlatZ};
};

// Checks that a multi-lane line segment is correctly created.
TEST_F(MultilaneBuilderPrimitivesTest, MultilaneLineSegment) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance);
  const double kLength = 50.;
  b.Connect("c0", kNumLanes, kR0, kLeftShoulder, kRightShoulder, start, kLength,
            kLowFlatZ);
  std::unique_ptr<const api::RoadGeometry> rg =
      b.Build(api::RoadGeometryId{"multilane-line-segment"});

  EXPECT_EQ(rg->id(), api::RoadGeometryId("multilane-line-segment"));
  EXPECT_EQ(rg->num_junctions(), 1);
  EXPECT_NE(rg->junction(0), nullptr);
  EXPECT_EQ(rg->junction(0)->id(), api::JunctionId("j:c0"));
  EXPECT_EQ(rg->junction(0)->num_segments(), 1);
  EXPECT_NE(rg->junction(0)->segment(0), nullptr);
  EXPECT_EQ(rg->junction(0)->segment(0)->id(), api::SegmentId("s:c0"));
  EXPECT_EQ(rg->junction(0)->segment(0)->num_lanes(), kNumLanes);

  const Vector3<double> r_versor(-std::sin(kStartHeading),
                                 std::cos(kStartHeading), 0.);
  const Vector3<double> start_reference_curve(start.xy().x(), start.xy().y(),
                                              start.z().z());
  for (int i = 0; i < kNumLanes; i++) {
    const api::Lane* lane = rg->junction(0)->segment(0)->lane(i);
    // Checks Lane's ID.
    EXPECT_EQ(lane->id().string(), std::string("l:c0_") + std::to_string(i));
    // Checks lane start geo position to verify that spacing is correctly
    // applied.
    const Vector3<double> lane_start_geo =
        start_reference_curve +
        (kR0 + static_cast<double>(i) * kLaneWidth) * r_versor;
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition({0., 0., 0.}),
        api::GeoPosition::FromXyz(lane_start_geo), kLinearTolerance));
    // Checks lane start and end BranchPoint.
    const api::BranchPoint* const start_bp =
        lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_EQ(start_bp->GetASide()->size(), 1);
    EXPECT_EQ(start_bp->GetASide()->get(0).lane, lane);
    EXPECT_EQ(start_bp->GetBSide()->size(), 0);
    const api::BranchPoint* const end_bp =
        lane->GetBranchPoint(api::LaneEnd::kFinish);
    EXPECT_EQ(end_bp->GetASide()->size(), 1);
    EXPECT_EQ(end_bp->GetASide()->get(0).lane, lane);
    EXPECT_EQ(end_bp->GetBSide()->size(), 0);
  }
  // Checks the number of branch points.
  EXPECT_EQ(rg->num_branch_points(), 2 * kNumLanes);
}

// Checks that a multi-lane arc segment is correctly created.
TEST_F(MultilaneBuilderPrimitivesTest, MultilaneArcSegment) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance);
  const double kRadius = 30.;
  const double kDTheta = 0.5 * M_PI;
  b.Connect("c0", kNumLanes, kR0, kLeftShoulder, kRightShoulder, start,
            ArcOffset(kRadius, kDTheta), kLowFlatZ);
  std::unique_ptr<const api::RoadGeometry> rg =
      b.Build(api::RoadGeometryId{"multilane-arc-segment"});

  EXPECT_EQ(rg->id(), api::RoadGeometryId("multilane-arc-segment"));
  EXPECT_EQ(rg->num_junctions(), 1);
  EXPECT_NE(rg->junction(0), nullptr);
  EXPECT_EQ(rg->junction(0)->id(), api::JunctionId("j:c0"));
  EXPECT_EQ(rg->junction(0)->num_segments(), 1);
  EXPECT_NE(rg->junction(0)->segment(0), nullptr);
  EXPECT_EQ(rg->junction(0)->segment(0)->id(), api::SegmentId("s:c0"));
  EXPECT_EQ(rg->junction(0)->segment(0)->num_lanes(), kNumLanes);

  const Vector3<double> r_versor(-std::sin(kStartHeading),
                                 std::cos(kStartHeading), 0.);
  const Vector3<double> start_reference_curve(start.xy().x(), start.xy().y(),
                                              start.z().z());
  for (int i = 0; i < kNumLanes; i++) {
    const api::Lane* const lane = rg->junction(0)->segment(0)->lane(i);
    // Checks Lane's ID.
    EXPECT_EQ(lane->id().string(), std::string("l:c0_") + std::to_string(i));
    // Checks lane start geo position to verify that spacing is correctly
    // applied.
    const Vector3<double> lane_start_geo =
        start_reference_curve +
        (kR0 + static_cast<double>(i) * kLaneWidth) * r_versor;
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition({0., 0., 0.}),
        api::GeoPosition::FromXyz(lane_start_geo), kLinearTolerance));
    // Checks lane start and end BranchPoint.
    const api::BranchPoint* const start_bp =
        lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_EQ(start_bp->GetASide()->size(), 1);
    EXPECT_EQ(start_bp->GetASide()->get(0).lane, lane);
    EXPECT_EQ(start_bp->GetBSide()->size(), 0);
    const api::BranchPoint* const end_bp =
        lane->GetBranchPoint(api::LaneEnd::kFinish);
    EXPECT_EQ(end_bp->GetASide()->size(), 1);
    EXPECT_EQ(end_bp->GetASide()->get(0).lane, lane);
    EXPECT_EQ(end_bp->GetBSide()->size(), 0);
  }
  // Checks the number of branch points.
  EXPECT_EQ(rg->num_branch_points(), 2 * kNumLanes);
}

// Checks that Junctions, Segments, Lanes and BranchPoints are correctly made
// after creating the following RoadGeometry.
// The following graph shows a flat cross connection. 'cX' is used to name the
// connections that later are used in code. 'x' characters are used to locate
// BranchPoints. '0', '1' and '2' mean Lane indexes. Finally, '-', '|' and '\'
// are used to draw Lane centerlines.
// <pre>
//
//                                0 1
//                        d       x x
//                                | | c5
//                                | |
//                                | |
//                                | |
//                                | |
//                        e       f x
//                              c7| |\\ c6
//   c1                    c2     |\| \\   c3
// 2 x--------------------x---\---|-|--\\x------------------x 1
// 1 x--------------------x----\--|-|-\\-x------------------x 0
// 0 a--------------------b-- \ \ | |    c
//                         c4\ \ \| |
//                            \ \ |\|
//                        g     x x x
//                              | | | c8
//                              | | |
//                              | | |
//                              | | |
//                              | | |
//                              | | |
//                              | | |
//                              x x x
//                              0 1 2
// </pre>
GTEST_TEST(MultilaneBuilderTest, MultilaneCross) {
  const double kLaneWidth{4.};
  const double kLeftShoulder{1.};
  const double kRightShoulder{1.};
  const api::HBounds kElevationBounds{0., 5.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.01 * M_PI};
  const int kTwoLanes{2};
  const int kThreeLanes{3};
  const EndpointZ kLowFlatZ{0., 0., 0., 0.};

  const Endpoint endpoint_a{{0., 0., 0.}, kLowFlatZ};
  const Endpoint endpoint_b{{50., 0., 0.}, kLowFlatZ};
  const Endpoint endpoint_c{{70., 0., 0.}, kLowFlatZ};
  const Endpoint endpoint_d{{50., 50., -M_PI / 2.}, kLowFlatZ};
  const Endpoint endpoint_e{{50., 14., -M_PI / 2.}, kLowFlatZ};
  const Endpoint endpoint_f{{60., 14., -M_PI / 2.}, kLowFlatZ};
  const Endpoint endpoint_g{{50., -6., -M_PI / 2.}, kLowFlatZ};

  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance);
  // Creates connections.
  b.Connect("c1", kThreeLanes, 0, kLeftShoulder, kRightShoulder, endpoint_a,
            50., kLowFlatZ);
  auto c2 = b.Connect("c2", kTwoLanes, 4., kLeftShoulder, kRightShoulder,
                      endpoint_b, 20., kLowFlatZ);
  b.Connect("c3", kTwoLanes, 4., kLeftShoulder, kRightShoulder, endpoint_c, 30.,
            kLowFlatZ);
  auto c4 = b.Connect("c4", kThreeLanes, 0., kLeftShoulder, kRightShoulder,
                      endpoint_b, ArcOffset(6., -M_PI / 2.), kLowFlatZ);
  b.Connect("c5", kTwoLanes, 10., kLeftShoulder, kRightShoulder, endpoint_d,
            36., kLowFlatZ);
  auto c6 = b.Connect("c6", kTwoLanes, 0., kLeftShoulder, kRightShoulder,
                      endpoint_f, ArcOffset(10., M_PI / 2.), kLowFlatZ);
  auto c7 = b.Connect("c7", kTwoLanes, 10., kLeftShoulder, kRightShoulder,
                      endpoint_e, 20., kLowFlatZ);
  b.Connect("c8", kThreeLanes, 6., kLeftShoulder, kRightShoulder, endpoint_g,
            44., kLowFlatZ);
  // Creates the crossing junction.
  std::vector<const Connection*> connections{c2, c4, c6, c7};
  b.MakeGroup("cross", connections);

  std::unique_ptr<const api::RoadGeometry> rg =
      b.Build(api::RoadGeometryId{"multilane-arc-segment"});

  EXPECT_EQ(rg->id(), api::RoadGeometryId("multilane-arc-segment"));

  // Junction ID --> Segment IDs.
  const std::map<std::string, std::vector<std::string>> junction_truth_map{
      {"j:c1", {"s:c1"}},
      {"j:c3", {"s:c3"}},
      {"j:c5", {"s:c5"}},
      {"j:c8", {"s:c8"}},
      {"j:cross", {"s:c2", "s:c4", "s:c6", "s:c7"}}};
  // Segment ID --> Lane IDs.
  const std::map<std::string, std::vector<std::string>> segment_truth_map{
      {"s:c1", {"l:c1_0", "l:c1_1", "l:c1_2"}},
      {"s:c2", {"l:c2_0", "l:c2_1"}},
      {"s:c3", {"l:c3_0", "l:c3_1"}},
      {"s:c4", {"l:c4_0", "l:c4_1", "l:c4_2"}},
      {"s:c5", {"l:c5_0", "l:c5_1"}},
      {"s:c6", {"l:c6_0", "l:c6_1"}},
      {"s:c7", {"l:c7_0", "l:c7_1"}},
      {"s:c8", {"l:c8_0", "l:c8_1", "l:c8_2"}}};
  // Lane ID --> {Start_A_Side_Lane_IDs, Start_B_Side_Lane_IDs,
  //              Finish_A_Side_Lane_IDs, Finish_B_Side_Lane_IDs}.
  const std::map<std::string, BranchPointLaneIds> lane_truth_map{
      {"l:c1_0", {{"l:c1_0"}, {}, {"l:c4_0"}, {"l:c1_0"}}},
      {"l:c1_1", {{"l:c1_1"}, {}, {"l:c2_0", "l:c4_1"}, {"l:c1_1"}}},
      {"l:c1_2", {{"l:c1_2"}, {}, {"l:c2_1", "l:c4_2"}, {"l:c1_2"}}},
      {"l:c2_0",
       {{"l:c2_0", "l:c4_1"}, {"l:c1_1"}, {"l:c2_0", "l:c6_0"}, {"l:c3_0"}}},
      {"l:c2_1",
       {{"l:c2_1", "l:c4_2"}, {"l:c1_2"}, {"l:c2_1", "l:c6_1"}, {"l:c3_1"}}},
      {"l:c3_0", {{"l:c2_0", "l:c6_0"}, {"l:c3_0"}, {"l:c3_0"}, {}}},
      {"l:c3_1", {{"l:c2_1", "l:c6_1"}, {"l:c3_1"}, {"l:c3_1"}, {}}},
      {"l:c4_0", {{"l:c4_0"}, {"l:c1_0"}, {"l:c4_0"}, {"l:c8_0"}}},
      {"l:c4_1",
       {{"l:c2_0", "l:c4_1"}, {"l:c1_1"}, {"l:c4_1", "l:c7_0"}, {"l:c8_1"}}},
      {"l:c4_2",
       {{"l:c2_1", "l:c4_2"}, {"l:c1_2"}, {"l:c4_2", "l:c7_1"}, {"l:c8_2"}}},
      {"l:c5_0", {{"l:c5_0"}, {}, {"l:c6_0", "l:c7_0"}, {"l:c5_0"}}},
      {"l:c5_1", {{"l:c5_1"}, {}, {"l:c6_1", "l:c7_1"}, {"l:c5_1"}}},
      {"l:c6_0",
       {{"l:c6_0", "l:c7_0"}, {"l:c5_0"}, {"l:c2_0", "l:c6_0"}, {"l:c3_0"}}},
      {"l:c6_1",
       {{"l:c6_1", "l:c7_1"}, {"l:c5_1"}, {"l:c2_1", "l:c6_1"}, {"l:c3_1"}}},
      {"l:c7_0",
       {{"l:c6_0", "l:c7_0"}, {"l:c5_0"}, {"l:c4_1", "l:c7_0"}, {"l:c8_1"}}},
      {"l:c7_1",
       {{"l:c6_1", "l:c7_1"}, {"l:c5_1"}, {"l:c4_2", "l:c7_1"}, {"l:c8_2"}}},
      {"l:c8_0", {{"l:c4_0"}, {"l:c8_0"}, {"l:c8_0"}, {}}},
      {"l:c8_1", {{"l:c4_1", "l:c7_0"}, {"l:c8_1"}, {"l:c8_1"}, {}}},
      {"l:c8_2", {{"l:c4_2", "l:c7_1"}, {"l:c8_2"}, {"l:c8_2"}, {}}}};

  EXPECT_EQ(rg->num_junctions(), junction_truth_map.size());
  for (int i = 0; i < rg->num_junctions(); i++) {
    // Checks each Junction.
    const api::Junction* const junction = rg->junction(i);
    EXPECT_NE(junction_truth_map.find(junction->id().string()),
              junction_truth_map.end());
    const std::vector<std::string>& segment_ids =
        junction_truth_map.at(junction->id().string());
    EXPECT_EQ(segment_ids.size(), junction->num_segments());
    for (int j = 0; j < junction->num_segments(); j++) {
      // Checks each Segment.
      const api::Segment* const segment = junction->segment(j);
      EXPECT_EQ(segment->id().string(), segment_ids[j]);
      EXPECT_NE(segment_truth_map.find(segment->id().string()),
                segment_truth_map.end());
      const std::vector<std::string>& lane_ids =
          segment_truth_map.at(segment->id().string());
      EXPECT_EQ(segment->num_lanes(), lane_ids.size());
      for (int k = 0; k < segment->num_lanes(); k++) {
        // Checks each Lane.
        const api::Lane* const lane = segment->lane(k);
        EXPECT_NE(lane_truth_map.find(lane->id().string()),
                  lane_truth_map.end());
        const BranchPointLaneIds& bp_lane_ids =
            lane_truth_map.at(lane->id().string());
        const api::BranchPoint* const start_bp =
            lane->GetBranchPoint(api::LaneEnd::kStart);
        EXPECT_EQ(start_bp->GetASide()->size(),
                  bp_lane_ids.start_a_side.size());
        for (int lane_index = 0; lane_index < start_bp->GetASide()->size();
             lane_index++) {
          EXPECT_EQ(start_bp->GetASide()->get(lane_index).lane->id().string(),
                    bp_lane_ids.start_a_side[lane_index]);
        }
        EXPECT_EQ(start_bp->GetBSide()->size(),
                  bp_lane_ids.start_b_side.size());
        for (int lane_index = 0; lane_index < start_bp->GetBSide()->size();
             lane_index++) {
          EXPECT_EQ(start_bp->GetBSide()->get(lane_index).lane->id().string(),
                    bp_lane_ids.start_b_side[lane_index]);
        }
        const api::BranchPoint* const end_bp =
            lane->GetBranchPoint(api::LaneEnd::kFinish);
        EXPECT_EQ(end_bp->GetASide()->size(), bp_lane_ids.finish_a_side.size());
        for (int lane_index = 0; lane_index < end_bp->GetASide()->size();
             lane_index++) {
          EXPECT_EQ(end_bp->GetASide()->get(lane_index).lane->id().string(),
                    bp_lane_ids.finish_a_side[lane_index]);
        }
        EXPECT_EQ(end_bp->GetBSide()->size(), bp_lane_ids.finish_b_side.size());
        for (int lane_index = 0; lane_index < end_bp->GetBSide()->size();
             lane_index++) {
          EXPECT_EQ(end_bp->GetBSide()->get(lane_index).lane->id().string(),
                    bp_lane_ids.finish_b_side[lane_index]);
        }
      }
    }
  }
  EXPECT_EQ(rg->num_branch_points(), 20);
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
