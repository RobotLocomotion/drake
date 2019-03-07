/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/builder.h"
/* clang-format on */

#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/test_utilities/check_id_indexing.h"
#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"
#include "drake/automotive/maliput/multilane/test_utilities/multilane_types_compare.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace {

using Which = api::LaneEnd::Which;

// StartReference::Spec using an Endpoint.
GTEST_TEST(StartReferenceSpecTest, Endpoint) {
  const Endpoint point{{1., 2., 3.}, {4., 5., 6., 7.}};
  constexpr double kEndpointLinearTolerance{1e-15};
  constexpr double kEndpointAngularTolerance{1e-15};

  const StartReference::Spec forward_dut =
      StartReference().at(point, Direction::kForward);
  EXPECT_TRUE(test::IsEndpointClose(forward_dut.endpoint(), point,
                                    kEndpointLinearTolerance,
                                    kEndpointAngularTolerance));

  const StartReference::Spec reversed_dut =
      StartReference().at(point, Direction::kReverse);
  EXPECT_TRUE(test::IsEndpointClose(reversed_dut.endpoint(), point.reverse(),
                                    kEndpointLinearTolerance,
                                    kEndpointAngularTolerance));
}

// StartReference::Spec using a connection's reference curve.
GTEST_TEST(StartReferenceSpecTest, Connection) {
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  const EndpointZ kFlatZ{0., 0., 0., 0.};
  const EndpointZ kFlatZWithoutThetaDot{0., 0., 0., {}};
  const EndpointXy kStartXy{1., 2., 3.};
  const LineOffset kLineOffset{10.};
  const EndpointXy kEndXy{1. + 10. * std::cos(3.), 2. + 10. * std::sin(3.), 3.};
  const Endpoint kStartEndpoint{kStartXy, kFlatZ};
  const Connection conn("conn", kStartEndpoint, kFlatZ, 2, 0., 1., 1.5, 1.5,
                        kLineOffset, kLinearTolerance, kScaleLength,
                        kComputationPolicy);
  constexpr double kEndpointLinearTolerance{1e-15};
  constexpr double kEndpointAngularTolerance{1e-15};

  const StartReference::Spec forward_start_dut =
      StartReference().at(conn, Which::kStart, Direction::kForward);
  const Endpoint expected_forward_start_endpoint{kStartXy,
                                                 kFlatZWithoutThetaDot};
  EXPECT_TRUE(test::IsEndpointClose(forward_start_dut.endpoint(),
                                    expected_forward_start_endpoint,
                                    kEndpointLinearTolerance,
                                    kEndpointAngularTolerance));

  const StartReference::Spec reversed_start_dut =
      StartReference().at(conn, Which::kStart, Direction::kReverse);
  const Endpoint expected_reversed_start_endpoint{
      kStartXy.reverse(), kFlatZWithoutThetaDot.reverse()};
  EXPECT_TRUE(test::IsEndpointClose(reversed_start_dut.endpoint(),
                                    expected_reversed_start_endpoint,
                                    kEndpointLinearTolerance,
                                    kEndpointAngularTolerance));

  const StartReference::Spec forward_end_dut =
      StartReference().at(conn, Which::kFinish, Direction::kForward);
  const Endpoint expected_forward_end_endpoint{kEndXy, kFlatZWithoutThetaDot};
  EXPECT_TRUE(test::IsEndpointClose(forward_end_dut.endpoint(),
                                    expected_forward_end_endpoint,
                                    kEndpointLinearTolerance,
                                    kEndpointAngularTolerance));

  const StartReference::Spec reversed_end_dut =
      StartReference().at(conn, Which::kFinish, Direction::kReverse);
  const Endpoint expected_reversed_end_endpoint{
      kEndXy.reverse(), kFlatZWithoutThetaDot.reverse()};
  EXPECT_TRUE(test::IsEndpointClose(reversed_end_dut.endpoint(),
                                    expected_reversed_end_endpoint,
                                    kEndpointLinearTolerance,
                                    kEndpointAngularTolerance));
}

// EndReference::Spec using an EndpointZ.
GTEST_TEST(EndReferenceSpecTest, Endpoint) {
  const EndpointZ z_point{4., 5., 6., 7.};
  constexpr double kEndpointLinearTolerance{1e-15};
  constexpr double kEndpointAngularTolerance{1e-15};

  const EndReference::Spec z_forward_dut =
      EndReference().z_at(z_point, Direction::kForward);
  EXPECT_TRUE(test::IsEndpointZClose(
      z_forward_dut.endpoint_z(), z_point,
      kEndpointLinearTolerance, kEndpointAngularTolerance));

  const EndReference::Spec z_reversed_dut =
      EndReference().z_at(z_point, Direction::kReverse);
  EXPECT_TRUE(test::IsEndpointZClose(
      z_reversed_dut.endpoint_z(), z_point.reverse(),
      kEndpointLinearTolerance, kEndpointAngularTolerance));
}

// EndReference::Spec using a connection's reference curve.
GTEST_TEST(EndReferenceSpecTest, Connection) {
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{
      ComputationPolicy::kPreferAccuracy};
  const EndpointZ kFlatZ{0., 0., 0., 0.};
  const EndpointXy kStartXy{1., 2., 3.};
  const Endpoint kStartEndpoint{kStartXy, kFlatZ};
  const EndpointZ kFlatZWithoutThetaDot{0., 0., 0., {}};
  const Connection conn("conn", kStartEndpoint, kFlatZ, 2, 0., 1., 1.5, 1.5,
                        LineOffset(10.), kLinearTolerance, kScaleLength,
                        kComputationPolicy);
  constexpr double kEndpointLinearTolerance{1e-15};
  constexpr double kEndpointAngularTolerance{1e-15};

  const EndReference::Spec forward_start_dut =
      EndReference().z_at(conn, Which::kStart, Direction::kForward);
  const EndpointZ expected_forward_start_endpoint{kFlatZWithoutThetaDot};
  EXPECT_TRUE(test::IsEndpointZClose(forward_start_dut.endpoint_z(),
                                     kFlatZWithoutThetaDot,
                                     kEndpointLinearTolerance,
                                     kEndpointAngularTolerance));

  const EndReference::Spec reversed_start_dut =
      EndReference().z_at(conn, Which::kStart, Direction::kReverse);
  EXPECT_TRUE(test::IsEndpointZClose(reversed_start_dut.endpoint_z(),
                                     kFlatZWithoutThetaDot.reverse(),
                                     kEndpointLinearTolerance,
                                     kEndpointAngularTolerance));

  const EndReference::Spec forward_end_dut =
      EndReference().z_at(conn, Which::kFinish, Direction::kForward);
  EXPECT_TRUE(test::IsEndpointZClose(forward_end_dut.endpoint_z(),
                                     kFlatZWithoutThetaDot,
                                     kEndpointLinearTolerance,
                                     kEndpointAngularTolerance));

  const EndReference::Spec reversed_end_dut =
      EndReference().z_at(conn, Which::kFinish, Direction::kReverse);
  EXPECT_TRUE(test::IsEndpointZClose(reversed_end_dut.endpoint_z(),
                                     kFlatZWithoutThetaDot.reverse(),
                                     kEndpointLinearTolerance,
                                     kEndpointAngularTolerance));
}

// LaneLayout check.
GTEST_TEST(LaneLayeoutTest, ConstructorAndAccessors) {
  const double kLeftShoulder{6.789};
  const double kRightShoulder{0.123};
  const int kNumLanes{4};
  const int kRefLane{2};
  const double kRefR0{-3.14};
  const LaneLayout dut(kLeftShoulder, kRightShoulder, kNumLanes, kRefLane,
                       kRefR0);

  EXPECT_EQ(dut.left_shoulder(), kLeftShoulder);
  EXPECT_EQ(dut.right_shoulder(), kRightShoulder);
  EXPECT_EQ(dut.num_lanes(), kNumLanes);
  EXPECT_EQ(dut.ref_lane(), kRefLane);
  EXPECT_EQ(dut.ref_r0(), kRefR0);
}

GTEST_TEST(MultilaneBuilderTest, ParameterConstructor) {
  const double kLaneWidth = 4.;
  const api::HBounds kElevationBounds(0., 5.);
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  const double kScaleLength = 1.0;
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  Builder builder(kLaneWidth, kElevationBounds, kLinearTolerance,
                  kAngularTolerance, kScaleLength, kComputationPolicy,
                  std::make_unique<GroupFactory>());
  EXPECT_EQ(builder.get_lane_width(), kLaneWidth);
  EXPECT_TRUE(api::test::IsHBoundsClose(builder.get_elevation_bounds(),
                                        kElevationBounds, 0.));
  EXPECT_EQ(builder.get_linear_tolerance(), kLinearTolerance);
  EXPECT_EQ(builder.get_angular_tolerance(), kAngularTolerance);
  EXPECT_EQ(builder.get_scale_length(), kScaleLength);
  EXPECT_EQ(builder.get_computation_policy(), kComputationPolicy);
}

// Checks that Connection instances are properly built by the Builder.
GTEST_TEST(MultilaneBuilderTest, ProperConnections) {
  constexpr double kLaneWidth = 4.;
  const api::HBounds kElevationBounds(0., 5.);
  constexpr double kLinearTolerance = 0.01;
  constexpr double kAngularTolerance = 0.01 * M_PI;
  constexpr double kEndpointLinearTolerance{1e-15};
  constexpr double kEndpointAngularTolerance{1e-15};
  constexpr double kScaleLength = 1.0;
  constexpr ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  Builder builder(kLaneWidth, kElevationBounds, kLinearTolerance,
                  kAngularTolerance, kScaleLength, kComputationPolicy,
                  std::make_unique<GroupFactory>());

  const double kLeftShoulder = 2.;
  const double kRightShoulder = 2.;
  const int kOneLane = 1;
  const int kRefLane = 0;
  const double kRefR0 = 0.;
  const LaneLayout kLaneLayout(kLeftShoulder, kRightShoulder,
                               kOneLane, kRefLane, kRefR0);
  const LineOffset kLineOffset(50.);
  const ArcOffset kArcOffset(50., -0.5 * M_PI);  // 90deg, 50m radius
  const EndpointZ kFlatZ(0., 0., 0., 0.);
  const Endpoint kStartEndpoint{{0., 0., 0.}, kFlatZ};

  const Connection* line_connection = builder.Connect(
      "line", kLaneLayout,
      StartReference().at(kStartEndpoint, Direction::kForward),
      kLineOffset, EndReference().z_at(kFlatZ, Direction::kForward));
  ASSERT_EQ(line_connection->type(), Connection::kLine);
  EXPECT_EQ(line_connection->id(), "line");
  EXPECT_EQ(line_connection->r0(), kRefR0);
  EXPECT_TRUE(test::IsEndpointClose(line_connection->start(),
                                    kStartEndpoint,
                                    kEndpointLinearTolerance,
                                    kEndpointAngularTolerance));
  EXPECT_EQ(line_connection->lane_width(), kLaneWidth);
  EXPECT_EQ(line_connection->left_shoulder(), kLeftShoulder);
  EXPECT_EQ(line_connection->right_shoulder(), kRightShoulder);
  EXPECT_EQ(line_connection->line_length(), kLineOffset.length());
  EXPECT_EQ(line_connection->linear_tolerance(), kLinearTolerance);
  EXPECT_EQ(line_connection->scale_length(), kScaleLength);
  EXPECT_EQ(line_connection->computation_policy(), kComputationPolicy);

  const Connection* arc_connection = builder.Connect(
      "arc", kLaneLayout, StartReference().at(
          *line_connection, Which::kFinish, Direction::kForward),
      kArcOffset, EndReference().z_at(kFlatZ, Direction::kForward));
  ASSERT_EQ(arc_connection->type(), Connection::kArc);
  EXPECT_EQ(arc_connection->id(), "arc");
  EXPECT_EQ(arc_connection->r0(), kRefR0);
  EXPECT_TRUE(test::IsEndpointClose(arc_connection->start(),
                                    line_connection->end(),
                                    kEndpointLinearTolerance,
                                    kEndpointAngularTolerance));
  EXPECT_EQ(arc_connection->lane_width(), kLaneWidth);
  EXPECT_EQ(arc_connection->left_shoulder(), kLeftShoulder);
  EXPECT_EQ(arc_connection->right_shoulder(), kRightShoulder);
  EXPECT_EQ(arc_connection->radius(), kArcOffset.radius());
  EXPECT_EQ(arc_connection->d_theta(), kArcOffset.d_theta());
  EXPECT_EQ(arc_connection->linear_tolerance(), kLinearTolerance);
  EXPECT_EQ(arc_connection->scale_length(), kScaleLength);
  EXPECT_EQ(arc_connection->computation_policy(), kComputationPolicy);
}


GTEST_TEST(MultilaneBuilderTest, Fig8) {
  const double kLaneWidth = 4.;
  const api::HBounds kElevationBounds(0., 5.);
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  const double kScaleLength = 1.0;
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());

  const double kLeftShoulder = 2.;
  const double kRightShoulder = 2.;
  const int kOneLane = 1;
  const int kRefLane = 0;
  const double kRefR0 = 0.;
  const LaneLayout kLaneLayout(kLeftShoulder, kRightShoulder, kOneLane,
                               kRefLane, kRefR0);

  const ArcOffset kCounterClockwiseArc(50., 0.75 * M_PI);  // 135deg, 50m radius
  const ArcOffset kClockwiseArc(50., -0.75 * M_PI);        // 135deg, 50m radius
  const LineOffset kLineOffset(50.);

  const EndpointZ kLowFlatZ(0., 0., 0., 0.);
  const EndpointZ kMidFlatZ(3., 0., 0., 0.);
  const EndpointZ kMidTiltLeftZ(3., 0., -0.4, 0.);
  const EndpointZ kMidTiltRightZ(3., 0., 0.4, 0.);
  const EndpointZ kHighFlatZ(6., 0., 0., 0.);
  const Endpoint kStartEndpoint{{0., 0., -M_PI / 4.}, kLowFlatZ};

  auto c0 = b.Connect("0", kLaneLayout,
                      StartReference().at(kStartEndpoint, Direction::kForward),
                      kLineOffset,
                      EndReference().z_at(kMidFlatZ, Direction::kForward));

  auto c1 =
      b.Connect("1", kLaneLayout,
                StartReference().at(*c0, Which::kFinish, Direction::kForward),
                kCounterClockwiseArc,
                EndReference().z_at(kMidTiltLeftZ, Direction::kForward));

  auto c2 =
      b.Connect("2", kLaneLayout,
                StartReference().at(*c1, Which::kFinish, Direction::kForward),
                kCounterClockwiseArc,
                EndReference().z_at(kMidFlatZ, Direction::kForward));

  auto c3 = b.Connect(
      "3", kLaneLayout,
      StartReference().at(*c2, Which::kFinish, Direction::kForward),
      kLineOffset, EndReference().z_at(kHighFlatZ, Direction::kForward));

  auto c4 = b.Connect(
      "4", kLaneLayout,
      StartReference().at(*c3, Which::kFinish, Direction::kForward),
      kLineOffset, EndReference().z_at(kMidTiltRightZ, Direction::kForward));

  auto c5 = b.Connect(
      "5", kLaneLayout,
      StartReference().at(*c4, Which::kFinish, Direction::kForward),
      kClockwiseArc, EndReference().z_at(kMidTiltRightZ, Direction::kForward));

  auto c6 = b.Connect(
      "6", kLaneLayout,
      StartReference().at(*c5, Which::kFinish, Direction::kForward),
      kClockwiseArc, EndReference().z_at(kMidFlatZ, Direction::kForward));

  // Tweak ends to check if fuzzy-matching is working.
  Endpoint c6end = c6->end();
  c6end = Endpoint(
      EndpointXy(c6end.xy().x() + kLinearTolerance * 0.5,
                 c6end.xy().y() - kLinearTolerance * 0.5, c6end.xy().heading()),
      EndpointZ(c6end.z().z() + kLinearTolerance * 0.5, c6end.z().z_dot(),
                c6end.z().theta(), *c6end.z().theta_dot()));

  EndpointZ c0start_z = c0->start().z();
  c0start_z =
      EndpointZ(c0start_z.z() - kLinearTolerance * 0.5, c0start_z.z_dot(),
                c0start_z.theta(), *c0start_z.theta_dot());

  b.Connect("7", kLaneLayout, StartReference().at(c6end, Direction::kForward),
            kLineOffset, EndReference().z_at(c0start_z, Direction::kForward));

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

  EXPECT_TRUE(api::test::CheckIdIndexing(rg.get()));
};


GTEST_TEST(MultilaneBuilderTest, QuadRing) {
  const double kLaneWidth = 4.;
  const api::HBounds kElevationBounds(0., 5.);
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  const double kScaleLength = 1.0;
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());

  const double kLeftShoulder = 2.;
  const double kRightShoulder = 2.;
  const int kOneLane = 1;
  const int kRefLane = 0;
  const double kRefR0 = 0.;
  const LaneLayout kLaneLayout(kLeftShoulder, kRightShoulder, kOneLane,
                               kRefLane, kRefR0);

  const ArcOffset kLargeClockwiseLoop(150., -2. * M_PI);
  const ArcOffset kSmallClockwiseLoop(50., -2. * M_PI);
  const ArcOffset kLargeCounterClockwiseLoop(150., 2. * M_PI);
  const ArcOffset kSmallCounterClockwiseLoop(50., 2. * M_PI);

  const EndpointZ kFlatZ(0., 0., 0., 0.);
  const Endpoint kNorthbound{{0., 0., M_PI / 2.}, kFlatZ};

  // This heads -y, loops to -x, clockwise, back to origin.
  auto left1 = b.Connect("left1", kLaneLayout,
                         StartReference().at(kNorthbound, Direction::kReverse),
                         kLargeClockwiseLoop,
                         EndReference().z_at(kFlatZ, Direction::kForward));
  // This heads +y, loops to -x, counterclockwise, back to origin.
  auto left0 = b.Connect("left0", kLaneLayout,
                         StartReference().at(kNorthbound, Direction::kForward),
                         kSmallCounterClockwiseLoop,
                         EndReference().z_at(kFlatZ, Direction::kForward));

  // This heads +y, loops to +x, clockwise, back to origin.
  auto right0 = b.Connect("right0", kLaneLayout,
                          StartReference().at(kNorthbound, Direction::kForward),
                          kSmallClockwiseLoop,
                          EndReference().z_at(kFlatZ, Direction::kForward));

  // This heads -y, loops to +x, counterclockwise, back to origin.
  auto right1 = b.Connect("right1", kLaneLayout,
                          StartReference().at(kNorthbound, Direction::kReverse),
                          kLargeCounterClockwiseLoop,
                          EndReference().z_at(kFlatZ, Direction::kForward));

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

  EXPECT_TRUE(api::test::CheckIdIndexing(rg.get()));
};


// Holds common properties for reference-to-reference curve primitive tests.
class MultilaneBuilderReferenceCurvePrimitivesTest : public ::testing::Test {
 protected:
  const double kLaneWidth{4.};
  const double kRefR0{10.};
  const double kLeftShoulder{2.};
  const double kRightShoulder{2.};
  const int kNumLanes{3};
  const int kRefLane{0};
  const LaneLayout kLaneLayout{kLeftShoulder, kRightShoulder, kNumLanes,
                               kRefLane, kRefR0};
  const api::HBounds kElevationBounds{0., 5.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.01 * M_PI};
  const double kScaleLength{1.0};
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  const EndpointZ kLowFlatZ{0., 0., 0., 0.};
  const double kStartHeading{-M_PI / 4.};
  const Endpoint kStart{{0., 0., kStartHeading}, kLowFlatZ};
};

// Checks that a multi-lane line segment is correctly created.
TEST_F(MultilaneBuilderReferenceCurvePrimitivesTest, LineSegment) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());

  const LineOffset kLineOffset(50.);
  b.Connect("c0", kLaneLayout, StartReference().at(kStart, Direction::kForward),
            kLineOffset, EndReference().z_at(kLowFlatZ, Direction::kForward));

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
  const Vector3<double> start_reference_curve(kStart.xy().x(), kStart.xy().y(),
                                              kStart.z().z());
  for (int i = 0; i < kNumLanes; i++) {
    const api::Lane* lane = rg->junction(0)->segment(0)->lane(i);
    // Checks Lane's ID.
    EXPECT_EQ(lane->id().string(), std::string("l:c0_") + std::to_string(i));
    // Checks lane start geo position to verify that spacing is correctly
    // applied.
    const Vector3<double> lane_start_geo =
        start_reference_curve +
        (kRefR0 + static_cast<double>(i) * kLaneWidth) * r_versor;
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
TEST_F(MultilaneBuilderReferenceCurvePrimitivesTest, ArcSegment) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());

  const double kRadius = 30.;
  const double kDTheta = 0.5 * M_PI;
  b.Connect("c0", kLaneLayout, StartReference().at(kStart, Direction::kForward),
            ArcOffset(kRadius, kDTheta),
            EndReference().z_at(kLowFlatZ, Direction::kForward));

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
  const Vector3<double> start_reference_curve(kStart.xy().x(), kStart.xy().y(),
                                              kStart.z().z());
  for (int i = 0; i < kNumLanes; i++) {
    const api::Lane* const lane = rg->junction(0)->segment(0)->lane(i);
    // Checks Lane's ID.
    EXPECT_EQ(lane->id().string(), std::string("l:c0_") + std::to_string(i));
    // Checks lane start geo position to verify that spacing is correctly
    // applied.
    const Vector3<double> lane_start_geo =
        start_reference_curve +
        (kRefR0 + static_cast<double>(i) * kLaneWidth) * r_versor;
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

// Holds common properties to create a Builder and Connections whose endpoint
// information lacks theta_dot so it is adjusted.
//
// For the tests below, let L be the curvature of the curve at the point so that
// L_line is the curvature for a line and L_arc is the curvature for an arc. In
// addition, let Δθ be the angle span of the arc offset.
// Then,
//
// L_line = 0
// L_arc = sign(Δθ) / radius
//
// And
//
// theta_dot = L * sin(-atan(z_dot))
//
// theta_dot constants in tests below were computed in Octave using previous
// expression.
class MultilaneBuilderPrimitiveContinuityConstraintTest
    : public ::testing::Test {
 protected:
  const double kLaneWidth{4.};
  const double kRefR0{0.};
  const double kLeftShoulder{2.};
  const double kRightShoulder{2.};
  const int kNumLanes{1};
  const int kRefLane{0};
  const LaneLayout kLaneLayout{kLeftShoulder, kRightShoulder, kNumLanes,
                               kRefLane, kRefR0};
  const api::HBounds kElevationBounds{0., 5.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.01 * M_PI};
  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{
      ComputationPolicy::kPreferAccuracy};
  const EndpointZ kStartZ{1., 2., M_PI / 6., {}};
  const double kStartHeading{-M_PI / 4.};
  const Endpoint kStartEndpoint{{0., 0., kStartHeading}, kStartZ};
  const EndpointZ kEndZ{4., 5., -M_PI / 6., {}};
};

// Checks how theta_dot is adjusted at the end points of the connection and set
// to zero always because of infinite curvature radius of a line.
TEST_F(MultilaneBuilderPrimitiveContinuityConstraintTest, MonolaneLineSegment) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());
  const LineOffset kLineOffset(50.);
  auto c0 =
      b.Connect("c0", kLaneLayout,
                StartReference().at(kStartEndpoint, Direction::kForward),
                kLineOffset, EndReference().z_at(kEndZ, Direction::kForward));
  EXPECT_NE(c0, nullptr);
  EXPECT_TRUE(test::IsEndpointClose(
      c0->start(), {kStartEndpoint.xy(), {1., 2., M_PI / 6., 0.}},
      kLinearTolerance, kAngularTolerance));
  EXPECT_TRUE(
      test::IsEndpointClose(c0->end(),
                            {{50. * std::cos(kStartHeading),
                              50. * std::sin(kStartHeading), kStartHeading},
                             {4., 5., -M_PI / 6., 0.}},
                            kLinearTolerance, kAngularTolerance));
}

// Checks how theta_dot is adjusted at the end points of the connection based
// on curvature and angular displacement.
TEST_F(MultilaneBuilderPrimitiveContinuityConstraintTest, MonolaneArcSegment) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());
  const double kRadius = 30.;
  const double kDTheta = 0.5 * M_PI;
  auto counter_clockwise_conn =
      b.Connect("counter_clockwise", kLaneLayout,
                StartReference().at(kStartEndpoint, Direction::kForward),
                ArcOffset(kRadius, kDTheta),
                EndReference().z_at(kEndZ, Direction::kForward));
  EXPECT_NE(counter_clockwise_conn, nullptr);
  EXPECT_TRUE(test::IsEndpointClose(
      counter_clockwise_conn->start(),
      {kStartEndpoint.xy(), {1., 2., M_PI / 6., -0.0298142396999972}},
      kLinearTolerance, kAngularTolerance));
  EXPECT_TRUE(test::IsEndpointClose(
      counter_clockwise_conn->end(),
      {{kRadius * std::sqrt(2.), 0., kStartHeading + kDTheta},
       {4., 5., -M_PI / 6., -0.0326860225230307}},
      kLinearTolerance, kAngularTolerance));

  auto clockwise_conn =
      b.Connect("clockwise", kLaneLayout,
                StartReference().at(kStartEndpoint, Direction::kForward),
                ArcOffset(kRadius, -kDTheta),
                EndReference().z_at(kEndZ, Direction::kForward));
  EXPECT_NE(clockwise_conn, nullptr);
  EXPECT_TRUE(test::IsEndpointClose(
      clockwise_conn->start(),
      {kStartEndpoint.xy(), {1., 2., M_PI / 6., 0.0298142396999972}},
      kLinearTolerance, kAngularTolerance));
  EXPECT_TRUE(test::IsEndpointClose(
      clockwise_conn->end(),
      {{0., -kRadius * std::sqrt(2.), kStartHeading - kDTheta},
       {4., 5., -M_PI / 6., 0.0326860225230307}},
      kLinearTolerance, kAngularTolerance));
}

// Holds common properties for lane-to-lane curve primitive tests.
class MultilaneBuilderLaneToLanePrimitivesTest : public ::testing::Test {
 protected:
  const double kLaneWidth{4.};
  const double kRefR0{10.};
  const double kLeftShoulder{2.};
  const double kRightShoulder{2.};
  const int kNumLanes{3};
  const int kRefLane{0};
  const int kStartLane{1};
  const int kEndLane{2};
  const LaneLayout kLaneLayout{kLeftShoulder, kRightShoulder, kNumLanes,
                               kRefLane, kRefR0};
  const api::HBounds kElevationBounds{0., 5.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.01 * M_PI};
  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{
      ComputationPolicy::kPreferAccuracy};
  const EndpointZ kLowFlatZ{0., 0., 0., {}};
  const EndpointZ kElevatedZ{5., 1., 0., {}};
  const double kStartHeading{-M_PI / 4.};
  const Endpoint kStart{{0., 0., kStartHeading}, kLowFlatZ};
};

// Checks that a multi-lane line segment is correctly created.
TEST_F(MultilaneBuilderLaneToLanePrimitivesTest, FlatLineSegment) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());

  const LineOffset kLineOffset(50.);
  b.Connect("c0", kLaneLayout,
            StartLane(kStartLane).at(kStart, Direction::kForward), kLineOffset,
            EndLane(kEndLane).z_at(kLowFlatZ, Direction::kForward));

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
  const Vector3<double> start_reference_curve =
      Vector3<double>(kStart.xy().x(), kStart.xy().y(), kStart.z().z()) +
      (kRefR0 - kLaneWidth) * r_versor;
  for (int i = 0; i < kNumLanes; i++) {
    const api::Lane* lane = rg->junction(0)->segment(0)->lane(i);
    // Checks Lane's ID.
    EXPECT_EQ(lane->id().string(), std::string("l:c0_") + std::to_string(i));
    // Checks lane start geo position to verify that spacing is correctly
    // applied.
    const Vector3<double> lane_start_geo =
        start_reference_curve +
        (-kRefR0 + static_cast<double>(i) * kLaneWidth) * r_versor;
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

// Checks that a multi-lane line segment is correctly created.
TEST_F(MultilaneBuilderLaneToLanePrimitivesTest, ElevatedEndLineSegment) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());

  const double kLength{50.};
  const LineOffset kLineOffset(kLength);
  b.Connect("c0", kLaneLayout,
            StartLane(kStartLane).at(kStart, Direction::kForward), kLineOffset,
            EndLane(kEndLane).z_at(kElevatedZ, Direction::kForward));

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
  const Vector3<double> start_reference_curve =
      Vector3<double>(kStart.xy().x(), kStart.xy().y(), kStart.z().z()) +
      (kRefR0 - kLaneWidth) * r_versor;
  const Vector3<double> end_reference_curve =
      start_reference_curve +
      kLength * Vector3<double>(std::cos(kStartHeading),
                                std::sin(kStartHeading), 0.) +
      Vector3<double>(0., 0., 5.);

  for (int i = 0; i < kNumLanes; i++) {
    const api::Lane* lane = rg->junction(0)->segment(0)->lane(i);
    // Checks Lane's ID.
    EXPECT_EQ(lane->id().string(), std::string("l:c0_") + std::to_string(i));
    // Checks lane start geo position to verify that spacing is correctly
    // applied.
    const Vector3<double> r_offset =
        (-kRefR0 + static_cast<double>(i) * kLaneWidth) * r_versor;
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition({0., 0., 0.}),
        api::GeoPosition::FromXyz(start_reference_curve + r_offset),
        kLinearTolerance));
    // Checks lane end geo position to verify elevation is correctly applied.
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition({lane->length(), 0., 0.}),
        api::GeoPosition::FromXyz(end_reference_curve + r_offset),
        kLinearTolerance));
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
TEST_F(MultilaneBuilderLaneToLanePrimitivesTest, ArcSegment) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());

  const double kRadius = 30.;
  const double kDTheta = 0.5 * M_PI;
  b.Connect("c0", kLaneLayout,
            StartLane(kStartLane).at(kStart, Direction::kForward),
            ArcOffset(kRadius, kDTheta),
            EndLane(kEndLane).z_at(kLowFlatZ, Direction::kForward));

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
  const Vector3<double> start_reference_curve =
      Vector3<double>(kStart.xy().x(), kStart.xy().y(), kStart.z().z()) +
      (kRefR0 - kLaneWidth) * r_versor;
  for (int i = 0; i < kNumLanes; i++) {
    const api::Lane* const lane = rg->junction(0)->segment(0)->lane(i);
    // Checks Lane's ID.
    EXPECT_EQ(lane->id().string(), std::string("l:c0_") + std::to_string(i));
    // Checks lane start geo position to verify that spacing is correctly
    // applied.
    const Vector3<double> lane_start_geo =
        start_reference_curve +
        (-kRefR0 + static_cast<double>(i) * kLaneWidth) * r_versor;
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
TEST_F(MultilaneBuilderLaneToLanePrimitivesTest, ElevatedEndArcSegment) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());

  const double kRadius = 30.;
  const double kDTheta = 0.5 * M_PI;
  b.Connect("c0", kLaneLayout,
            StartLane(kStartLane).at(kStart, Direction::kForward),
            ArcOffset(kRadius, kDTheta),
            EndLane(kEndLane).z_at(kElevatedZ, Direction::kForward));

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

  const Vector3<double> r_versor_start(-std::sin(kStartHeading),
                                       std::cos(kStartHeading), 0.);
  const Vector3<double> start_reference_curve =
      Vector3<double>(kStart.xy().x(), kStart.xy().y(), kStart.z().z()) +
      (kRefR0 - kLaneWidth) * r_versor_start;
  const Vector3<double> r_versor_end(-std::sin(kStartHeading + kDTheta),
                                     std::cos(kStartHeading + kDTheta), 0.);
  const Vector3<double> end_reference_curve =
      start_reference_curve + kRadius * Vector3<double>(std::sqrt(2.), 0., 0.) +
      Vector3<double>(0., 0., 5.);

  for (int i = 0; i < kNumLanes; i++) {
    const api::Lane* const lane = rg->junction(0)->segment(0)->lane(i);
    // Checks Lane's ID.
    EXPECT_EQ(lane->id().string(), std::string("l:c0_") + std::to_string(i));
    // Checks lane start geo position to verify that spacing is correctly
    // applied.
    const Vector3<double> r_offset_start =
        (-kRefR0 + static_cast<double>(i) * kLaneWidth) * r_versor_start;
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition({0., 0., 0.}),
        api::GeoPosition::FromXyz(start_reference_curve + r_offset_start),
        kLinearTolerance));
    // Checks lane end geo position to verify elevation is correctly applied.
    const Vector3<double> r_offset_end =
        (-kRefR0 + static_cast<double>(i) * kLaneWidth) * r_versor_end;
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition({lane->length(), 0., 0.}),
        api::GeoPosition::FromXyz(end_reference_curve + r_offset_end),
        kLinearTolerance));
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

namespace {

// An encapsulated Multilane road build procedure.
class BuildProcedure {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BuildProcedure);

  // Constructs a named build procedure.
  explicit BuildProcedure(const std::string& name)
      : name_(name) {
  }

  virtual ~BuildProcedure() = default;

  // Applies procedure to the given @p builder.
  virtual void ApplyTo(Builder* builder) const = 0;

  // Asserts that the procedure was properly applied
  // to the given @p road_geometry.
  virtual void AssertProperConstruction(
      const api::RoadGeometry& road_geometry) const = 0;

  // Returns procedure's name.
  const std::string& name() const { return name_; }

 private:
  // Procedure name.
  const std::string name_;
};

// An stream operator overload for shared references to
// BuildProcedure instances. Useful for gtest to print.
std::ostream& operator<<(
    std::ostream& os, const std::shared_ptr<BuildProcedure>& proc) {
  return os << proc->name();
}

// An encapsulated Multilane road turn build procedure.
class TurnBuildProcedure : public BuildProcedure {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TurnBuildProcedure);

  // Constructs a named build procedure for a turn.
  // @param name Name for the build procedure.
  // @param start_endpoint Turn's start endpoint.
  // @param straight_conn_length Turn straight section's length.
  // @param curved_conn_radius Turn curved section's arc radius.
  // @param curved_conn_angular_delta Turn curved section's arc
  //                                  subtended angle.
  explicit TurnBuildProcedure(
      const std::string& name, const Endpoint& start_endpoint,
      double straight_conn_length, double curved_conn_radius,
      double curved_conn_angular_delta)
      : BuildProcedure(name), start_endpoint_(start_endpoint),
        straight_conn_length_(straight_conn_length),
        curved_conn_radius_(curved_conn_radius),
        curved_conn_angular_delta_(curved_conn_angular_delta) {}

  void AssertProperConstruction(
      const api::RoadGeometry& road_geometry) const override {
    ASSERT_EQ(road_geometry.num_junctions(), 2);
    const api::Junction* straight_junction =
        road_geometry.ById().GetJunction(api::JunctionId{
            "j:" + std::string{kStraightConnName}});
    const api::Junction* curved_junction =
        road_geometry.ById().GetJunction(api::JunctionId{
            "j:" + std::string{kCurvedConnName}});
    ASSERT_EQ(curved_junction->num_segments(), 1);
    ASSERT_EQ(straight_junction->num_segments(), 1);
    const api::Segment* straight_segment = straight_junction->segment(0);
    const api::Segment* curved_segment = curved_junction->segment(0);
    ASSERT_EQ(curved_segment->num_lanes(), kLaneNum);
    ASSERT_EQ(straight_segment->num_lanes(), kLaneNum);
    const api::Lane* straight_lane = straight_segment->lane(0);
    const api::Lane* curved_lane = curved_segment->lane(kLaneNum - 1);

    const api::LaneEndSet* straight_lane_branches =
        straight_lane->GetOngoingBranches(api::LaneEnd::kStart);
    ASSERT_EQ(straight_lane_branches->size(), 1);
    ASSERT_EQ(straight_lane_branches->get(0).lane, curved_lane);
    ASSERT_EQ(straight_lane_branches->get(0).end, api::LaneEnd::kStart);
    const api::LaneEndSet* curved_lane_branches =
        curved_lane->GetOngoingBranches(api::LaneEnd::kStart);
    ASSERT_EQ(curved_lane_branches->size(), 1);
    ASSERT_EQ(curved_lane_branches->get(0).lane, straight_lane);
    ASSERT_EQ(curved_lane_branches->get(0).end, api::LaneEnd::kStart);

    const double kS{0.};
    const double kH{0.};
    const api::RBounds lbounds = straight_lane->lane_bounds(kS);
    const double lane_width = lbounds.max() - lbounds.min();
    for (double r = lbounds.min(); r <= lbounds.max(); r += lane_width/10) {
      // Since the curved lane heading at the origin is opposite to that of
      // the straight lane by construction, the r-offset sign is reversed.
      EXPECT_TRUE(api::test::IsGeoPositionClose(
          straight_lane->ToGeoPosition({kS, r, kH}),
          curved_lane->ToGeoPosition({kS, -r, kH}),
          road_geometry.linear_tolerance()))
          << "Position discontinuity at r = " << r;
      // Since the curved lane heading at the origin is opposite to that of
      // the straight lane by construction, the resulting orientation is
      // rotated pi radians about the h-axis.
      const api::Rotation rrotation = curved_lane->GetOrientation({kS, -r, kH});
      const Quaternion<double> pi_rotation(
          AngleAxis<double>(M_PI, Vector3<double>::UnitZ()));
      EXPECT_TRUE(api::test::IsRotationClose(
          straight_lane->GetOrientation({kS, r, kH}),
          // Applies a pi radians rotation around the h-axis to the curved
          // lane orientation (i.e. apply an intrinsic pi radians rotation
          // about the lane local z-axis, effectively post-multiplying).
          api::Rotation::FromQuat(rrotation.quat() * pi_rotation),
          road_geometry.angular_tolerance()))
          << " Orientation discontinuity at r = " << r;
    }
  }

 protected:
  // Returns straight connection line spec.
  LineOffset line_offset() const {
    return LineOffset(straight_conn_length_);
  }

  // Returns curved connection arc spec.
  ArcOffset arc_offset() const {
    return ArcOffset(curved_conn_radius_, curved_conn_angular_delta_);
  }

  // Returns turn start endpoint.
  const Endpoint& start_endpoint() const { return start_endpoint_; }

  const int kLaneNum{2};
  const double kLeftShoulder{2.};
  const double kRightShoulder{2.};
  const EndpointZ kFlatZ{0., 0., 0., {}};
  const std::string kStraightConnName{"straight"};
  const std::string kCurvedConnName{"curved"};

 private:
  const Endpoint start_endpoint_;
  const double straight_conn_length_;
  const double curved_conn_radius_;
  const double curved_conn_angular_delta_;
};

// An encapsulated Multilane turn build procedure that
// constructs it by specifying the straight connection by
// it's reference curve starting at the endpoint and the
// curved connection by reversing the former connection
// start endpoint.
class TurnUsingRefToRConn
    : public TurnBuildProcedure {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TurnUsingRefToRConn);
  using TurnBuildProcedure::TurnBuildProcedure;

  void ApplyTo(Builder* builder) const override {
    const int ref_lane{0};
    const double lane_width{builder->get_lane_width()};
    const double total_width{kLaneNum * lane_width};
    // Locates the reference curve along the centerline of
    // the Connection segment.
    const LaneLayout lane_layout{kLeftShoulder, kRightShoulder, kLaneNum,
                                 ref_lane, -(total_width - lane_width) / 2.};

    const Connection* straight_conn = builder->Connect(
        kStraightConnName, lane_layout,
        StartReference().at(start_endpoint(), Direction::kForward),
        line_offset(), EndReference().z_at(kFlatZ, Direction::kForward));
    ASSERT_TRUE(straight_conn != nullptr);

    const Connection* curved_conn = builder->Connect(
        kCurvedConnName, lane_layout, StartReference().at(
            *straight_conn, api::LaneEnd::kStart, Direction::kReverse),
        arc_offset(), EndReference().z_at(kFlatZ, Direction::kForward));
    ASSERT_TRUE(curved_conn != nullptr);
  }
};

// An encapsulated Multilane turn build procedure that
// constructs it by specifying the straight connection by
// it's reference curve starting at the endpoint and the
// curved connection by reversing the endpoint.
class TurnUsingRefToRRef
    : public TurnBuildProcedure {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TurnUsingRefToRRef);
  using TurnBuildProcedure::TurnBuildProcedure;

  void ApplyTo(Builder* builder) const override {
    const int ref_lane{0};
    const double lane_width{builder->get_lane_width()};
    const double total_width{kLaneNum * lane_width};
    // Locates the reference curve along the centerline of
    // the Connection segment.
    const LaneLayout lane_layout{kLeftShoulder, kRightShoulder, kLaneNum,
                                 ref_lane, -(total_width - lane_width) / 2.};

    const Connection* straight_conn = builder->Connect(
        kStraightConnName, lane_layout,
        StartReference().at(start_endpoint(), Direction::kForward),
        line_offset(), EndReference().z_at(kFlatZ, Direction::kForward));
    ASSERT_TRUE(straight_conn != nullptr);

    const Connection* curved_conn = builder->Connect(
        kCurvedConnName, lane_layout, StartReference().at(
            start_endpoint(), Direction::kReverse),
        arc_offset(), EndReference().z_at(kFlatZ, Direction::kForward));
    ASSERT_TRUE(curved_conn != nullptr);
  }
};

// An encapsulated Multilane turn build procedure that
// constructs it by specifying the straight connection by
// it's reference curve starting at the endpoint and the
// curved connection by reversing the former connection
// reference lane at its start end.
class TurnUsingStraightRefToCurvedRLane
    : public TurnBuildProcedure {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TurnUsingStraightRefToCurvedRLane);
  using TurnBuildProcedure::TurnBuildProcedure;

  void ApplyTo(Builder* builder) const override {
    const int ref_lane{0};
    const double lane_width{builder->get_lane_width()};
    const double total_width{kLaneNum * lane_width};
    // Locates the reference curve on the left (for an observer
    // looking in the direction of increasing s coordinates) of
    // the Connection segment, half a lane width away from its
    // centerline.
    const LaneLayout lane_layout{
      kLeftShoulder, kRightShoulder, kLaneNum,
      ref_lane, -(total_width + lane_width) / 2.};

    const Connection* straight_conn = builder->Connect(
        kStraightConnName, lane_layout,
        StartReference().at(start_endpoint(), Direction::kForward),
        line_offset(), EndReference().z_at(kFlatZ, Direction::kForward));
    ASSERT_TRUE(straight_conn != nullptr);

    const int other_lane{kLaneNum - 1};

    const Connection* curved_conn = builder->Connect(
        kCurvedConnName, lane_layout, StartLane(ref_lane).at(
            *straight_conn, other_lane, api::LaneEnd::kStart,
            Direction::kReverse), arc_offset(),
        EndLane(ref_lane).z_at(kFlatZ, Direction::kForward));
    ASSERT_TRUE(curved_conn != nullptr);
  }
};

// An encapsulated Multilane turn build procedure that
// constructs it by specifying the curved connection by
// it's reference curve starting at the endpoint and the
// straight connection by reversing the former connection
// reference lane at its start end.
class TurnUsingCurvedRefToStraightRLane
    : public TurnBuildProcedure {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TurnUsingCurvedRefToStraightRLane);
  using TurnBuildProcedure::TurnBuildProcedure;

  void ApplyTo(Builder* builder) const override {
    const int ref_lane{0};
    const double lane_width{builder->get_lane_width()};
    const double total_width{kLaneNum * lane_width};
    // Locates the reference curve on the left (for an observer
    // looking in the direction of increasing s coordinates) of
    // the Connection segment, half a lane width away from its
    // centerline.
    const LaneLayout lane_layout{
      kLeftShoulder, kRightShoulder, kLaneNum,
      ref_lane, -(total_width + lane_width) / 2.};

    const Connection* curved_conn = builder->Connect(
        kCurvedConnName, lane_layout,
        StartReference().at(start_endpoint(), Direction::kForward),
        arc_offset(), EndReference().z_at(kFlatZ, Direction::kForward));
    ASSERT_TRUE(curved_conn != nullptr);

    const int other_lane{kLaneNum - 1};

    const Connection* straight_conn = builder->Connect(
        kStraightConnName, lane_layout, StartLane(ref_lane).at(
            *curved_conn, other_lane, api::LaneEnd::kStart,
            Direction::kReverse), line_offset(),
        EndLane(ref_lane).z_at(kFlatZ, Direction::kForward));
    ASSERT_TRUE(straight_conn != nullptr);
  }
};


}  // namespace

using TestWithBuildProcedure =
    ::testing::TestWithParam<std::shared_ptr<BuildProcedure>>;

// A test fixture for Multilane build procedures.
class MultilaneBuildProcedureTest : public TestWithBuildProcedure {
 protected:
  // Applies provided build procedure param to the given @p builder.
  void ApplyProcedureTo(Builder* builder) {
    ASSERT_TRUE(GetParam() != nullptr);
    GetParam()->ApplyTo(builder);
  }

  // Asserts that the given @p road_geometry was properly built by
  // the provided build procedure param.
  void AssertProcedureResult(const api::RoadGeometry& road_geometry) {
    ASSERT_TRUE(GetParam() != nullptr);
    GetParam()->AssertProperConstruction(road_geometry);
  }

  const double kLaneWidth{4.};
  const api::HBounds kElevationBounds{0., 5.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.01 * M_PI};
  const double kScaleLength{1.0};
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
};

// Test fixture parameterized by BuildProcedure instances that are
// expected to produce G1 continuity issues. See Builder class
// documentation for further reference.
class MultilaneDiscontinuousBuildProcedureTest
    : public MultilaneBuildProcedureTest {
};

// Tests that the Builder refuses to issue G1 discontinuous road
// geometries.
TEST_P(MultilaneDiscontinuousBuildProcedureTest, ThrowingUponBuild) {
  Builder builder(kLaneWidth, kElevationBounds,
                  kLinearTolerance, kAngularTolerance,
                  kScaleLength, kComputationPolicy,
                  std::make_unique<GroupFactory>());
  ApplyProcedureTo(&builder);
  EXPECT_THROW({
      const std::unique_ptr<const api::RoadGeometry> road_geometry =
          builder.Build(api::RoadGeometryId{"bad_road"});
    },
    std::runtime_error);
}

// Returns a collection of BuildProcedure instances that make use
// of reversed semantics in a way that would introduce G1 continuity
// issues.
std::vector<std::shared_ptr<BuildProcedure>>
ListBadReverseBuildProcedures() {
  constexpr double kStraightConnLength{50.};
  constexpr double kCurvedConnRadius{50.};
  constexpr double kCurvedConnAngularDelta{M_PI/2.};
  const Endpoint kBadStartEndpoint{{0., 0., M_PI/2.}, {0., 0., 0.3, 0.1}};
  std::vector<std::shared_ptr<BuildProcedure>> procedures;
  procedures.push_back(
      std::make_unique<TurnUsingRefToRConn>(
          "BankedTurnUsingRefToRConnWithBadThetaDot",
          kBadStartEndpoint, kStraightConnLength,
          kCurvedConnRadius, kCurvedConnAngularDelta));
  procedures.push_back(
      std::make_unique<TurnUsingRefToRRef>(
          "BankedTurnUsingRefToRRefWithBadThetaDot",
          kBadStartEndpoint, kStraightConnLength,
          kCurvedConnRadius, kCurvedConnAngularDelta));
  procedures.push_back(
      std::make_unique<TurnUsingStraightRefToCurvedRLane>(
          "BankedTurnUsingStraightRefToCurvedRLaneWithBadThetaDot",
          kBadStartEndpoint, kStraightConnLength,
          kCurvedConnRadius, kCurvedConnAngularDelta));
  procedures.push_back(
      std::make_unique<TurnUsingCurvedRefToStraightRLane>(
          "BankedTurnUsingCurvedRefToStraightRLaneWithBadThetaDot",
          kBadStartEndpoint, kStraightConnLength,
          kCurvedConnRadius, kCurvedConnAngularDelta));
  return procedures;
}

INSTANTIATE_TEST_CASE_P(BadReverseConnectionTest,
                        MultilaneDiscontinuousBuildProcedureTest,
                        ::testing::ValuesIn(ListBadReverseBuildProcedures()));

// Test fixture parameterized by BuildProcedure instances, that
// are expected to produce G1 continuous roads.
class MultilaneContinuousBuildProcedureTest
    : public MultilaneBuildProcedureTest {
};

// Tests (exhaustively) that the Builder achieves G1 continuity.
TEST_P(MultilaneContinuousBuildProcedureTest, Continuity) {
  Builder builder(kLaneWidth, kElevationBounds,
                  kLinearTolerance, kAngularTolerance,
                  kScaleLength, kComputationPolicy,
                  std::make_unique<GroupFactory>());
  ApplyProcedureTo(&builder);
  const std::unique_ptr<const api::RoadGeometry> road_geometry =
      builder.Build(api::RoadGeometryId{"good_road"});
  AssertProcedureResult(*road_geometry);
}

// Returns a collection of BuildProcedure instances that make use
// of reversed semantics in a way that produces G1 continuous roads.
std::vector<std::shared_ptr<BuildProcedure>>
ListGoodReverseBuildProcedures() {
  constexpr double kStraightConnLength{50.};
  constexpr double kCurvedConnRadius{50.};
  constexpr double kCurvedConnAngularDelta{M_PI/2.};
  const Endpoint kStartEndpoint{{0., 0., M_PI/2.}, {0., 0., 0.3, {}}};
  std::vector<std::shared_ptr<BuildProcedure>> procedures;
  procedures.push_back(
      std::make_unique<TurnUsingRefToRConn>(
          "BankedTurnUsingRefToRConnWithoutThetaDot",
          kStartEndpoint, kStraightConnLength,
          kCurvedConnRadius, kCurvedConnAngularDelta));
  procedures.push_back(
      std::make_unique<TurnUsingRefToRRef>(
          "BankedTurnUsingRefToRRefWithoutThetaDot",
          kStartEndpoint, kStraightConnLength,
          kCurvedConnRadius, kCurvedConnAngularDelta));
  procedures.push_back(
      std::make_unique<TurnUsingStraightRefToCurvedRLane>(
          "BankedTurnUsingStraightRefToCurvedRLaneWithoutThetaDot",
          kStartEndpoint, kStraightConnLength,
          kCurvedConnRadius, kCurvedConnAngularDelta));
  procedures.push_back(
      std::make_unique<TurnUsingCurvedRefToStraightRLane>(
          "BankedTurnUsingCurvedRefToStraightRLaneWithoutThetaDot",
          kStartEndpoint, kStraightConnLength,
          kCurvedConnRadius, kCurvedConnAngularDelta));
  return procedures;
}

INSTANTIATE_TEST_CASE_P(GoodReverseConnectionTest,
                        MultilaneContinuousBuildProcedureTest,
                        ::testing::ValuesIn(ListGoodReverseBuildProcedures()));

// Holds Lane IDs for each Lane, both at the start and end of the Lane.
struct BranchPointLaneIds {
  std::vector<std::string> start_a_side;
  std::vector<std::string> start_b_side;
  std::vector<std::string> finish_a_side;
  std::vector<std::string> finish_b_side;
};

// Checks that Junctions, Segments, Lanes and BranchPoints are correctly made
// after creating the following RoadGeometry.
// The following graph shows a flat cross connection. 'cX' is used to name the
// connections that later are used in code. 'x' characters are used to locate
// BranchPoints. '0', '1' and '2' mean Lane indexes. Finally, '-', '|' and '\'
// are used to draw Lane centerlines.
// <pre>
//
//                                0 1
//                                d x
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
//
// Each of the test cases below build the same road geometry, but one uses only
// reference curve Build::Connect() methods and the other lane curve
// Build::Connection()
class MultilaneBuilderMultilaneCrossTest : public ::testing::Test {
 protected:
  const double kLaneWidth{4.};
  const double kLeftShoulder{1.};
  const double kRightShoulder{1.};
  const api::HBounds kElevationBounds{0., 5.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.01 * M_PI};
  const double kScaleLength{1.0};
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  const int kTwoLanes{2};
  const int kThreeLanes{3};
  const int kRefLane{0};
  const int kFirstLane{0};
  const int kSecondLane{1};
  const int kThirdLane{2};
  const EndpointZ kLowFlatZ{0., 0., 0., 0.};
  const EndpointZ kLowFlatZWithoutThetaDot{0., 0., 0., {}};

  const Endpoint endpoint_a{{0., 0., 0.}, kLowFlatZ};
  const Endpoint endpoint_a_without_theta_dot{{0., 0., 0.},
                                              kLowFlatZWithoutThetaDot};
  const Endpoint endpoint_b{{50., 0., 0.}, kLowFlatZ};
  const Endpoint endpoint_c{{70., 0., 0.}, kLowFlatZ};
  const Endpoint endpoint_d{{60., 50., -M_PI / 2.}, kLowFlatZ};
  const Endpoint endpoint_d_without_theta_dot{{60., 50., -M_PI / 2.},
                                              kLowFlatZWithoutThetaDot};
  const Endpoint endpoint_e{{50., 14., -M_PI / 2.}, kLowFlatZ};
  const Endpoint endpoint_f{{60., 14., -M_PI / 2.}, kLowFlatZ};
  const Endpoint endpoint_g{{50., -6., -M_PI / 2.}, kLowFlatZ};

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

  void BuildAndCheckRoadGeometry(Builder* b) {
    std::unique_ptr<const api::RoadGeometry> rg =
        b->Build(api::RoadGeometryId{"multilane-cross"});

    EXPECT_EQ(rg->id(), api::RoadGeometryId("multilane-cross"));

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
          EXPECT_EQ(end_bp->GetASide()->size(),
                    bp_lane_ids.finish_a_side.size());
          for (int lane_index = 0; lane_index < end_bp->GetASide()->size();
               lane_index++) {
            EXPECT_EQ(end_bp->GetASide()->get(lane_index).lane->id().string(),
                      bp_lane_ids.finish_a_side[lane_index]);
          }
          EXPECT_EQ(end_bp->GetBSide()->size(),
                    bp_lane_ids.finish_b_side.size());
          for (int lane_index = 0; lane_index < end_bp->GetBSide()->size();
               lane_index++) {
            EXPECT_EQ(end_bp->GetBSide()->get(lane_index).lane->id().string(),
                      bp_lane_ids.finish_b_side[lane_index]);
          }
        }
      }
    }
    EXPECT_EQ(rg->num_branch_points(), 20);

    EXPECT_TRUE(api::test::CheckIdIndexing(rg.get()));
  }
};

TEST_F(MultilaneBuilderMultilaneCrossTest, MultilaneRefCurveToRefCurve) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());
  // Creates connections.
  b.Connect(
      "c1",
      LaneLayout(kLeftShoulder, kRightShoulder, kThreeLanes, kRefLane, 0.),
      StartReference().at(endpoint_a, Direction::kForward), LineOffset(50.),
      EndReference().z_at(kLowFlatZ, Direction::kReverse));

  auto c2 = b.Connect(
      "c2", LaneLayout(kLeftShoulder, kRightShoulder, kTwoLanes, kRefLane, 4.),
      StartReference().at(endpoint_b, Direction::kForward), LineOffset((20.)),
      EndReference().z_at(kLowFlatZ, Direction::kReverse));

  b.Connect(
      "c3", LaneLayout(kLeftShoulder, kRightShoulder, kTwoLanes, kRefLane, 4.),
      StartReference().at(endpoint_c, Direction::kForward), LineOffset(30.),
      EndReference().z_at(kLowFlatZ, Direction::kReverse));

  auto c4 = b.Connect("c4", LaneLayout(kLeftShoulder, kRightShoulder,
                                       kThreeLanes, kRefLane, 0.),
                      StartReference().at(endpoint_b, Direction::kForward),
                      ArcOffset(6., -M_PI / 2.),
                      EndReference().z_at(kLowFlatZ, Direction::kReverse));

  b.Connect(
      "c5", LaneLayout(kLeftShoulder, kRightShoulder, kTwoLanes, kRefLane, 0.),
      StartReference().at(endpoint_d, Direction::kForward), LineOffset(36.),
      EndReference().z_at(kLowFlatZ, Direction::kReverse));

  auto c6 = b.Connect(
      "c6", LaneLayout(kLeftShoulder, kRightShoulder, kTwoLanes, kRefLane, 0.),
      StartReference().at(endpoint_f, Direction::kForward),
      ArcOffset(10., M_PI / 2.),
      EndReference().z_at(kLowFlatZ, Direction::kReverse));

  auto c7 = b.Connect(
      "c7", LaneLayout(kLeftShoulder, kRightShoulder, kTwoLanes, kRefLane, 10.),
      StartReference().at(endpoint_e, Direction::kForward), LineOffset(20.),
      EndReference().z_at(kLowFlatZ, Direction::kReverse));

  b.Connect(
      "c8",
      LaneLayout(kLeftShoulder, kRightShoulder, kThreeLanes, kRefLane, 6.),
      StartReference().at(endpoint_g, Direction::kForward), LineOffset(44.),
      EndReference().z_at(kLowFlatZ, Direction::kReverse));

  // Creates the crossing junction.
  std::vector<const Connection*> connections{c2, c4, c6, c7};
  b.MakeGroup("cross", connections);

  BuildAndCheckRoadGeometry(&b);
}

TEST_F(MultilaneBuilderMultilaneCrossTest, MultilaneLaneCurveToLaneCurve) {
  Builder b(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
            kScaleLength, kComputationPolicy, std::make_unique<GroupFactory>());

  // Creates connections.
  auto c1 = b.Connect(
      "c1",
      LaneLayout(kLeftShoulder, kRightShoulder, kThreeLanes, kRefLane, 0.),
      StartLane(kFirstLane)
          .at(endpoint_a_without_theta_dot, Direction::kForward),
      LineOffset(50.),
      EndLane(kFirstLane).z_at(kLowFlatZWithoutThetaDot, Direction::kReverse));

  auto c2 = b.Connect(
      "c2", LaneLayout(kLeftShoulder, kRightShoulder, kTwoLanes, kRefLane, 4.),
      StartLane(kFirstLane)
          .at(*c1, kSecondLane, Which::kFinish, Direction::kForward),
      LineOffset(20.),
      EndLane(kFirstLane).z_at(kLowFlatZWithoutThetaDot, Direction::kReverse));

  auto c3 = b.Connect(
      "c3", LaneLayout(kLeftShoulder, kRightShoulder, kTwoLanes, kRefLane, 4.),
      StartLane(kSecondLane)
          .at(*c2, kSecondLane, Which::kFinish, Direction::kForward),
      LineOffset(30.),
      EndLane(kFirstLane).z_at(kLowFlatZWithoutThetaDot, Direction::kReverse));

  auto c4 = b.Connect(
      "c4",
      LaneLayout(kLeftShoulder, kRightShoulder, kThreeLanes, kRefLane, 0.),
      StartLane(kSecondLane)
          .at(*c2, kFirstLane, Which::kStart, Direction::kForward),
      ArcOffset(6., -M_PI / 2.),
      EndLane(kThirdLane).z_at(kLowFlatZWithoutThetaDot, Direction::kReverse));

  auto c5 = b.Connect(
      "c5", LaneLayout(kLeftShoulder, kRightShoulder, kTwoLanes, kRefLane, 0.),
      StartLane(kFirstLane)
          .at(endpoint_d_without_theta_dot, Direction::kForward),
      LineOffset(36.),
      EndLane(kFirstLane).z_at(kLowFlatZWithoutThetaDot, Direction::kReverse));

  auto c6 = b.Connect(
      "c6", LaneLayout(kLeftShoulder, kRightShoulder, kTwoLanes, kRefLane, 0.),
      StartLane(kFirstLane)
          .at(*c5, kFirstLane, Which::kFinish, Direction::kForward),
      ArcOffset(10., M_PI / 2.),
      EndLane(kSecondLane)
          .z_at(*c3, kSecondLane, Which::kStart, Direction::kForward));

  auto c7 = b.Connect(
      "c7", LaneLayout(kLeftShoulder, kRightShoulder, kTwoLanes, kRefLane, 10.),
      StartLane(kFirstLane)
          .at(*c5, kFirstLane, Which::kFinish, Direction::kForward),
      LineOffset(20.),
      EndLane(kFirstLane)
          .z_at(*c4, kSecondLane, Which::kFinish, Direction::kForward));

  b.Connect(
      "c8",
      LaneLayout(kLeftShoulder, kRightShoulder, kThreeLanes, kRefLane, 6.),
      StartLane(kFirstLane)
          .at(*c4, kFirstLane, Which::kFinish, Direction::kForward),
      LineOffset(44.),
      EndLane(kThirdLane).z_at(kLowFlatZWithoutThetaDot, Direction::kReverse));

  // Creates the crossing junction.
  std::vector<const Connection*> connections{c2, c4, c6, c7};
  b.MakeGroup("cross", connections);

  BuildAndCheckRoadGeometry(&b);
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
