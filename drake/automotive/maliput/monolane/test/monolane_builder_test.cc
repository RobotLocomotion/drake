/* clang-format off */
#include "drake/automotive/maliput/monolane/builder.h"
/* clang-format on */

#include <cmath>
#include <iostream>

#include <gtest/gtest.h>

namespace drake {
namespace maliput {
namespace monolane {

GTEST_TEST(MonolaneBuilderTest, Fig8) {
  const api::RBounds kLaneBounds(-2., 2.);
  const api::RBounds kDriveableBounds(-4., 4.);
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  Builder b(kLaneBounds, kDriveableBounds, kLinearTolerance, kAngularTolerance);

  const EndpointZ kLowFlatZ(0., 0., 0., 0.);
  const EndpointZ kMidFlatZ(3., 0., 0., 0.);
  const EndpointZ kMidTiltLeftZ(3., 0., -0.4, 0.);
  const EndpointZ kMidTiltRightZ(3., 0., 0.4, 0.);
  const EndpointZ kHighFlatZ(6., 0., 0., 0.);

  const ArcOffset kCounterClockwiseArc(50., 0.75 * M_PI);  // 135deg, 50m radius
  const ArcOffset kClockwiseArc(50., -0.75 * M_PI);  // 135deg, 50m radius

  Endpoint start {{0., 0., -M_PI / 4.}, kLowFlatZ};
  auto c0 = b.Connect("0", start,
                      50., kMidFlatZ);

  auto c1 = b.Connect("1", c0->end(), kCounterClockwiseArc, kMidTiltLeftZ);
  auto c2 = b.Connect("2", c1->end(), kCounterClockwiseArc, kMidFlatZ);

  auto c3 = b.Connect("3", c2->end(), 50., kHighFlatZ);
  auto c4 = b.Connect("4", c3->end(), 50., kMidFlatZ);

  auto c5 = b.Connect("5", c4->end(), kClockwiseArc, kMidTiltRightZ);
  auto c6 = b.Connect("6", c5->end(), kClockwiseArc, kMidFlatZ);

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

  b.Connect("7", c6end, 50., c0start_z);

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"figure-eight"});

  EXPECT_EQ(rg->id().id, "figure-eight");

  EXPECT_EQ(rg->num_junctions(), 8);
  for (int j = 0; j < rg->num_junctions(); ++j) {
    const api::Junction* jnx = rg->junction(j);
    EXPECT_EQ(jnx->road_geometry(), rg.get());
    EXPECT_EQ(jnx->num_segments(), 1);

    const api::Segment* seg = jnx->segment(0);
    EXPECT_EQ(seg->junction(), jnx);
    EXPECT_EQ(seg->num_lanes(), 1);

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


GTEST_TEST(MonolaneBuilderTest, QuadRing) {
  const api::RBounds kLaneBounds(-2., 2.);
  const api::RBounds kDriveableBounds(-4., 4.);
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  Builder b(kLaneBounds, kDriveableBounds, kLinearTolerance, kAngularTolerance);

  const EndpointZ kFlatZ(0., 0., 0., 0.);
  const ArcOffset kLargeClockwiseLoop(150., -2. * M_PI);
  const ArcOffset kSmallClockwiseLoop(50., -2. * M_PI);
  const ArcOffset kLargeCounterClockwiseLoop(150., 2. * M_PI);
  const ArcOffset kSmallCounterClockwiseLoop(50., 2. * M_PI);

  Endpoint northbound {{0., 0., M_PI / 2.}, kFlatZ};

  // This heads -y, loops to -x, clockwise, back to origin.
  auto left1 = b.Connect("left1", northbound.reverse(),
                         kLargeClockwiseLoop, kFlatZ);
  // This heads +y, loops to -x, counterclockwise, back to origin.
  auto left0 = b.Connect("left0", northbound,
                         kSmallCounterClockwiseLoop, kFlatZ);
  // This heads +y, loops to +x, clockwise, back to origin.
  auto right0 = b.Connect("right0", northbound,
                          kSmallClockwiseLoop, kFlatZ);
  // This heads -y, loops to +x, counterclockwise, back to origin.
  auto right1 = b.Connect("right1", northbound.reverse(),
                          kLargeCounterClockwiseLoop, kFlatZ);

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
  b.SetDefaultBranch(
      left1, api::LaneEnd::kStart, left1, api::LaneEnd::kFinish);
  b.SetDefaultBranch(
      left0, api::LaneEnd::kFinish, right0, api::LaneEnd::kStart);
  b.SetDefaultBranch(
      right0, api::LaneEnd::kFinish, right1, api::LaneEnd::kFinish);
  b.SetDefaultBranch(
      right1, api::LaneEnd::kStart, left1, api::LaneEnd::kFinish);

  b.SetDefaultBranch(
      left1, api::LaneEnd::kFinish, left1, api::LaneEnd::kStart);
  b.SetDefaultBranch(
      left0, api::LaneEnd::kStart, right1, api::LaneEnd::kStart);
  b.SetDefaultBranch(
      right0, api::LaneEnd::kStart, right1, api::LaneEnd::kStart);
  // And, leave right1/finish without a default branch.

  b.MakeGroup("all", {right1, right0, left0, left1});

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"figure-eight"});

  EXPECT_EQ(rg->num_branch_points(), 1);
  EXPECT_EQ(rg->branch_point(0)->GetASide()->size(), 4);
  EXPECT_EQ(rg->branch_point(0)->GetBSide()->size(), 4);

  EXPECT_EQ(rg->num_junctions(), 1);
  const api::Junction* junction = rg->junction(0);
  EXPECT_EQ(junction->id().id, "j:all");

  EXPECT_EQ(junction->num_segments(), 4);
  for (int si = 0; si < junction->num_segments(); ++si) {
    const api::Segment* segment = junction->segment(si);
    EXPECT_EQ(segment->num_lanes(), 1);
    const api::Lane* lane = segment->lane(0);

    if (lane->id().id == "l:left1") {
      EXPECT_EQ(lane->segment()->id().id, "s:left1");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->lane->id().id,
                "l:left1");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->end,
                api::LaneEnd::kFinish);
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->lane->id().id,
                "l:left1");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->end,
                api::LaneEnd::kStart);
    } else if (lane->id().id == "l:left0") {
      EXPECT_EQ(lane->segment()->id().id, "s:left0");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->lane->id().id,
                "l:right1");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->end,
                api::LaneEnd::kStart);
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->lane->id().id,
                "l:right0");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->end,
                api::LaneEnd::kStart);
    } else if (lane->id().id == "l:right0") {
      EXPECT_EQ(lane->segment()->id().id, "s:right0");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->lane->id().id,
                "l:right1");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->end,
                api::LaneEnd::kStart);
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->lane->id().id,
                "l:right1");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish)->end,
                api::LaneEnd::kFinish);
    } else if (lane->id().id == "l:right1") {
      EXPECT_EQ(lane->segment()->id().id, "s:right1");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->lane->id().id,
                "l:left1");
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kStart)->end,
                api::LaneEnd::kFinish);
      EXPECT_EQ(lane->GetDefaultBranch(api::LaneEnd::kFinish).get(), nullptr);
    } else {
      GTEST_FAIL();
    }
  }
};


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
