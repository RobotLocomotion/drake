#include "drake/automotive/maliput/monolane/builder.h"

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"

namespace drake {
namespace maliput {
namespace monolane {

GTEST_TEST(MonolaneBuilderTest, Fig8) {
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  Builder b({-2., 2.}, {-4., 4.}, kLinearTolerance, kAngularTolerance);

  Endpoint start {{0., 0., -M_PI / 4.}, {0., 0., 0., 0.}};
  auto c0 = b.Connect("0", start,
                      50., {3., 0., 0., 0.});

  auto c1 = b.Connect("1", c0->end(),
                      ArcOffset(50., 0.75 * M_PI), {3., 0., 0.4, 0.});
  auto c2 = b.Connect("2", c1->end(),
                      ArcOffset(50., 0.75 * M_PI), {3., 0., 0., 0.});

  auto c3 = b.Connect("3", c2->end(),
                      50., {6., 0., 0., 0.});
  auto c4 = b.Connect("4", c3->end(),
                      50., {3., 0., 0., 0.});

  auto c5 = b.Connect("5", c4->end(),
                      ArcOffset(50., -0.75 * M_PI), {3., 0., -0.4, 0.});
  auto c6 = b.Connect("6", c5->end(),
                      ArcOffset(50., -0.75 * M_PI), {3., 0., 0., 0.});

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


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
