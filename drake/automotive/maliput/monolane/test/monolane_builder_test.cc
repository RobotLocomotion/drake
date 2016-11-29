#include "drake/automotive/maliput/monolane/builder.h"

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"

namespace drake {
namespace maliput {
namespace monolane {

const double kPositionPrecision = 1e-2;
const double kVeryExact = 1e-7;

GTEST_TEST(MonolaneBuilderTest, Fig8) {
  const double kPosPrecision = 0.01;
  const double kOriPrecision = 0.01 * M_PI;
  Builder b({-2., 2.}, {-4., 4.}, kPosPrecision, kOriPrecision);

  XYZPoint start {{0., 0., -M_PI / 4.}, {0., 0., 0., 0.}};
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

  b.Connect("6", c6->end(),
            50.,
            c0->start());

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
  for (int b = 0; b < rg->num_branch_points(); ++b) {
    const api::BranchPoint* bp = rg->branch_point(b);
    /////    EXPECT_EQ(bp->GetBranches()->size(), 2);
    EXPECT_EQ(bp->GetASide()->size(), 1);
    EXPECT_EQ(bp->GetBSide()->size(), 1);
  }
};


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
