#include "drake/automotive/monolane_onramp_merge.h"

#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace automotive {
namespace {

namespace api = maliput::api;

const api::Lane* GetLaneByJunctionId(const api::RoadGeometry& rg,
                                     const std::string& junction_id) {
  for (int i = 0; i < rg.num_junctions(); ++i) {
    if (rg.junction(i)->id().id == junction_id) {
      return rg.junction(i)->segment(0)->lane(0);
    }
  }
  throw std::runtime_error("No matching junction name in the road network");
}

// Tests the soundness of the MonolaneOnrampMerge example generator.
GTEST_TEST(MonolaneOnrampMergeTest, TestDefaultAndNonDefaultAttributes) {
  const double kSPosition = 0.;

  // Create the road with default road characteristics.
  std::unique_ptr<MonolaneOnrampMerge> merge_example(new MonolaneOnrampMerge);
  std::unique_ptr<const maliput::api::RoadGeometry> rg =
      merge_example->BuildOnramp();
  EXPECT_NE(nullptr, rg);

  // Check the correctness of the default parameters.
  EXPECT_EQ(
      RoadCharacteristics{}.lane_bounds.r_max,
      rg->junction(0)->segment(0)->lane(0)->lane_bounds(kSPosition).r_max);
  EXPECT_EQ(
      RoadCharacteristics{}.driveable_bounds.r_max,
      rg->junction(0)->segment(0)->lane(0)->driveable_bounds(kSPosition).r_max);

  EXPECT_EQ(rg->id().id, "monolane-merge-example");
  EXPECT_EQ(rg->num_junctions(), 9);

  // Verify that there's only one ongoing branch from the `onramp1` lane, and
  // that it is lane `post0`.
  const api::Lane* onramp1 = GetLaneByJunctionId(*rg, "j:onramp1");
  EXPECT_NO_THROW(onramp1->GetOngoingBranches(api::LaneEnd::kStart));
  const api::LaneEndSet* lanes_beyond_onramp =
      onramp1->GetOngoingBranches(api::LaneEnd::kStart);
  EXPECT_EQ(1, lanes_beyond_onramp->size());
  EXPECT_EQ("l:post0", lanes_beyond_onramp->get(0).lane->id().id);

  // Verify that there's only one ongoing branch from the `pre0` lane, and
  // that it is also lane `post0`.
  const api::Lane* pre0 = GetLaneByJunctionId(*rg, "j:pre0");
  EXPECT_NO_THROW(pre0->GetOngoingBranches(api::LaneEnd::kStart));
  const api::LaneEndSet* lanes_beyond_pre0 =
      pre0->GetOngoingBranches(api::LaneEnd::kStart);
  EXPECT_EQ(1, lanes_beyond_pre0->size());
  EXPECT_EQ("l:post0", lanes_beyond_pre0->get(0).lane->id().id);

  // Initialize non-default road characteristics.
  const double kLaneWidth = 6.3;
  const double kDriveableWidth = 9.3;
  const RoadCharacteristics new_road{kLaneWidth, kDriveableWidth};

  std::unique_ptr<MonolaneOnrampMerge> new_merge_example(
      new MonolaneOnrampMerge(new_road));
  std::unique_ptr<const maliput::api::RoadGeometry> new_rg =
      new_merge_example->BuildOnramp();
  EXPECT_NE(nullptr, new_rg);

  // Check the correctness of the non-default parameters.
  EXPECT_EQ(
      kLaneWidth / 2.,
      new_rg->junction(0)->segment(0)->lane(0)->lane_bounds(kSPosition).r_max);
  EXPECT_EQ(kDriveableWidth / 2., new_rg->junction(0)
                                      ->segment(0)
                                      ->lane(0)
                                      ->driveable_bounds(kSPosition)
                                      .r_max);

  EXPECT_EQ(new_rg->id().id, "monolane-merge-example");
  EXPECT_EQ(new_rg->num_junctions(), 9);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
