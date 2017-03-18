#include "drake/automotive/monolane_onramp_merge.h"

#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace automotive {
namespace {

namespace mono = maliput::monolane;

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
  EXPECT_EQ(rg->num_junctions(), 8);

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
  EXPECT_EQ(new_rg->num_junctions(), 8);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
