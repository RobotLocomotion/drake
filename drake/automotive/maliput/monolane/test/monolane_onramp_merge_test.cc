#include "drake/automotive/maliput/monolane/monolane_onramp_merge.h"

#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace automotive {
namespace {

namespace mono = maliput::monolane;

// Tests the soundness of the MonolaneOnrampMerge example generator.
GTEST_TEST(MonolaneOnrampMergeTest, TestDefaultAndNonDefaultAttributes) {
  // Create the road with default road characteristics.
  std::unique_ptr<MonolaneOnrampMerge<double>> merge_example(
      new MonolaneOnrampMerge<double>);
  std::unique_ptr<const maliput::api::RoadGeometry> rg =
      merge_example->own_road_geometry();
  EXPECT_NE(nullptr, rg);

  // Initialize non-trivial road characteristics.
  const RoadCharacteristics new_road{6.3, 9.3};
  std::unique_ptr<MonolaneOnrampMerge<double>> new_merge_example(
      new MonolaneOnrampMerge<double>(new_road));
  std::unique_ptr<const maliput::api::RoadGeometry> new_rg =
      new_merge_example->own_road_geometry();
  EXPECT_NE(nullptr, new_rg);

  EXPECT_EQ(new_rg->id().id, "monolane-merge-example");
  EXPECT_EQ(new_rg->num_junctions(), 9);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
