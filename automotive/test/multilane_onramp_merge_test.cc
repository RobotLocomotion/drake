#include "drake/automotive/multilane_onramp_merge.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"

namespace drake {
namespace automotive {
namespace {

namespace api = maliput::api;

// @return `lane_id` Lane from `rg` RoadGeometry assuming `junction_id` Junction
// has only one segment in that Junction.
// @throws std::runtime_error When the Lane cannot be found.
const api::Lane* GetLaneByJunctionId(const api::RoadGeometry& rg,
                                     const std::string& junction_id,
                                     int lane_id) {
  for (int i = 0; i < rg.num_junctions(); ++i) {
    if (rg.junction(i)->id() == api::JunctionId(junction_id)) {
      return rg.junction(i)->segment(0)->lane(lane_id);
    }
  }
  throw std::runtime_error("No matching junction name in the road network");
}

// Check the correctness of the default parameters of a
// MultilaneRoadCharacteristics structure.
GTEST_TEST(MultilaneOnrampMergeTest, DefaultMultilaneRoadCharacteristics) {
  const double kZeroTolerance = 0.;
  const MultilaneRoadCharacteristics rc{};
  EXPECT_EQ(4., rc.lane_width);
  EXPECT_EQ(2., rc.left_shoulder);
  EXPECT_EQ(2., rc.right_shoulder);
  EXPECT_EQ(1., rc.lane_number);
  EXPECT_TRUE(api::test::IsHBoundsClose({0., 5.2}, rc.elevation_bounds,
                                        kZeroTolerance));
}

// Check the correctness of non-default parameters of a
// MultilaneRoadCharacteristics structure.
GTEST_TEST(MultilaneOnrampMergeTest, NonDefaultMultilaneRoadCharacteristics) {
  const double kZeroTolerance = 0.;
  const double kLaneWidth = 6.3;
  const double kLeftShoulder = 1.2;
  const double kRightShoulder = 2.1;
  const int kLaneNumber = 3;
  const MultilaneRoadCharacteristics rc(kLaneWidth, kLeftShoulder,
                                        kRightShoulder, kLaneNumber);
  EXPECT_EQ(kLaneWidth, rc.lane_width);
  EXPECT_EQ(kLeftShoulder, rc.left_shoulder);
  EXPECT_EQ(kRightShoulder, rc.right_shoulder);
  EXPECT_EQ(kLaneNumber, rc.lane_number);
  EXPECT_TRUE(api::test::IsHBoundsClose({0., 5.2}, rc.elevation_bounds,
                                        kZeroTolerance));
}

// Tests the default RoadGeometry of MultilaneOnrampMerge example generator.
GTEST_TEST(MultilaneOnrampMergeTest, TestDefaultAttributes) {
  const double kZeroTolerance = 0.;
  const double kSPosition = 0.;
  const int kFirstLane = 0;
  const MultilaneRoadCharacteristics rc{};

  // Create the road with default road characteristics.
  auto merge_example = std::make_unique<MultilaneOnrampMerge>();
  std::unique_ptr<const maliput::api::RoadGeometry> rg =
      merge_example->BuildOnramp();
  EXPECT_NE(nullptr, rg);

  EXPECT_EQ(rg->id(), api::RoadGeometryId("multilane-merge-example"));
  EXPECT_EQ(rg->num_junctions(), 9);
  for (int i = 0; i < rg->num_junctions(); ++i) {
    EXPECT_EQ(1, rg->junction(i)->num_segments());
    EXPECT_EQ(1, rg->junction(i)->segment(0)->num_lanes());
  }

  // Checks lane and driveable bounds.
  const api::RBounds kLaneBounds{-rc.lane_width / 2., rc.lane_width / 2.};
  EXPECT_TRUE(api::test::IsRBoundsClose(
      kLaneBounds,
      rg->junction(0)->segment(0)->lane(0)->lane_bounds(kSPosition),
      kZeroTolerance));

  const api::RBounds kDriveableBounds{-rc.lane_width / 2. - rc.left_shoulder,
                                      rc.lane_width / 2. + rc.right_shoulder};
  EXPECT_TRUE(api::test::IsRBoundsClose(
      kDriveableBounds,
      rg->junction(0)->segment(0)->lane(0)->driveable_bounds(kSPosition),
      kZeroTolerance));

  // Verify that there's only one ongoing branch from `onramp1_0`, and that it
  // is lane `post0_0`.
  EXPECT_NO_THROW(GetLaneByJunctionId(*rg, "j:onramp1", kFirstLane));
  const api::Lane* lane_onramp1 =
      GetLaneByJunctionId(*rg, "j:onramp1", kFirstLane);
  const api::LaneEndSet* lanes_beyond_onramp1 =
      lane_onramp1->GetOngoingBranches(api::LaneEnd::kStart);
  EXPECT_EQ(1, lanes_beyond_onramp1->size());
  const api::Lane* lane_beyond_onramp1 = lanes_beyond_onramp1->get(0).lane;
  EXPECT_EQ(api::LaneId("l:post0_0"), lane_beyond_onramp1->id());

  // Verify that the default branch of `onramp1_0` is `post0_0`.
  optional<api::LaneEnd> onramp1_default_lane_end =
      lane_onramp1->GetDefaultBranch(api::LaneEnd::kStart);
  EXPECT_TRUE(onramp1_default_lane_end);
  EXPECT_EQ(api::LaneId("l:post0_0"), onramp1_default_lane_end->lane->id());

  // Verify that there's only one ongoing branch from `pre0_0`, and that it is
  // also lane `post0_0`.
  EXPECT_NO_THROW(GetLaneByJunctionId(*rg, "j:pre0", kFirstLane));
  const api::Lane* lane_pre0 = GetLaneByJunctionId(*rg, "j:pre0", kFirstLane);
  const api::LaneEndSet* lanes_beyond_pre0 =
      lane_pre0->GetOngoingBranches(api::LaneEnd::kStart);
  EXPECT_EQ(1, lanes_beyond_pre0->size());
  const api::Lane* lane_beyond_pre0 = lanes_beyond_pre0->get(0).lane;
  EXPECT_EQ(api::LaneId("l:post0_0"), lane_beyond_pre0->id());

  // Verify that the default branch of `pre0_0` is `post0_0`.
  optional<api::LaneEnd> pre0_default_lane_end =
      lane_pre0->GetDefaultBranch(api::LaneEnd::kStart);
  EXPECT_TRUE(pre0_default_lane_end);
  EXPECT_EQ(api::LaneId("l:post0_0"), pre0_default_lane_end->lane->id());
}

// Tests a non default RoadGeometry of MultilaneOnrampMerge example generator.
GTEST_TEST(MultilaneOnrampMergeTest, TestNonDefaultAttributes) {
  const double kZeroTolerance = 0.;
  const double kSPosition = 0.;
  // Initialize non-default road characteristics.
  const double kLaneWidth = 6.3;
  const double kShoulder = 1.2;
  const int kLaneNumber = 3;
  const MultilaneRoadCharacteristics rc(kLaneWidth, kShoulder, kShoulder,
                                        kLaneNumber);

  auto merge_example = std::make_unique<MultilaneOnrampMerge>(rc);
  std::unique_ptr<const maliput::api::RoadGeometry> rg =
      merge_example->BuildOnramp();
  EXPECT_NE(nullptr, rg);

  // Check the correctness of the non-default parameters.
  const api::RBounds kLaneBounds{-rc.lane_width / 2., rc.lane_width / 2.};
  EXPECT_TRUE(api::test::IsRBoundsClose(
      kLaneBounds,
      rg->junction(0)->segment(0)->lane(0)->lane_bounds(kSPosition),
      kZeroTolerance));

  const api::RBounds kDriveableBounds{
      -rc.lane_width / 2. - rc.left_shoulder,
      rc.lane_width / 2. + 2. * kLaneWidth + rc.right_shoulder};
  EXPECT_TRUE(api::test::IsRBoundsClose(
      kDriveableBounds,
      rg->junction(0)->segment(0)->lane(0)->driveable_bounds(kSPosition),
      kZeroTolerance));

  EXPECT_EQ(rg->id(), api::RoadGeometryId("multilane-merge-example"));
  EXPECT_EQ(rg->num_junctions(), 9);
  for (int i = 0; i < rg->num_junctions(); ++i) {
    EXPECT_EQ(1, rg->junction(i)->num_segments());
    if (rg->id().string().find("j:post") != std::string::npos) {
      EXPECT_EQ(kLaneNumber, rg->junction(i)->segment(0)->num_lanes());
    } else if (rg->id().string().find("j:pre") != std::string::npos) {
      EXPECT_EQ(kLaneNumber / 2 + 1, rg->junction(i)->segment(0)->num_lanes());
    } else if (rg->id().string().find("j:onramp") != std::string::npos) {
      EXPECT_EQ(kLaneNumber / 2 + 1, rg->junction(i)->segment(0)->num_lanes());
    }
  }
}

// A test fixture to analyze the output RoadGeometry with different lane numbers
// for the generator.
class MultilaneOnrampVariableLaneTest : public ::testing::TestWithParam<int> {
 protected:
  const double kLaneWidth = 3.;
  const double kShoulder = 1.2;
};

// Checks the correct lane number assignment to each road.
TEST_P(MultilaneOnrampVariableLaneTest, TestLaneNumberAssigment) {
  const int lane_number = GetParam();
  const MultilaneRoadCharacteristics rc(kLaneWidth, kShoulder, kShoulder,
                                        lane_number);

  auto merge_example = std::make_unique<MultilaneOnrampMerge>(rc);
  std::unique_ptr<const maliput::api::RoadGeometry> rg =
      merge_example->BuildOnramp();
  EXPECT_NE(nullptr, rg);

  EXPECT_EQ(rg->id(), api::RoadGeometryId("multilane-merge-example"));
  EXPECT_EQ(rg->num_junctions(), 9);
  for (int i = 0; i < rg->num_junctions(); ++i) {
    EXPECT_EQ(1, rg->junction(i)->num_segments());
    if (rg->id().string().find("j:post") != std::string::npos) {
      EXPECT_EQ(lane_number, rg->junction(i)->segment(0)->num_lanes());
    } else if (rg->id().string().find("j:pre") != std::string::npos) {
      EXPECT_EQ(lane_number / 2 + 1, rg->junction(i)->segment(0)->num_lanes());
    } else if (rg->id().string().find("j:onramp") != std::string::npos) {
      EXPECT_EQ(lane_number / 2 + 1, rg->junction(i)->segment(0)->num_lanes());
    }
  }
}

// Checks the correct default BranchPoint assignment.
TEST_P(MultilaneOnrampVariableLaneTest, TestDefaultBranchAssigment) {
  const int lane_number = GetParam();
  const MultilaneRoadCharacteristics rc(kLaneWidth, kShoulder, kShoulder,
                                        lane_number);

  auto merge_example = std::make_unique<MultilaneOnrampMerge>(rc);
  std::unique_ptr<const maliput::api::RoadGeometry> rg =
      merge_example->BuildOnramp();
  EXPECT_NE(nullptr, rg);

  EXPECT_EQ(rg->id(), api::RoadGeometryId("multilane-merge-example"));
  EXPECT_EQ(rg->num_junctions(), 9);

  // Builds a Lane name from the name and the lane ID.
  auto lane_name = [](const std::string& name, int lane_id) {
    return std::string("l:") + name + std::string("_") +
           std::to_string(lane_id);
  };

  const int on_ramp_lane_number = lane_number / 2 + 1;
  // Verify that there's only one ongoing branch from `onramp1_X`, and that it
  // is lane `post0_Y`.
  for (int i = 0; i < on_ramp_lane_number; ++i) {
    EXPECT_NO_THROW(GetLaneByJunctionId(*rg, "j:onramp1", i));
    const api::Lane* lane_onramp1 = GetLaneByJunctionId(*rg, "j:onramp1", i);
    const api::LaneEndSet* lanes_beyond_onramp1 =
        lane_onramp1->GetOngoingBranches(api::LaneEnd::kStart);
    EXPECT_EQ(1, lanes_beyond_onramp1->size());
    const api::Lane* lane_beyond_onramp1 = lanes_beyond_onramp1->get(0).lane;
    EXPECT_EQ(
        api::LaneId(lane_name("post0", lane_number - on_ramp_lane_number + i)),
        lane_beyond_onramp1->id());

    // Verify that the default branch of `onramp1_X` is `post0_Y`.
    optional<api::LaneEnd> onramp1_default_lane_end =
        lane_onramp1->GetDefaultBranch(api::LaneEnd::kStart);
    EXPECT_TRUE(onramp1_default_lane_end);
    EXPECT_EQ(
        api::LaneId(lane_name("post0", lane_number - on_ramp_lane_number + i)),
        onramp1_default_lane_end->lane->id());
  }

  const int pre_lane_number = lane_number / 2 + 1;
  // Verify that there's only one ongoing branch from `pre0_X`, and that it is
  // also lane `post0_X`.
  for (int i = 0; i < pre_lane_number; ++i) {
    EXPECT_NO_THROW(GetLaneByJunctionId(*rg, "j:pre0", i));
    const api::Lane* lane_pre0 = GetLaneByJunctionId(*rg, "j:pre0", i);
    const api::LaneEndSet* lanes_beyond_pre0 =
        lane_pre0->GetOngoingBranches(api::LaneEnd::kStart);
    EXPECT_EQ(1, lanes_beyond_pre0->size());
    const api::Lane* lane_beyond_pre0 = lanes_beyond_pre0->get(0).lane;
    EXPECT_EQ(api::LaneId(lane_name("post0", i)), lane_beyond_pre0->id());

    // Verify that the default branch of `pre0_X` is `post0_X`.
    optional<api::LaneEnd> pre0_default_lane_end =
        lane_pre0->GetDefaultBranch(api::LaneEnd::kStart);
    EXPECT_TRUE(pre0_default_lane_end);
    EXPECT_EQ(api::LaneId(lane_name("post0", i)),
              pre0_default_lane_end->lane->id());
  }
}

INSTANTIATE_TEST_CASE_P(MultilaneOnrampMergeLaneNumber,
                        MultilaneOnrampVariableLaneTest, testing::Range(1, 6));

}  // namespace
}  // namespace automotive
}  // namespace drake
