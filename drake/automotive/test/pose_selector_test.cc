#include "drake/automotive/pose_selector.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/dragway/road_geometry.h"

namespace drake {
namespace automotive {
namespace pose_selector {
namespace {

using systems::rendering::PoseVector;
using systems::rendering::PoseBundle;

constexpr double kLaneLength{100.};
constexpr double kLaneWidth{4.};

constexpr double kEgoSPosition{10.};
constexpr double kEgoRPosition{-0.5 * kLaneWidth};
constexpr double kLeadingSPosition{31.};
constexpr double kTrailingSPosition{7.};
constexpr double kSOffset{4.};
constexpr int kFarAheadIndex{0};
constexpr int kJustAheadIndex{1};
constexpr int kJustBehindIndex{2};
constexpr int kFarBehindIndex{3};

static void SetDefaultPoses(PoseVector<double>* ego_pose,
                            PoseBundle<double>* traffic_poses) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == 4);
  DRAKE_DEMAND(kEgoSPosition > 0. && kLaneLength > kEgoSPosition);
  DRAKE_DEMAND(kLeadingSPosition > kEgoSPosition &&
               kLaneLength > kLeadingSPosition);
  DRAKE_DEMAND(kEgoSPosition > kTrailingSPosition && kTrailingSPosition > 0.);

  // Create poses for four traffic cars and one ego positioned in the right
  // lane,
  // interspersed as follows:
  //
  //     Far Behind   Just Behind     Ego     Just Ahead   Far Ahead
  //   |------o------------o-----------o----------o------------o-------------|
  //  s=0     3            7           10         31           35           100
  ego_pose->set_translation(Eigen::Translation3d(
      kEgoSPosition /* s */, kEgoRPosition /* r */, 0. /* h */));
  const Eigen::Translation3d translation_far_ahead(
      kLeadingSPosition + kSOffset /* s */, kEgoRPosition /* r */, 0. /* h */);
  const Eigen::Translation3d translation_just_ahead(
      kLeadingSPosition /* s */, kEgoRPosition /* r */, 0. /* h */);
  const Eigen::Translation3d translation_just_behind(
      kTrailingSPosition /* s */, kEgoRPosition /* r */, 0. /* h */);
  const Eigen::Translation3d translation_far_behind(
      kTrailingSPosition - kSOffset /* s */, kEgoRPosition /* r */, 0. /* h */);
  traffic_poses->set_pose(kFarAheadIndex,
                          Eigen::Isometry3d(translation_far_ahead));
  traffic_poses->set_pose(kJustAheadIndex,
                          Eigen::Isometry3d(translation_just_ahead));
  traffic_poses->set_pose(kJustBehindIndex,
                          Eigen::Isometry3d(translation_just_behind));
  traffic_poses->set_pose(kFarBehindIndex,
                          Eigen::Isometry3d(translation_far_behind));
}

GTEST_TEST(PoseSelectorTest, DragwayTest) {
  // Create a straight road, two lanes wide, in which the two s-r
  // Lane-coordinate frames are aligned with the x-y world coordinates, with s_i
  // = 0 for the i-th lane, i ∈ {0, 1}, corresponds to x = 0, and r_i = 0
  // corresponds to y = (i - 0.5) * kLaneWidth.  See sketch below.
  //
  // +y ^
  //    | -  -  -  -  -  -  -  -   <-- lane 1 (y_1)
  //  0 |-----------------------> +x
  //    | -  -  -  -  -  -  -  -   <-- lane 0 (y_0)
  const int kNumLanes{2};
  const maliput::dragway::RoadGeometry road(
      maliput::api::RoadGeometryId({"Test Dragway"}), kNumLanes, kLaneLength,
      kLaneWidth, 0. /* shoulder width */);
  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(4);

  // Define the default poses.
  SetDefaultPoses(&ego_pose, &traffic_poses);

  // Calculate the current road position and use it to determine the ego car's
  // lane.
  const maliput::api::RoadPosition& ego_position =
      CalcRoadPosition(road, ego_pose.get_isometry());

  maliput::api::RoadPosition leading_position;
  maliput::api::RoadPosition trailing_position;
  std::tie(leading_position, trailing_position) =
      FindClosestPair(road, ego_pose, traffic_poses);

  // Verifies that we are on the road and that the correct car was identified.
  EXPECT_EQ(kLeadingSPosition, leading_position.pos.s);
  EXPECT_EQ(kTrailingSPosition, trailing_position.pos.s);

  // Test that we get the same result when just the leading car is returned.
  const maliput::api::RoadPosition& traffic_position =
      FindClosestLeading(road, ego_pose, traffic_poses);
  EXPECT_EQ(kLeadingSPosition, traffic_position.pos.s);

  // Peer into the adjacent lane to the left.
  std::tie(leading_position, trailing_position) = FindClosestPair(
      road, ego_pose, traffic_poses, ego_position.lane->to_left());

  // Expect to see no cars in the left lane.
  EXPECT_EQ(std::numeric_limits<double>::infinity(), leading_position.pos.s);
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), trailing_position.pos.s);

  // Bump the "just ahead" car into the lane to the left.
  Isometry3<double> isometry_just_ahead =
      traffic_poses.get_pose(kJustAheadIndex);
  isometry_just_ahead.translation().y() += kLaneWidth;
  traffic_poses.set_pose(kJustAheadIndex, isometry_just_ahead);
  std::tie(leading_position, std::ignore) =
      FindClosestPair(road, ego_pose, traffic_poses);

  // Expect the "far ahead" car to be identified.
  EXPECT_EQ(kLeadingSPosition + kSOffset, leading_position.pos.s);

  // Bump the "far ahead" car into the lane to the left.
  Isometry3<double> isometry_far_ahead = traffic_poses.get_pose(kFarAheadIndex);
  isometry_far_ahead.translation().y() += kLaneWidth;
  traffic_poses.set_pose(kFarAheadIndex, isometry_far_ahead);
  std::tie(leading_position, std::ignore) =
      FindClosestPair(road, ego_pose, traffic_poses);

  // Looking forward, we expect there to be no car in sight.
  EXPECT_EQ(std::numeric_limits<double>::infinity(), leading_position.pos.s);

  // Peer into the adjacent lane to the left.
  std::tie(leading_position, trailing_position) = FindClosestPair(
      road, ego_pose, traffic_poses, ego_position.lane->to_left());

  // Expect there to be no car behind on the immediate left and the "just ahead"
  // car to be leading.
  EXPECT_EQ(kLeadingSPosition, leading_position.pos.s);
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), trailing_position.pos.s);
}

}  // namespace
}  // namespace pose_selector
}  // namespace automotive
}  // namespace drake
